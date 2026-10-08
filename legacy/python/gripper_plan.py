"""Deterministic face-move compiler for two perpendicular grippers.

Angles are right-hand positive about outward axes A=+X and B=-Y.
Raw macros start/end with both grippers closed and yaw=0. Mounts are fixed.
An independent post-pass removes redundant regrips without changing rotations.
"""
from dataclasses import asdict, dataclass
import math

import numpy as np

from rubiks_cube import FACES, parse_move, rotation

AXES = {'A': np.array([1, 0, 0]), 'B': np.array([0, -1, 0])}
JAW_CLOSED = 115.
# Fixed lookup, not a search for a shortest robot path.
PRESENT_TO_A = {
    (1, 0, 0): [], (-1, 0, 0): [('B', 2)],
    (0, 0, 1): [('B', -1)], (0, 0, -1): [('B', 1)],
    (0, 1, 0): [('A', 1), ('B', -1)],
    (0, -1, 0): [('A', 1), ('B', 1)],
}


@dataclass(frozen=True)
class Action:
    kind: str
    hand: str = ''
    target: float = 0.
    mode: str = ''
    move: str = ''

    def as_dict(self):
        return asdict(self)


def wrist_rotation(hand, angle):
    if not math.isfinite(angle) or not np.isclose(angle/(math.pi/2), round(angle/(math.pi/2))):
        raise ValueError('Cube reorientation must use complete quarter turns')
    axis = int(np.flatnonzero(AXES[hand])[0])
    return np.rint(rotation(axis, AXES[hand][axis]*angle)).astype(int)


def checked_orientation(orientation):
    if orientation is None:
        return np.eye(3, dtype=int)
    q = np.asarray(orientation)
    if (q.shape != (3, 3) or not np.isin(q, [-1, 0, 1]).all()
            or not np.array_equal(q.T@q, np.eye(3)) or round(np.linalg.det(q)) != 1):
        raise ValueError('Expected one of the 24 proper cube orientations')
    return q.astype(int).copy()


def clear_hand(hand):
    return [Action('release', hand), Action('jaw', hand, 0)]


def engage_hand(hand):
    return [Action('jaw', hand, JAW_CLOSED), Action('grasp', hand, mode='core')]


def reorient(hand, quarters):
    other = 'B' if hand == 'A' else 'A'
    return (clear_hand(other)
            + [Action('yaw', hand, quarters*math.pi/2, mode='whole')]
            + engage_hand(other) + clear_hand(hand)
            + [Action('yaw', hand, 0, mode='empty')]
            + engage_hand(hand))


def compile_moves(moves, orientation=None):
    q = checked_orientation(orientation)
    plan = []
    for move in moves:
        face, count = parse_move(move)
        axis, sign = FACES[face]
        normal = q@np.eye(3, dtype=int)[axis]*sign
        for hand, quarters in PRESENT_TO_A[tuple(normal)]:
            plan.extend(reorient(hand, quarters))
            q = wrist_rotation(hand, quarters*math.pi/2)@q
        # B holds the core. A changes from whole-cube grip to active-layer grip.
        plan.extend([Action('release', 'A'), Action('layer_unlock', 'A', move=move),
                     Action('grasp', 'A', mode='face', move=move),
                     Action('yaw', 'A', -count*math.pi/2, mode='face', move=move),
                     Action('layer_lock', 'A', move=move)])
        plan.extend(clear_hand('A'))
        plan.append(Action('yaw', 'A', 0, mode='empty'))
        plan.extend(engage_hand('A'))
        plan.append(Action('checkpoint', move=move))
    replay_plan(plan, orientation)
    return plan


def replay_plan(plan, orientation=None):
    """Check support, clearance, orientation and turn semantics without physics."""
    q = checked_orientation(orientation)
    grasp = {'A': 'core', 'B': 'core'}
    jaw = {'A': JAW_CLOSED, 'B': JAW_CLOSED}
    yaw = {'A': 0., 'B': 0.}
    unlocked = None
    face_turned = False
    moves = []
    for action in plan:
        h = action.hand
        other = 'B' if h == 'A' else 'A'
        if h and h not in AXES:
            raise ValueError('Unknown gripper')
        if action.kind == 'release':
            if grasp[other] != 'core':
                raise ValueError('Releasing would remove cube support')
            grasp[h] = None
        elif action.kind == 'jaw':
            if action.target == 0 and grasp[h] is not None:
                raise ValueError('Release before opening')
            jaw[h] = action.target
        elif action.kind == 'grasp':
            if jaw[h] != JAW_CLOSED:
                raise ValueError('Grasp requires closed jaws')
            if action.mode not in ('core', 'face'):
                raise ValueError('Unknown grasp mode')
            if action.mode == 'face' and (unlocked != action.move or grasp[other] != 'core'):
                raise ValueError('Face grasp requires an unlocked layer and core support')
            grasp[h] = action.mode
        elif action.kind == 'layer_unlock':
            face, _ = parse_move(action.move)
            axis, sign = FACES[face]
            if unlocked or grasp[h] or grasp[other] != 'core':
                raise ValueError('Layer release needs independent core support')
            if not np.array_equal(q@np.eye(3, dtype=int)[axis]*sign, AXES[h]):
                raise ValueError('Requested face is not presented to this wrist')
            unlocked, face_turned = action.move, False
        elif action.kind == 'yaw':
            delta = action.target-yaw[h]
            if action.mode == 'whole':
                if (grasp[h] != 'core' or grasp[other] is not None or
                        jaw[other] != 0 or unlocked):
                    raise ValueError('Whole rotation needs sole support and other hand clear')
                q = wrist_rotation(h, delta)@q
            elif action.mode == 'face':
                _, count = parse_move(action.move)
                if (grasp[h] != 'face' or grasp[other] != 'core'
                        or unlocked != action.move or not np.isclose(delta, -count*math.pi/2)):
                    raise ValueError('Incorrect face rotation or missing support')
                face_turned = True
            elif action.mode == 'empty':
                if grasp[h] is not None or jaw[h] != 0:
                    raise ValueError('Empty wrist reset requires fully open jaws for clearance')
            else:
                raise ValueError('Unknown rotation mode')
            yaw[h] = action.target
        elif action.kind == 'layer_lock':
            if unlocked != action.move or not face_turned:
                raise ValueError('Cannot lock an unexecuted layer move')
            moves.append(action.move)
            unlocked = None
        elif action.kind == 'checkpoint':
            if 'core' not in grasp.values() or any(yaw.values()) or unlocked:
                raise ValueError('Checkpoint requires cube support and zeroed wrists')
        else:
            raise ValueError(f'Unknown action: {action.kind}')
    if unlocked:
        raise ValueError('Plan ends with a layer unlocked')
    return {'moves': moves, 'orientation': q.tolist(),
            'grasped': [h for h in AXES if grasp[h] is not None], 'yaw_rad': yaw,
            'jaw_command': jaw}


def optimize_plan(plan, orientation=None):
    """Post-process an already compiled plan; keep every actual wrist rotation.

    Remove close/grasp/checkpoint/release/open sandwiches and transient core
    grasps. A checkpoint is retained, but can run while one hand safely holds the
    cube. Every rewrite must pass the support state machine and preserve the
    complete returned cube/hand state. Input actions are immutable and untouched.
    """
    expected = replay_plan(plan, orientation)
    result = list(plan)
    while True:
        changed = False
        for i, action in enumerate(result):
            start_grasp = i
            close = action.kind == 'jaw' and action.target == JAW_CLOSED
            if close:
                start_grasp += 1
            if start_grasp >= len(result):
                continue
            grasp = result[start_grasp]
            if grasp.kind != 'grasp' or grasp.mode != 'core' or (close and grasp.hand != action.hand):
                continue
            j = start_grasp+1
            while j < len(result) and result[j].kind == 'checkpoint':
                j += 1
            if j >= len(result) or result[j].kind != 'release' or result[j].hand != grasp.hand:
                continue
            end = j+1
            if close:
                if (end >= len(result) or result[end].kind != 'jaw'
                        or result[end].hand != grasp.hand or result[end].target != 0):
                    continue
                end += 1
            candidate = result[:i]+result[start_grasp+1:j]+result[end:]
            try:
                equivalent = replay_plan(candidate, orientation) == expected
            except ValueError:
                equivalent = False
            if equivalent:
                result, changed = candidate, True
                break
        if not changed:
            break
    # Removing a sandwich may expose repeated open/close commands. They do not
    # change geometry or support; contact confirmation still happens at grasp.
    jaw = {'A': JAW_CLOSED, 'B': JAW_CLOSED}
    compact = []
    for action in result:
        if action.kind == 'jaw':
            if jaw[action.hand] == action.target:
                continue
            jaw[action.hand] = action.target
        compact.append(action)
    if replay_plan(compact, orientation) != expected:
        raise RuntimeError('Optimization changed cube or final gripper state')
    return compact
