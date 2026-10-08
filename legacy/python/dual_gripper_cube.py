"""Execute Kociemba moves with perpendicular Robotiq grippers in MuJoCo.

Grasps transmit forces only through frictional contacts. Internal cube motors
are disabled before robot execution; wrist actuator torque moves the cube.
"""
import argparse
import json
import math
from pathlib import Path

import mujoco
import numpy as np

from cube_solver import facelets_from_cube, solve_facelets, SOLVED
from dual_gripper_model import build_scene
from gripper_plan import (Action, AXES, JAW_CLOSED, compile_moves, optimize_plan,
                          replay_plan, wrist_rotation)
from rubiks_cube import RubiksCube, FACES, ROOT, PITCH, SCRAMBLE, parse_move, rotation


class DualGripperCube(RubiksCube):
    def __init__(self, speed=1., wrist_speed=None, jaw_speed=None):
        if not math.isfinite(speed) or speed <= 0:
            raise ValueError("speed must be finite and positive")
        self.speed = float(speed)
        self.wrist_speed = float(speed if wrist_speed is None else wrist_speed)
        self.jaw_speed = float(speed if jaw_speed is None else jaw_speed)
        if not all(math.isfinite(v) and v > 0 for v in [self.wrist_speed, self.jaw_speed]):
            raise ValueError('Independent speeds must be finite and positive')
        self.fast_jaws = jaw_speed is not None and jaw_speed > 1
        self.motion_checks = []
        super().__init__(build_scene(fast_jaws=self.fast_jaws))
        self.grasped = {'A': None, 'B': None}
        self.orientation = np.eye(3, dtype=int)
        self.current_action = None
        self.executed = []
        self.grasp_checks = []
        self.robot_ready = False
        self.cube_bodies = set(self.piece_ids) | {self.model.body(f'center_{f}').id for f in FACES}
        self.required_open_gap = math.sqrt(2)*(3*PITCH+.0002)+.004
        self.clearance_report = {'rotation_steps_checked': 0, 'forbidden_contacts': 0,
                                 'min_open_gap_m': None, 'required_open_gap_m': self.required_open_gap}
        self.geom_hand = np.array([
            0 if self.model.body(body).name.startswith('A_') else
            1 if self.model.body(body).name.startswith('B_') else -1
            for body in self.model.geom_bodyid])
        self.geom_cube = np.isin(self.model.geom_bodyid, list(self.cube_bodies))
        self.advance(.3)

    def open_gap(self, hand):
        """Actual inner aperture projected between the two oriented tip boxes."""
        right = self.model.geom(f'{hand}_right_tip').id
        left = self.model.geom(f'{hand}_left_tip').id
        delta = self.data.geom_xpos[right]-self.data.geom_xpos[left]
        direction = delta/np.linalg.norm(delta)
        radii = sum(np.abs(self.data.geom_xmat[g].reshape(3, 3).T@direction)
                    @ self.model.geom_size[g] for g in [right, left])
        return float(np.linalg.norm(delta)-radii)

    def _wait_open(self, hand, callback=None):
        for _ in range(100):
            if self.open_gap(hand) >= self.required_open_gap:
                return
            self.advance(.01, callback)
        raise RuntimeError(f'{hand}: insufficient open aperture for in-place rotation')

    def _check_rotation_clearance(self, action, callback=None):
        open_hand = (action.hand if action.mode == 'empty' else
                     ('B' if action.hand == 'A' else 'A') if action.mode == 'whole' else None)
        if open_hand:
            gap = self.open_gap(open_hand)
            previous = self.clearance_report['min_open_gap_m']
            self.clearance_report['min_open_gap_m'] = gap if previous is None else min(previous, gap)
            if gap < self.required_open_gap:
                raise RuntimeError(f'{open_hand}: open aperture fell below rotation clearance')
        if self.data.ncon:
            g1, g2 = self.data.contact.geom.T
            h1, h2 = self.geom_hand[g1], self.geom_hand[g2]
            forbidden = (h1 >= 0) & (h2 >= 0) & (h1 != h2)
            if open_hand:
                which = 0 if open_hand == 'A' else 1
                forbidden |= ((h1 == which) & self.geom_cube[g2]) | ((h2 == which) & self.geom_cube[g1])
            bad = forbidden & (self.data.contact.dist < 0)
            if np.any(bad):
                self.clearance_report['forbidden_contacts'] += int(np.sum(bad))
                pair = self.data.contact.geom[np.flatnonzero(bad)[0]]
                names = [self.model.geom(int(g)).name or self.model.body(self.model.geom_bodyid[g]).name for g in pair]
                raise RuntimeError(f'In-place rotation collision: {names}')
        self.clearance_report['rotation_steps_checked'] += 1
        if callback:
            callback(self)

    def pad_contacts(self, hand):
        """Return contacts with positive normal force on each extension pad."""
        result = {}
        for side in ['right', 'left']:
            tip = self.model.geom(f'{hand}_{side}_tip').id
            for index, contact in enumerate(self.data.contact):
                g1, g2 = contact.geom
                other = g2 if g1 == tip else g1 if g2 == tip else -1
                if other < 0 or self.model.geom_bodyid[other] not in self.cube_bodies:
                    continue
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, index, force)
                if force[0] > .01:
                    result.setdefault(side, []).append({'normal_force_n': float(force[0]),
                                                       'body': int(self.model.geom_bodyid[other])})
        return result

    def _move(self, actuator_name, target, duration, callback=None, joint_name=None):
        if self.fast_jaws and not joint_name:
            return self._fast_jaw(actuator_name, target, duration, callback)
        actuator = self.model.actuator(actuator_name).id
        start = float(self.data.ctrl[actuator])
        gain = self.model.actuator_gainprm[actuator, 0]
        damping = -self.model.actuator_biasprm[actuator, 2]
        duration /= self.wrist_speed if joint_name else self.jaw_speed
        steps = max(1, round(duration/self.model.opt.timestep))
        duration = steps*self.model.opt.timestep
        began = float(self.data.time)
        peak_velocity = 0.
        peak_force = 0.
        initial_gap = self.open_gap(actuator_name[0]) if not joint_name else None
        for k in range(1, steps+1):
            u = k/steps
            smooth = u**3*(10+u*(-15+6*u))
            speed = 30*u*u*(1-u)**2/duration
            # Velocity feed-forward through the PD position reference.
            ff = damping/gain*(target-start)*speed if joint_name and gain else 0
            self.data.ctrl[actuator] = start+(target-start)*smooth+ff
            self.step(callback)
            peak_force = max(peak_force, abs(float(self.data.actuator_force[actuator])))
            if joint_name:
                dof = self.model.joint(joint_name).dofadr[0]
                peak_velocity = max(peak_velocity, abs(float(self.data.qvel[dof])))
        motion_end = float(self.data.time)
        actual_end = float(self.data.qpos[self.model.joint(joint_name).qposadr[0]]) if joint_name else self.open_gap(actuator_name[0])
        self.motion_checks.append({"actuator": actuator_name, "start_time_s": began,
            "duration_s": motion_end-began, "target": target, "actual_at_ramp_end": actual_end,
            "peak_joint_velocity_rad_s": peak_velocity if joint_name else None,
            "peak_actuator_force": peak_force, "initial_aperture_m": initial_gap})
        self.data.ctrl[actuator] = target
        self.advance(.15, callback)
        if joint_name:
            address = self.model.joint(joint_name).qposadr[0]
            tolerance = .006 if joint_name.endswith('yaw') else .001
            actual = float(self.data.qpos[address])
            if abs(actual-target) > tolerance:
                raise RuntimeError(f'{joint_name} failed tracking: {actual:.6f}, target {target:.6f}')

    def _fast_jaw(self, actuator_name, target, duration, callback=None):
        """High-bandwidth finger actuator with contact-triggered effort limiting."""
        a = self.model.actuator(actuator_name).id
        hand = actuator_name[0]
        ratio = .8/255
        kp, kv = 100*self.jaw_speed**2, 10*self.jaw_speed
        self.model.actuator_gainprm[a, 0] = kp*ratio
        self.model.actuator_biasprm[a, 1:3] = [-kp, -kv]
        self.model.actuator_forcerange[a] = [-40, 40]
        initial = float(self.data.actuator_length[a])
        goal = 0. if target == 0 else .332
        dt = self.model.opt.timestep
        duration = max(dt, round(duration/self.jaw_speed/dt)*dt)
        began = float(self.data.time)
        gap0 = previous_gap = self.open_gap(hand)
        first_reached = None
        peak_speed = peak_force = 0.
        ramp_gap = None
        contact = False
        stable = 0
        for k in range(round((duration+.1)/dt)):
            t = (k+1)*dt
            u = min(1., t/duration)
            p = u**3*(10+u*(-15+6*u))
            v = 30*u*u*(1-u)**2/duration if u < 1 else 0.
            reference = initial+(goal-initial)*p+kv/kp*(goal-initial)*v
            if target and (contact or u == 1):
                self.model.actuator_forcerange[a] = [-5, 5]
                reference = .345
            self.data.ctrl[a] = reference/ratio
            self.step(callback)
            gap = self.open_gap(hand)
            if ramp_gap is None and t >= duration:
                ramp_gap = gap
            peak_speed = max(peak_speed, abs(gap-previous_gap)/dt)
            peak_force = max(peak_force, abs(float(self.data.actuator_force[a])))
            previous_gap = gap
            pads = self.pad_contacts(hand) if target else {}
            contact |= bool(pads)
            ready = len(pads) == 2 if target else gap >= self.required_open_gap
            if ready and first_reached is None:
                first_reached = float(self.data.time)-began
            stable = stable+1 if ready else 0
            if t >= duration and stable*dt >= .004:
                break
        else:
            raise RuntimeError(f'{hand}: fast jaw failed contact/clearance feedback')
        self.motion_checks.append({'actuator': actuator_name, 'start_time_s': began,
            'duration_s': float(self.data.time)-began, 'reference_duration_s': duration,
            'target': target, 'actual_at_ramp_end': ramp_gap, 'final_aperture_m': gap,
            'stable_reached_s': float(self.data.time)-began-.004+dt,
            'initial_aperture_m': gap0, 'first_reached_s': first_reached,
            'peak_aperture_velocity_m_s': peak_speed, 'peak_actuator_force': peak_force,
            'peak_joint_velocity_rad_s': None})

    def _grasp(self, hand, mode, move='', callback=None):
        # Robotiq's compliant four-bar linkage can settle asymmetrically after a
        # handoff. Wait on measured two-sided contact, not a fixed jaw timeout.
        stable = 0
        for _ in range(100):
            stable = stable+1 if len(self.pad_contacts(hand)) == 2 else 0
            if stable >= 4:
                break
            self.advance(.002 if self.fast_jaws else .01, callback)
        contacts = self.pad_contacts(hand)
        if stable < 4:
            raise RuntimeError(f'{hand}: both fingertips must contact the cube before grasp; got {contacts}')
        self.grasped[hand] = mode
        self.grasp_checks.append({'hand': hand, 'mode': mode, 'contacts': contacts,
                                  'time_s': float(self.data.time)})
        self.advance(.002 if self.fast_jaws else .05, callback)

    def _release(self, hand):
        other = 'B' if hand == 'A' else 'A'
        fixture = self.data.eq_active[self.model.equality('loading_fixture').id]
        if self.grasped[other] != 'core' and not fixture:
            raise RuntimeError('Refusing release without independent cube support')
        self.grasped[hand] = None

    def initialize_grasps(self, callback=None):
        if self.robot_ready:
            raise RuntimeError('Grippers are already initialized')
        for face in FACES:
            eq = self.model.equality(f'lock_{face}').id
            self.model.eq_data[eq, 0] = self.targets[face]
            self.data.eq_active[eq] = True
            motor = self.model.actuator(f'drive_{face}').id
            self.model.actuator_gainprm[motor] = 0
            self.model.actuator_biasprm[motor] = 0
            self.data.ctrl[motor] = 0
        for hand in AXES:
            self.current_action = Action('jaw', hand, JAW_CLOSED).as_dict()
            self._move(f'{hand}_fingers_actuator', JAW_CLOSED, .35, callback)
            self._grasp(hand, 'core', callback=callback)
        self.data.eq_active[self.model.equality('loading_fixture').id] = False
        self.robot_ready = True
        self.advance(.15, callback)

    def _unlock(self, move):
        if self.active_face:
            raise RuntimeError('A layer is already unlocked')
        face, _ = parse_move(move)
        axis, sign = FACES[face]
        core = self.model.body('core').id
        normal = self.data.xmat[core].reshape(3, 3)@np.eye(3)[axis]*sign
        if np.linalg.norm(normal-AXES['A']) > .025:
            raise RuntimeError('Face is not aligned with the A wrist')
        self.active_face = face
        for index in np.flatnonzero(self.slots[:, axis] == sign):
            self._attach(index, f'center_{face}')
        self.data.eq_active[self.model.equality(f'lock_{face}').id] = False

    def _lock(self, move, callback=None):
        face, count = parse_move(move)
        axis, sign = FACES[face]
        expected = self.targets[face]-count*math.pi/2
        angle = float(self.data.qpos[self.model.joint(f'hinge_{face}').qposadr[0]])
        if abs(angle-expected) > .02:
            raise RuntimeError(f'{move}: physical layer did not turn: {angle}, expected {expected}')
        selected = np.flatnonzero(self.slots[:, axis] == sign)
        r = np.rint(rotation(axis, -sign*count*math.pi/2)).astype(int)
        self.slots[selected] = self.slots[selected]@r.T
        self.orientations[selected] = r@self.orientations[selected]
        self.targets[face] = expected
        eq = self.model.equality(f'lock_{face}').id
        self.model.eq_data[eq, 0] = expected
        self.data.eq_active[eq] = True
        for index in selected:
            self._attach(index, 'core', ideal=True)
        self.advance(.15, callback)
        self.active_face = None
        self.history.append(move)

    def execute(self, plan, callback=None):
        if not self.robot_ready:
            raise RuntimeError('Initialize both grasps before executing a plan')
        replay_plan(plan, self.orientation)
        for index, action in enumerate(plan):
            self.current_action = {'index': index, 'total': len(plan), **action.as_dict()}
            h = action.hand
            if action.kind == 'release':
                self._release(h)
            elif action.kind == 'jaw':
                self._move(f'{h}_fingers_actuator', action.target, .3, callback)
                if action.target == 0:
                    self._wait_open(h, callback)
            elif action.kind == 'grasp':
                self._grasp(h, action.mode, action.move, callback)
            elif action.kind == 'yaw':
                old = float(self.data.ctrl[self.model.actuator(f'{h}_yaw_drive').id])
                duration = 1.2 if abs(action.target-old) > 2 else .85
                observe = lambda _: self._check_rotation_clearance(action, callback)
                self._move(f'{h}_yaw_drive', action.target, duration, observe, f'{h}_yaw')
                if action.mode == 'whole':
                    self.orientation = wrist_rotation(h, action.target-old)@self.orientation
            elif action.kind == 'layer_unlock':
                self._unlock(action.move)
            elif action.kind == 'layer_lock':
                self._lock(action.move, callback)
            elif action.kind == 'checkpoint':
                position, angle = self.pose_error()
                if position > .0002 or angle > .02:
                    raise RuntimeError(f'Cube pose mismatch after {action.move}: {position}, {angle}')
                if self.data.warning.number.any():
                    raise RuntimeError('MuJoCo reported a numerical warning')
            self.executed.append(action.as_dict())

    def robot_report(self):
        return {**self.report(), 'facelets': facelets_from_cube(self),
                'orientation': self.orientation.tolist(), 'grasped': self.grasped.copy(),
                'primitive_actions': len(self.executed), 'grasp_checks': len(self.grasp_checks),
                'cube_motor_force_max': float(np.max(np.abs(self.data.actuator_force[:6]))),
                'core_position_m': self.data.xpos[self.model.body('core').id].tolist(),
                'motion_speed_requested': self.speed, 'wrist_speed_requested': self.wrist_speed,
                'jaw_speed_requested': self.jaw_speed, 'fast_parallel_fingers': self.fast_jaws,
                'motion_checks': self.motion_checks,
                'grasp_model': 'actuator-driven jaws with friction-only contact; no grasp weld',
                'layout': 'A +X, B -Y; fixed mounts, no translation joints; open-jaw in-place reset',
                'rotation_clearance': self.clearance_report.copy()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scramble', default=' '.join(SCRAMBLE))
    parser.add_argument('--output', type=Path, default=ROOT/'output/dual_gripper_friction')
    parser.add_argument('--speed', type=float, default=1., help='Motion reference speed multiplier; actuator limits are unchanged')
    parser.add_argument('--wrist-speed', type=float, help='Independent wrist trajectory multiplier')
    parser.add_argument('--jaw-speed', type=float, help='Independent finger multiplier; >1 enables tuned parallel-finger drive')
    args = parser.parse_args()
    cube = DualGripperCube(speed=args.speed, wrist_speed=args.wrist_speed, jaw_speed=args.jaw_speed)
    for move in args.scramble.split():
        cube.turn(move)
    state = facelets_from_cube(cube)
    solution = solve_facelets(state)
    raw_plan = compile_moves(solution)
    plan = optimize_plan(raw_plan)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'scene.xml').write_text(cube.xml)
    (args.output/'plan.json').write_text(json.dumps({
        'initial_facelets': state, 'solver': 'muodov/kociemba 1.2.1', 'solution': solution,
        'precondition': 'both grippers closed and grasping core, yaw=0; fixed mounts; loading fixture released',
        'units': 'yaw radians right-hand about A=+X/B=-Y; jaw Robotiq command 0..255',
        'unoptimized_actions': [a.as_dict() for a in raw_plan],
        'actions': [a.as_dict() for a in plan]}, indent=2))
    print('Solver:', ' '.join(solution), flush=True)
    print('Primitive actions:', len(plan), flush=True)
    cube.initialize_grasps()
    cube.execute(plan)
    cube.advance(.5)
    report = cube.robot_report()
    if report['facelets'] != SOLVED:
        raise RuntimeError('Physical cube did not finish solved')
    (args.output/'verification.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
