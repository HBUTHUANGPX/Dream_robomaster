"""A motor-driven MuJoCo cube with switchable welds, not pose animation.

Run: uv run python rubiks_cube.py --viewer
"""
from pathlib import Path
import argparse
import itertools
import json
import math
import queue
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent
PITCH = .020
ORIGIN = np.array([0., 0., .105])
FACES = {'R': (0, 1), 'L': (0, -1), 'U': (2, 1),
         'D': (2, -1), 'F': (1, -1), 'B': (1, 1)}
COLORS = {(0, 1): '.82 .035 .055 1', (0, -1): '1 .30 .015 1',
          (2, 1): '.94 .95 .96 1', (2, -1): '1 .77 .015 1',
          (1, -1): '.015 .63 .27 1', (1, 1): '.025 .19 .85 1'}
SCRAMBLE = "R U F' L2 D B R' U2 F D'".split()


def vec(values):
    return ' '.join(str(float(x)) for x in values)


def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def parse_move(move):
    if (not isinstance(move, str) or len(move) not in (1, 2)
            or move[0] not in FACES
            or (len(move) == 2 and move[1] not in ("'", '2'))):
        raise ValueError(f'Invalid face move: {move!r}; use R, U\', F2, etc.')
    return move[0], (-1 if move.endswith("'") else 2 if move.endswith('2') else 1)


def inverse_moves(moves):
    inverse = []
    for move in reversed(moves):
        face, count = parse_move(move)
        inverse.append(face + ('2' if count == 2 else "'" if count == 1 else ''))
    return inverse


def rotation(axis, angle):
    unit = np.eye(3)[axis]
    cross = np.array([[0, -unit[2], unit[1]], [unit[2], 0, -unit[0]],
                      [-unit[1], unit[0], 0]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)


def piece_geometry(body, slot, offset):
    add(body, 'geom', type='box', pos=vec(offset), size='.00965 .00965 .00965',
        mass='.006', rgba='.025 .029 .035 1')
    for axis, sign in enumerate(slot):
        if sign == 0:
            continue
        pos = np.array(offset, dtype=float)
        pos[axis] += sign*.00985
        size = np.full(3, .00825)
        size[axis] = .00025
        add(body, 'geom', type='box', pos=vec(pos), size=vec(size), mass='0',
            rgba=COLORS[(axis, int(sign))], contype='0', conaffinity='0')


def build_xml():
    root = ET.Element('mujoco', model='Rubiks cube - switchable weld constraints')
    add(root, 'compiler', angle='radian', autolimits='true')
    add(root, 'option', timestep='.001', integrator='implicitfast',
        solver='Newton', iterations='80', tolerance='1e-10', gravity='0 0 -9.81')
    default = add(root, 'default')
    # Cube-vs-cube contact disabled; external geoms with bit 2 still collide.
    add(default, 'geom', contype='1', conaffinity='2', friction='.8 .002 .0001')
    add(default, 'equality', solref='.004 1', solimp='.99 .99 .001')
    visual = add(root, 'visual')
    add(visual, 'global', offwidth='1280', offheight='720')
    add(visual, 'quality', shadowsize='4096', offsamples='4')
    add(visual, 'headlight', ambient='.32 .32 .32', diffuse='.5 .5 .5', specular='.15 .15 .15')
    add(visual, 'rgba', haze='.055 .065 .09 1')
    asset = add(root, 'asset')
    add(asset, 'texture', name='sky', type='skybox', builtin='gradient',
        rgb1='.035 .045 .07', rgb2='.10 .13 .18', width='512', height='3072')
    add(asset, 'material', name='floor_mat', rgba='.13 .16 .21 1',
        reflectance='.15', specular='.25', shininess='.4')
    world = add(root, 'worldbody')
    add(world, 'light', pos='.15 -.2 .5', dir='-.25 .3 -1',
        diffuse='.85 .85 .85', castshadow='true')
    add(world, 'light', pos='-.2 -.1 .25', dir='.5 .1 -1',
        diffuse='.45 .48 .55', castshadow='false')
    add(world, 'geom', name='floor', type='plane', size='1 1 .01',
        material='floor_mat', contype='2', conaffinity='1')
    add(world, 'geom', name='plinth', type='cylinder', pos='0 0 .012',
        size='.055 .012', rgba='.075 .09 .12 1', contype='0', conaffinity='0')
    add(world, 'geom', name='core_support', type='cylinder', pos='0 0 .051',
        size='.003 .027', rgba='.22 .25 .3 1', contype='0', conaffinity='0')
    core = add(world, 'body', name='core', pos=vec(ORIGIN))
    add(core, 'geom', type='sphere', size='.008', rgba='.04 .05 .07 1',
        contype='0', conaffinity='0')
    actuators = add(root, 'actuator')
    for face, (axis, sign) in FACES.items():
        center = add(core, 'body', name=f'center_{face}')
        unit = np.eye(3)[axis]*sign
        add(center, 'joint', name=f'hinge_{face}', type='hinge', axis=vec(unit),
            damping='.00002', armature='.000001')
        piece_geometry(center, unit, unit*PITCH)
        add(actuators, 'position', name=f'drive_{face}', joint=f'hinge_{face}',
            kp='.3', kv='.003', forcerange='-.15 .15')
    equality = add(root, 'equality')
    slots = [p for p in itertools.product((-1, 0, 1), repeat=3)
             if np.count_nonzero(p) >= 2]
    for index, slot in enumerate(slots):
        name = f'piece_{index:02d}'
        body = add(world, 'body', name=name, pos=vec(ORIGIN+PITCH*np.array(slot)))
        add(body, 'freejoint', name=f'{name}_free')
        piece_geometry(body, slot, np.zeros(3))
        for parent in ['core'] + [f'center_{f}' for f in FACES]:
            add(equality, 'weld', name=f'{name}_to_{parent}', body1=parent,
                body2=name, active='true' if parent == 'core' else 'false',
                torquescale='.03')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode'), np.array(slots, dtype=int)


class RubiksCube:
    def __init__(self, xml=None):
        default_xml, self.initial_slots = build_xml()
        self.xml = default_xml if xml is None else xml
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.slots = self.initial_slots.copy()
        self.orientations = np.repeat(np.eye(3, dtype=int)[None], 20, axis=0)
        self.piece_ids = np.array([self.model.body(f'piece_{i:02d}').id for i in range(20)])
        self.targets = {face: 0. for face in FACES}
        self.active_face = None
        self.progress = 0.
        self.history = []
        mujoco.mj_forward(self.model, self.data)

    def piece_at_initial(self, slot):
        index = np.flatnonzero(np.all(self.initial_slots == slot, axis=1))[0]
        return self.piece_ids[index]

    def _attach(self, index, parent, ideal=False):
        name = f'piece_{index:02d}'
        eq = self.model.equality(f'{name}_to_{parent}').id
        parent_id = self.model.body(parent).id
        piece_id = self.piece_ids[index]
        parent_rotation = self.data.xmat[parent_id].reshape(3, 3)
        if ideal:
            core = self.model.body('core').id
            core_rotation = self.data.xmat[core].reshape(3, 3)
            position = self.data.xpos[core] + core_rotation@(PITCH*self.slots[index])
            orientation = core_rotation@self.orientations[index]
        else:
            position = self.data.xpos[piece_id]
            orientation = self.data.xmat[piece_id].reshape(3, 3)
        relative_position = parent_rotation.T @ (position-self.data.xpos[parent_id])
        relative_rotation = parent_rotation.T @ orientation
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, relative_rotation.ravel())
        # Body weld, anchor at body2 origin: anchor[0:3], relpose[3:10], scale[10].
        self.model.eq_data[eq, :3] = 0
        self.model.eq_data[eq, 3:6] = relative_position
        self.model.eq_data[eq, 6:10] = quat
        start = self.model.equality(f'{name}_to_core').id
        self.data.eq_active[start:start+7] = False
        self.data.eq_active[eq] = True

    def step(self, callback=None):
        mujoco.mj_step(self.model, self.data)
        # Rendering, attachment and acceptance checks must see the integrated pose.
        mujoco.mj_forward(self.model, self.data)
        if callback is not None:
            callback(self)

    def advance(self, seconds, callback=None):
        for _ in range(round(seconds/self.model.opt.timestep)):
            self.step(callback)

    def turn(self, move, callback=None):
        face, count = parse_move(move)
        if self.active_face is not None:
            raise RuntimeError('A face turn is already in progress')
        axis, sign = FACES[face]
        selected = np.flatnonzero(self.slots[:, axis] == sign)
        self.active_face = face
        for index in selected:
            self._attach(index, f'center_{face}')
        actuator = self.model.actuator(f'drive_{face}').id
        start = self.targets[face]
        end = start-count*math.pi/2
        duration = .75 if abs(count) == 1 else 1.05
        steps = round(duration/self.model.opt.timestep)
        for k in range(1, steps+1):
            self.progress = k/steps
            u = self.progress
            smooth = u*u*u*(10+u*(-15+6*u))
            self.data.ctrl[actuator] = start+(end-start)*smooth
            self.step(callback)
        self.targets[face] = end
        self.data.ctrl[actuator] = end
        self.advance(.15, callback)
        hinge = self.model.joint(f'hinge_{face}')
        if abs(self.data.qpos[hinge.qposadr[0]]-end) > .002:
            raise RuntimeError('Turn did not reach alignment; refusing layer reassignment')
        discrete_rotation = np.rint(rotation(axis, -sign*count*math.pi/2)).astype(int)
        self.slots[selected] = self.slots[selected] @ discrete_rotation.T
        self.orientations[selected] = discrete_rotation @ self.orientations[selected]
        for index in selected:
            # A small solver correction at an aligned boundary, never a qpos reset.
            self._attach(index, 'core', ideal=True)
        self.advance(.10, callback)
        self.active_face = None
        self.progress = 0.
        self.history.append(move)

    def is_solved(self):
        return bool(np.array_equal(self.slots, self.initial_slots)
                    and np.all(self.orientations == np.eye(3, dtype=int)))

    def _errors(self, slots, orientations):
        core = self.model.body('core').id
        core_rotation = self.data.xmat[core].reshape(3, 3)
        positions = (self.data.xpos[self.piece_ids]-self.data.xpos[core])@core_rotation
        matrices = core_rotation.T@self.data.xmat[self.piece_ids].reshape(-1, 3, 3)
        position_error = np.linalg.norm(positions-PITCH*slots, axis=1).max()
        traces = np.einsum('nij,nij->n', matrices, orientations)
        angle_error = np.arccos(np.clip((traces-1)/2, -1, 1)).max()
        return float(position_error), float(angle_error)

    def pose_error(self):
        return self._errors(self.slots, self.orientations)

    def solved_pose_error(self):
        return self._errors(self.initial_slots, np.repeat(np.eye(3)[None], 20, axis=0))

    def report(self):
        position, angle = self.solved_pose_error()
        return {'solved': self.is_solved(), 'simulation_time_s': self.data.time,
                'moves': self.history.copy(), 'max_solved_position_error_m': position,
                'max_solved_orientation_error_rad': angle,
                'warnings': self.data.warning.number.tolist(),
                'mujoco_version': mujoco.__version__}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--moves', default=' '.join(SCRAMBLE))
    parser.add_argument('--restore', action='store_true', help='Apply inverse of supplied moves')
    parser.add_argument('--export-xml', type=Path)
    args = parser.parse_args()
    cube = RubiksCube()
    moves = args.moves.split()
    for move in moves:
        parse_move(move)
    if args.export_xml:
        args.export_xml.parent.mkdir(parents=True, exist_ok=True)
        args.export_xml.write_text(cube.xml)
    if args.viewer:
        from mujoco import viewer as mj_viewer
        commands = queue.Queue()

        def key_callback(key):
            if key in map(ord, FACES):
                commands.put(chr(key))
            elif key == ord('S'):
                commands.put('scramble')
            elif key == ord('Z'):
                commands.put('undo')

        print('R/L/U/D/F/B: clockwise face turn; S: scramble; Z: undo all moves.')
        with mj_viewer.launch_passive(cube.model, cube.data, key_callback=key_callback) as viewer:
            viewer.cam.lookat[:] = ORIGIN
            viewer.cam.distance = .24
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            wall_start, sim_start = time.monotonic(), cube.data.time
            last_sync = -1.

            def sync(current):
                nonlocal last_sync
                if current.data.time-last_sync >= 1/60:
                    viewer.sync()
                    last_sync = current.data.time
                    time.sleep(max(0, wall_start+current.data.time-sim_start-time.monotonic()))

            while viewer.is_running():
                if commands.empty():
                    cube.advance(.016, sync)
                    continue
                command = commands.get_nowait()
                sequence = (moves if command == 'scramble' else
                            inverse_moves(cube.history) if command == 'undo' else [command])
                for move in sequence:
                    if not viewer.is_running():
                        break
                    cube.turn(move, sync)
    else:
        for move in moves + (inverse_moves(moves) if args.restore else []):
            cube.turn(move)
    print(json.dumps(cube.report(), indent=2))


if __name__ == '__main__':
    main()
