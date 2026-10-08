"""Compose an attributed Menagerie gripper pair with the movable cube."""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from gripper_plan import AXES
from rubiks_cube import ROOT, ORIGIN, FACES, add, build_xml, vec

ASSETS = ROOT/'assets/robotiq_2f85'
CENTER = np.array([0., 0., .22])
MOUNT_DISTANCE = .2055


def gripper_rotation(normal):
    # Native opening is along local X; local Z points toward the object.
    local_x = np.array([0., 0., 1.])
    local_z = -np.asarray(normal)
    return np.column_stack((local_x, np.cross(local_z, local_x), local_z))


def add_gripper(root, hand):
    native = ET.parse(ASSETS/'2f85.xml').getroot()
    prefix = hand+'_'
    # STL mesh names are implicit upstream. Make them explicit before prefixing.
    for mesh in native.findall('./asset/mesh'):
        mesh.set('name', Path(mesh.get('file')).stem)
        mesh.set('file', str(ASSETS/'assets'/mesh.get('file')))
    references = {'name', 'class', 'childclass', 'body1', 'body2', 'joint',
                  'joint1', 'joint2', 'tendon', 'mesh', 'material'}
    for element in native.iter():
        for key in references & element.attrib.keys():
            element.set(key, prefix+element.get(key))
    collision = native.find(f"./default/default/default[@class='{prefix}collision']/geom")
    collision.set('contype', '2')
    collision.set('conaffinity', '3')
    # Narrow extensions contact one outer layer. Their 4 mm outward offset gives
    # a >89 mm open aperture, clearing the cube's diagonal during in-place yaw.
    for side in ['right', 'left']:
        pad = native.find(f".//body[@name='{prefix}{side}_pad']")
        add(pad, 'geom', name=f'{prefix}{side}_extension', type='box',
            pos='0 .0014 .041', size='.008 .003 .011', mass='.001',
            rgba='.20 .45 .64 1' if hand == 'A' else '.85 .42 .12 1',
            contype='2', conaffinity='3', group='0')
        # Give the rubber tip explicit contact priority: averaging with the
        # cube's softer defaults otherwise accumulates slip over handoffs.
        add(pad, 'geom', name=f'{prefix}{side}_tip', type='box',
            pos='0 .0014 .055', size='.008 .004 .007', mass='.001',
            friction='1.2 .005 .0001', condim='4', priority='2', solref='.002 1',
            solimp='.9999 .9999 .001', rgba='.09 .13 .17 1',
            contype='2', conaffinity='3', group='0')
    for section in ['asset', 'default', 'contact', 'tendon', 'equality', 'actuator']:
        target = root.find(section)
        if target is None:
            target = add(root, section)
        target.extend(deepcopy(list(native.find(section))))
    n = AXES[hand]
    wrist = add(root.find('worldbody'), 'body', name=f'{hand}_wrist', pos=vec(CENTER))
    add(wrist, 'joint', name=f'{hand}_yaw', type='hinge', axis=vec(n),
        range='-3.2 3.2', damping='.015', armature='.0005')
    add(wrist, 'geom', type='cylinder', pos=vec(n*.215), zaxis=vec(n),
        size='.027 .018', mass='.15', contype='2', conaffinity='3',
        rgba='.14 .36 .62 1' if hand == 'A' else '.84 .35 .10 1')
    orientation = np.empty(4)
    mujoco.mju_mat2Quat(orientation, gripper_rotation(n).ravel())
    mount = add(wrist, 'body', name=f'{hand}_mount', pos=vec(n*MOUNT_DISTANCE), quat=vec(orientation))
    mount.extend(deepcopy(list(native.find('worldbody'))))
    add(root.find('actuator'), 'position', name=f'{hand}_yaw_drive',
        joint=f'{hand}_yaw', kp='20', kv='1', forcerange='-15 15')


def build_scene(fast_jaws=False):
    xml, _ = build_xml()
    root = ET.fromstring(xml)
    root.set('model', 'Two perpendicular Robotiq 2F-85 grippers solving a cube')
    root.find('option').set('cone', 'elliptic')
    root.find('option').set('impratio', '10')
    world = root.find('worldbody')
    for name in ['plinth', 'core_support']:
        world.remove(world.find(f"geom[@name='{name}']"))
    for body in world.findall('body'):
        body.set('pos', vec(np.fromstring(body.get('pos'), sep=' ')+CENTER-ORIGIN))
    core = world.find("body[@name='core']")
    core.insert(0, ET.Element('freejoint', name='cube_free'))
    eq = root.find('equality')
    # The demonstration cube's soft welds were tuned for gravity alone. External
    # gripping loads need stronger translation and especially angular retention.
    for weld in eq.findall('weld'):
        weld.set('solref', '.002 1')
        weld.set('solimp', '.9999 .9999 .001')
        weld.set('torquescale', '.3')
    add(eq, 'weld', name='loading_fixture', body1='core',
        solref='.003 1', solimp='.999 .999 .001', torquescale='.06')
    for face in FACES:
        add(eq, 'joint', name=f'lock_{face}', joint1=f'hinge_{face}', active='false',
            polycoef='0 0 0 0 0', solref='.003 1', solimp='.999 .999 .001')
    for hand in ['A', 'B']:
        add_gripper(root, hand)
        n = AXES[hand]
        p = CENTER+n*.26
        add(world, 'geom', name=f'{hand}_pedestal', type='box',
            pos=vec([p[0], p[1], .08]), size='.048 .048 .08',
            rgba='.16 .20 .26 1', contype='0', conaffinity='0')
    if fast_jaws:
        root.find('option').set('timestep', '.0001')
        root.find('option').set('impratio', '100')
        for constraint in eq:
            if constraint.get('solref'):
                constraint.set('solref', '.0003 1')
            if constraint.tag == 'connect':
                constraint.set('solimp', '.9999 .9999 .001')
        for tip in root.findall('.//geom'):
            if tip.get('name', '').endswith('_tip'):
                tip.set('solref', '.0003 1')
        for joint in root.findall('./default//joint'):
            if 'armature' in joint.attrib:
                joint.set('armature', str(float(joint.get('armature'))/100))
        # High-speed parallel-finger variant: prevent the adaptive coupler from
        # folding under inertial load. These constraints stay inside each hand.
        for hand in ['A', 'B']:
            for side in ['right', 'left']:
                add(eq, 'joint', name=f'{hand}_{side}_parallel_guide',
                    joint1=f'{hand}_{side}_coupler_joint', polycoef='0 0 0 0 0',
                    solref='.0003 1', solimp='.9999 .9999 .001')
    ET.indent(root)
    return ET.tostring(root, encoding='unicode')
