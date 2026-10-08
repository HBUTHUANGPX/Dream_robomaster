"""Generate a standalone MJCF with four driven wheels and 48 passive rollers."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
RADIUS = 0.076
HALF_LENGTH = 0.18
HALF_TRACK = 0.18

def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k,v in attrs.items()})

def main():
    m = ET.Element('mujoco', model='RoboMaster simplified infantry')
    add(m,'compiler', angle='radian', meshdir='meshes', autolimits='true')
    add(m,'option', timestep='0.001', integrator='implicitfast', cone='elliptic', iterations='60', gravity='0 0 -9.81')
    default=add(m,'default')
    add(default,'geom', friction='1.0 0.002 0.0001', solref='0.008 1', solimp='0.95 0.99 0.001', condim='4')
    visual=add(m,'visual');add(visual,'global', offwidth='1280', offheight='960')
    asset=add(m,'asset')
    for name in ['armor_am02','armor_frame_a']:
        add(asset,'mesh',name=name,file=f'{name}.obj')
    add(asset,'texture',name='tiles',type='2d',builtin='checker',rgb1='.16 .19 .23',rgb2='.23 .27 .31',width='512',height='512')
    add(asset,'material',name='floor',texture='tiles',texrepeat='12 12',reflectance='.1')
    world=add(m,'worldbody')
    add(world,'light',pos='1 -2 3',dir='-0.3 0.3 -1',diffuse='.9 .9 .9',ambient='.35 .35 .35')
    add(world,'light',pos='-2 1 2',dir='0 0 -1',diffuse='.5 .5 .5')
    add(world,'geom',name='ground',type='plane',size='6 6 .1',material='floor',contype='1',conaffinity='2')
    robot=add(world,'body',name='chassis',pos='0 0 .08')
    add(robot,'freejoint',name='root')
    add(robot,'geom',name='lower_deck',type='box',pos='0 0 .045',size='.205 .135 .018',mass='8',rgba='.12 .15 .2 1',contype='2',conaffinity='1')
    add(robot,'geom',name='upper_deck',type='box',pos='0 0 .135',size='.12 .12 .012',mass='2',rgba='.28 .33 .39 1',contype='2',conaffinity='1')
    add(robot,'geom',name='battery',type='box',pos='-.045 0 .085',size='.075 .06 .023',mass='2',rgba='.08 .09 .12 1',contype='0',conaffinity='0')
    add(robot,'geom',name='front_marker',type='box',pos='.09 0 .151',size='.018 .04 .004',mass='.01',rgba='.95 .5 .08 1',contype='0',conaffinity='0')
    for x in [-.10,.10]:
        for y in [-.10,.10]:
            add(robot,'geom',type='cylinder',pos=f'{x} {y} .09',size='.009 .035',mass='.08',rgba='.45 .49 .54 1',contype='0',conaffinity='0')
    # Four bumper rails below armor, outside every armor module.
    for axis in range(2):
        for sign in [-1,1]:
            pos=[0,0,.022];pos[axis]=sign*.275
            size=[.285,.01,.009] if axis==1 else [.01,.265,.009]
            add(robot,'geom',name=f'bumper_{axis}_{sign}',type='box',pos=' '.join(map(str,pos)),size=' '.join(map(str,size)),mass='.25',rgba='.32 .37 .43 1',contype='2',conaffinity='1')
    # Armor local +X points outward, local +Z up. Face normals are 15 degrees up.
    for name,angle in [('front',0),('left',math.pi/2),('rear',math.pi),('right',-math.pi/2)]:
        assembly=add(robot,'body',name=f'armor_mount_{name}',pos=f'{.245*math.cos(angle)} {.245*math.sin(angle)} .175',euler=f'0 0 {angle}')
        plate=add(assembly,'body',name=f'armor_{name}',euler=f'0 {-math.pi/12} 0')
        add(plate,'inertial',pos='0 0 0',mass='.359',diaginertia='.001 .0005 .0006')
        add(plate,'geom',type='mesh',mesh='armor_am02',rgba='.21 .24 .29 1',contype='0',conaffinity='0',mass='0')
        add(plate,'geom',name=f'armor_collision_{name}',type='box',size='.0095 .0705 .065',rgba='0 0 0 0',contype='2',conaffinity='1',mass='0')
        add(plate,'site',name=f'armor_face_{name}',pos='.0095 0 0',size='.001',rgba='0 0 0 0')
        for side in [-1,1]:
            add(plate,'geom',name=f'light_{name}_{side}',type='box',pos=f'.0097 {side*.062} 0',size='.0005 .003 .026',rgba='.05 .4 1 1',contype='0',conaffinity='0',mass='0')
            # Two original A-frame meshes per plate; 95 mm between mounting lines.
            add(assembly,'geom',type='mesh',mesh='armor_frame_a',pos=f'-.009176295 {side*.0475} -.002458781',rgba='.48 .51 .55 1',contype='0',conaffinity='0',mass='.06')
        add(assembly,'geom',type='box',pos='-.09 0 -.045',size='.07 .045 .007',mass='.15',rgba='.28 .32 .38 1',contype='0',conaffinity='0')
        # Bracket platform is below the module; screw/hole centers are explicitly exposed.
        add(assembly,'geom',type='box',pos='-.034 0 -.034868407',size='.026 .062 .003',mass='.15',rgba='.3 .34 .4 1',contype='0',conaffinity='0')
        for x in [-.044894086,-.019894086]:
            for y in [-.0475,.0475]:
                add(assembly,'site',name=f'm4_{name}_{x}_{y}',pos=f'{x} {y} -.031868407',size='.0021',rgba='.8 .8 .8 1')
    actuators=add(m,'actuator')
    for name,x,y,hand in [('fl',.18,.18,-1),('fr',.18,-.18,1),('rl',-.18,.18,1),('rr',-.18,-.18,-1)]:
        add(robot,'geom',type='cylinder',fromto=f'{x} {y*.60} 0 {x} {y} 0',size='.012',mass='.12',rgba='.38 .42 .47 1',contype='0',conaffinity='0')
        add(robot,'geom',type='box',pos=f'{x} {y*.60} .025',size='.025 .016 .03',mass='.1',rgba='.2 .23 .27 1',contype='0',conaffinity='0')
        wheel=add(robot,'body',name=f'wheel_{name}',pos=f'{x} {y} 0')
        add(wheel,'joint',name=f'wheel_{name}',type='hinge',axis='0 1 0',damping='.02',armature='.003')
        add(wheel,'geom',type='cylinder',size='.049 .022',quat='.70710678 .70710678 0 0',mass='.45',rgba='.45 .49 .55 1',contype='0',conaffinity='0')
        for j in range(12):
            theta=2*math.pi*j/12
            roller=add(wheel,'body',name=f'roller_{name}_{j}',pos=f'{.062*math.sin(theta)} 0 {-.062*math.cos(theta)}',zaxis=f'{hand*math.cos(theta)} 1 {hand*math.sin(theta)}')
            add(roller,'joint',name=f'roller_{name}_{j}',type='hinge',axis='0 0 1',damping='.00002',armature='.000001')
            add(roller,'geom',name=f'roller_geom_{name}_{j}',type='ellipsoid',size='.014 .014 .038',mass='.025',rgba='.055 .06 .07 1',contype='2',conaffinity='1')
        add(actuators,'velocity',name=f'drive_{name}',joint=f'wheel_{name}',kv='2',ctrlrange='-35 35',forcerange='-5 5')
    ET.indent(m)
    ET.ElementTree(m).write(ROOT/'assets/robot.xml',encoding='unicode')

if __name__=='__main__':
    main()
