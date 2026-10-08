"""Compose two existing physical robots; no changes to the original asset."""
from copy import deepcopy
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import mujoco

ROOT=Path(__file__).resolve().parents[1]
TEAMS=('blue','red')


def add(parent,tag,**attrs):
    return ET.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})


def build_model():
    tree=ET.parse(ROOT/'assets/robot.xml').getroot()
    tree.set('model','RoboMaster RGB vision duel')
    tree.find('compiler').set('meshdir',str(ROOT/'assets/meshes'))
    tree.find('visual/global').set('offwidth','1000')
    tree.find('visual/global').set('offheight','750')
    asset=tree.find('asset')
    add(asset,'texture',name='number_3',type='2d',file=str(ROOT/'assets/armor_labels/3.png'))
    add(asset,'material',name='number_3',texture='number_3',texrepeat='1 1',texuniform='false',
        rgba='1 1 1 1',emission='.3',specular='0')
    vertices=[(x,y,z) for x in (-.0001,.0001) for y,z in
              ((-.055,-.055),(.055,-.055),(.055,.055),(-.055,.055))]
    add(asset,'mesh',name='number_plate',vertex=' '.join(str(v) for p in vertices for v in p),
        texcoord='0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0',
        face='4 5 6 4 6 7 0 2 1 0 3 2 0 1 5 0 5 4 1 2 6 1 6 5 2 3 7 2 7 6 3 0 4 3 4 7')
    add(asset,'texture',type='skybox',builtin='gradient',rgb1='.13 .19 .29',rgb2='.025 .035 .06',width='512',height='3072')
    asset.find("material[@name='floor']").set('reflectance','0')
    for name,color in [('blue','.03 .12 1 1'),('red','1 .025 .025 1')]:
        add(asset,'material',name=f'{name}_lamp',rgba=color,emission='1',specular='0')
    world=tree.find('worldbody')
    # Fine CAD triangles and millimetre trails produce shadow-map acne at this
    # arena scale. Keep diffuse lighting deterministic for the CV baseline.
    for light in world.findall('light'):
        light.set('castshadow','false')
    template=world.find("body[@name='chassis']")
    world.remove(template)
    drive_template=list(tree.find('actuator'))
    tree.remove(tree.find('actuator'))
    actuators=add(tree,'actuator')
    world.find("geom[@name='ground']").set('size','6 4 .1')
    world.find("geom[@name='ground']").set('conaffinity','6')
    for axis in (0,1):
        for sign in (-1,1):
            pos=[0,0,.3];pos[axis]=sign*(6 if axis==0 else 4)
            size=[.06,4,.3] if axis==0 else [6,.06,.3]
            add(world,'geom',name=f'wall_{axis}_{sign}',type='box',pos=' '.join(map(str,pos)),
                size=' '.join(map(str,size)),rgba='.25 .31 .4 1',contype='1',conaffinity='6')
    for team,spawn,heading in [('blue',-2.,0),('red',2.,math.pi)]:
        robot=deepcopy(template)
        for element in robot.iter():
            for attr in ('name','joint'):
                if attr in element.attrib:
                    element.set(attr,f'{team}_{element.get(attr)}')
            if element.tag=='geom' and 'light_' in element.get('name',''):
                element.attrib.pop('rgba',None)
                element.set('material',f'{team}_lamp')
            if 'armor_collision_' in element.get('name',''):
                element.set('group','3')
        robot.set('pos',f'{spawn} 0 .08')
        for side in ('front','left','rear','right'):
            armor=robot.find(f".//body[@name='{team}_armor_{side}']")
            # Physical, camera-visible label; its ID is never passed to vision.
            # Mesh UV right/up map to armor +Y/+Z; outward normal is +X.
            add(armor,'geom',name=f'{team}_number_{side}',type='mesh',mesh='number_plate',pos='.0103 0 0',
                material='number_3',
                contype='0',conaffinity='0',mass='0')
        robot.set('euler',f'0 0 {heading}')
        world.append(robot)
        yaw=add(robot,'body',name=f'{team}_yaw',pos='0 0 .29')
        add(yaw,'joint',name=f'{team}_yaw_joint',axis='0 0 1',damping='.8',armature='.025')
        add(yaw,'geom',type='cylinder',size='.065 .035',mass='.4',rgba='.22 .27 .34 1')
        pitch=add(yaw,'body',name=f'{team}_pitch',pos='0 0 .045')
        # Negative Y axis makes positive pitch point upward.
        add(pitch,'joint',name=f'{team}_pitch_joint',axis='0 -1 0',range='-.45 .5',damping='.4',armature='.012')
        add(pitch,'geom',type='box',pos='.025 0 0',size='.075 .043 .028',mass='.35',rgba='.13 .18 .23 1')
        # Official drawing 3-31: OD 21 mm; 97 mm mounting segment; 1 mm wall.
        for j in range(16):
            angle=2*math.pi*j/16
            add(pitch,'geom',type='box',pos=f'.1485 {.010*math.cos(angle)} {.010*math.sin(angle)}',
                euler=f'{angle} 0 0',size='.0485 .0005 .002',mass='.002',rgba='.47 .51 .58 1')
        add(pitch,'site',name=f'{team}_muzzle',pos='.197 0 0',size='.003',rgba='0 0 0 0')
        add(pitch,'geom',type='box',pos='.04 0 .046',size='.022 .025 .017',mass='.05',rgba='.12 .15 .18 1')
        add(pitch,'camera',name=f'{team}_camera',pos='.065 0 .046',xyaxes='0 -1 0 0 0 1',fovy='45')
        # Preserve the original exclusion of intra-robot roller contacts while
        # allowing ground and opponent collisions. Each robot owns one bit.
        for geom in robot.iter('geom'):
            if geom.get('contype','1') != '0':
                geom.set('contype','2' if team=='blue' else '4')
                geom.set('conaffinity','5' if team=='blue' else '3')
        for actuator in drive_template:
            item=deepcopy(actuator)
            item.set('name',f'{team}_{item.get("name")}')
            item.set('joint',f'{team}_{item.get("joint")}')
            item.set('ctrlrange','-85 85')
            actuators.append(item)
        for axis,limit in [('yaw','-6.2832 6.2832'),('pitch','-.45 .5')]:
            add(actuators,'position',name=f'{team}_{axis}_servo',joint=f'{team}_{axis}_joint',
                kp='65',kv='5',ctrlrange=limit,forcerange='-8 8')
    return mujoco.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'))
