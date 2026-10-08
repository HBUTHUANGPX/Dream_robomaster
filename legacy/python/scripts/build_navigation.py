"""Build original arena visuals and exact triangle-prism collision surfaces."""
from pathlib import Path
import json,hashlib
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
ROOT=Path(__file__).resolve().parents[1]

def add(p,tag,**kw):return ET.SubElement(p,tag,{k:str(v) for k,v in kw.items()})
def vec(v):return ' '.join(f'{x:.8g}' for x in np.asarray(v).ravel())

def main():
    tree=ET.parse(ROOT/'assets/robot.xml');m=tree.getroot()
    m.find('compiler').set('meshdir','meshes')
    asset=m.find('asset');world=m.find('worldbody')
    world.remove(world.find("geom[@name='ground']"))
    mesh=trimesh.load(ROOT/'assets/arena/rmuc2023.stl')
    add(asset,'mesh',name='arena_visual',file='../arena/rmuc2023.stl')
    add(world,'geom',name='arena_visual',type='mesh',mesh='arena_visual',contype=0,conaffinity=0,group=0,rgba='.36 .43 .48 1')
    # A convex hull of the whole map would fill every ramp/valley. Each triangle
    # is instead extruded 3mm inward; its contact-facing surface stays original.
    for i,(tri,n) in enumerate(zip(mesh.triangles,mesh.face_normals)):
        if np.linalg.norm(np.cross(tri[1]-tri[0],tri[2]-tri[0]))<1e-8:continue
        vertices=np.vstack([tri,tri-.003*n]);center=vertices.mean(0)
        add(asset,'mesh',name=f'contact_{i}',vertex=vec(vertices-center),face='0 1 2 3 5 4 0 3 4 0 4 1 1 4 5 1 5 2 2 5 3 2 3 0')
        add(world,'geom',name=f'terrain_{i}',type='mesh',mesh=f'contact_{i}',pos=vec(center),contype=1,conaffinity=2,group=3,rgba='0 0 0 0',friction='1.3 .002 .0001')
    robot=world.find("body[@name='chassis']");robot.set('pos','7.5 4 .182')
    for geom in robot.iter('geom'):
        geom.set('group','1')
        if geom.get('name','').startswith('roller_geom'):geom.set('friction','1.3 .002 .0001')
    # Fixed dual sensor design: separate height planes, rear/down-looking sensor.
    for name,pos,inverted in [('mid360_top','0 0 .53',False),('mid360_ground','-.11 0 .44',True)]:
        if inverted:
            add(robot,'geom',name=name+'_mast',type='cylinder',fromto='-.19 0 .15 -.19 0 .515',size='.01',mass='.12',rgba='.5 .55 .6 1',contype=0,conaffinity=0,group=1)
            add(robot,'geom',name=name+'_arm',type='box',pos='-.145 0 .507',size='.055 .05 .003',mass='.08',rgba='.5 .55 .6 1',contype=0,conaffinity=0,group=1)
        else:
            add(robot,'geom',name=name+'_mast',type='cylinder',fromto='0 0 .15 0 0 .460',size='.012',mass='.12',rgba='.5 .55 .6 1',contype=0,conaffinity=0,group=1)
            add(robot,'geom',name=name+'_plate',type='box',pos='0 0 .463',size='.05 .05 .003',mass='.08',rgba='.5 .55 .6 1',contype=0,conaffinity=0,group=1)
        body=add(robot,'body',name=name,pos=pos,quat='0 1 0 0' if inverted else '1 0 0 0')
        add(body,'geom',name=name+'_case',type='box',pos='0 0 -.034',size='.0325 .0325 .03',mass='.265',rgba='.16 .19 .22 1',contype=2,conaffinity=1,group=1)
        add(body,'geom',type='cylinder',pos='0 0 -.002',size='.029 .005',mass='.001',rgba='.05 .8 .65 1',contype=0,conaffinity=0,group=1)
        add(body,'site',name=name+'_origin',pos='0 0 0',size='.002',rgba='0 0 0 0',group=1)
    add(robot,'site',name='imu',pos='0 0 .1',size='.003',rgba='0 0 0 0')
    sensors=add(m,'sensor');add(sensors,'gyro',name='gyro',site='imu');add(sensors,'accelerometer',name='accel',site='imu');add(sensors,'framequat',name='attitude',objtype='site',objname='imu')
    for a in m.find('actuator'):
        a.set('kv','4');a.set('forcerange','-10 10')
    m.find('option').set('iterations','40')
    for light in world.findall('light'):world.remove(light)
    add(world,'light',pos='7 10 15',dir='0 0 -1',directional='true',diffuse='.9 .9 .9',ambient='.5 .5 .5',castshadow='false')
    ET.indent(m);tree.write(ROOT/'assets/navigation.xml',encoding='unicode')
    metadata={'repository':'https://github.com/HBUTHUANGPX/Hbut_LC_sentry','commit':'c9a0b28e2f76ff7a8ac07ed4fb5f3ad1c776d068','source':'RMUS_map_ws/src/rmus_map_2/meshes/base_link.stl','sha256':hashlib.sha256((ROOT/'assets/arena/rmuc2023.stl').read_bytes()).hexdigest(),'bounds_m':mesh.bounds.tolist(),'triangles':len(mesh.faces),'collision':'Each original triangle extruded inward 3mm; geometry otherwise unchanged.'}
    (ROOT/'assets/arena/source.json').write_text(json.dumps(metadata,indent=2))
    print(metadata)

if __name__=='__main__':main()
