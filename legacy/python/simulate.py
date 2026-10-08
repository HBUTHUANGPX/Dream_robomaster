"""Run the physical mecanum model: uv run python simulate.py --help."""
from pathlib import Path
import argparse
import json
import math
import time
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / 'assets/robot.xml'
RADIUS = .076
LEVER = .36

def wheel_speeds(vx, vy, wz):
    """Body convention: X forward, Y left, Z up; FL, FR, RL, RR, rad/s."""
    return np.array([vx-vy-LEVER*wz, vx+vy+LEVER*wz,
                     vx+vy-LEVER*wz, vx-vy+LEVER*wz]) / RADIUS

def load():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data

def yaw(data):
    w,x,y,z = data.qpos[3:7]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))

def command(model, data, velocity):
    speeds=wheel_speeds(*velocity)
    # Scale together at actuator limits to preserve the commanded direction.
    data.ctrl[:] = speeds / max(1., np.max(np.abs(speeds))/35.)

def advance(model, data, velocity, seconds):
    for _ in range(round(seconds/model.opt.timestep)):
        command(model,data,velocity)
        mujoco.mj_step(model,data)

def demo_velocity(t):
    return [(0.4,0,0),(0,.4,0),(0,0,.7),(0,0,0)][int(t//3)%4]

def snapshot(model, data, path):
    from PIL import Image
    camera=mujoco.MjvCamera()
    camera.lookat[:]=data.qpos[:3]+[0,0,.08]
    camera.distance=1.25
    camera.azimuth=135
    camera.elevation=-28
    with mujoco.Renderer(model,height=960,width=1280) as renderer:
        renderer.update_scene(data,camera=camera)
        Image.fromarray(renderer.render()).save(path)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--duration',type=float,default=12)
    parser.add_argument('--velocity',type=float,nargs=3,metavar=('VX','VY','WZ'))
    parser.add_argument('--snapshot',type=Path)
    args=parser.parse_args()
    if args.duration<=0 or not math.isfinite(args.duration):
        parser.error('--duration must be finite and positive')
    if args.velocity and not np.isfinite(args.velocity).all():
        parser.error('--velocity must be finite')
    model,data=load()
    advance(model,data,(0,0,0),.5)
    start=data.time
    if args.headless:
        for _ in range(round(args.duration/model.opt.timestep)):
            command(model,data,args.velocity or demo_velocity(data.time-start))
            mujoco.mj_step(model,data)
    else:
        from mujoco import viewer as mj_viewer
        state={'velocity':args.velocity or [0,0,0], 'demo':False}
        def key_callback(key):
            commands={ord('W'):[.4,0,0],ord('S'):[-.4,0,0],ord('A'):[0,.4,0],ord('D'):[0,-.4,0],ord('Q'):[0,0,.7],ord('E'):[0,0,-.7],32:[0,0,0]}
            if key in commands:
                state['velocity']=commands[key];state['demo']=False
            elif key==ord('P'):
                state['demo']=not state['demo']
        print('W/S forward/back; A/D strafe; Q/E rotate; SPACE stop; P demo. Commands latch until replaced.')
        with mj_viewer.launch_passive(model,data,key_callback=key_callback) as viewer:
            viewer.cam.distance=1.4;viewer.cam.elevation=-28;viewer.cam.azimuth=135
            while viewer.is_running():
                tick=time.monotonic()
                velocity=demo_velocity(data.time-start) if state['demo'] else state['velocity']
                for _ in range(10):
                    command(model,data,velocity);mujoco.mj_step(model,data)
                viewer.cam.lookat[:]=data.qpos[:3]+[0,0,.08]
                viewer.sync()
                time.sleep(max(0,.01-(time.monotonic()-tick)))
    if args.snapshot:
        args.snapshot.parent.mkdir(parents=True,exist_ok=True)
        snapshot(model,data,args.snapshot)
    print(json.dumps({'time_s':data.time,'position_m':data.qpos[:3].tolist(),'yaw_rad':yaw(data),'mass_kg':float(model.body_mass.sum()),'warnings':data.warning.number.tolist()},indent=2))

if __name__=='__main__':
    main()
