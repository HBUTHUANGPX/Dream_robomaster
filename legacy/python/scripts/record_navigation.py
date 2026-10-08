"""Record a fresh physical navigation run with map, trajectories and telemetry."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
from pathlib import Path
import sys,subprocess,math,json,argparse
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from navigation.task import NavigationTask
from navigation.terrain import ROOT,RES

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='navigation_omni_demo.mp4');parser.add_argument('--case',type=int,default=2);parser.add_argument('--yaw-rate',type=float,default=0.);parser.add_argument('--playback',type=float,default=1.);args=parser.parse_args()
    if not 0<args.playback<=4:parser.error('playback must be within (0, 4]')
    task=NavigationTask();goal=task.presets()[args.case];task.set_goal(goal['x'],goal['y']);task.set_yaw_rate(args.yaw_rate)
    output=ROOT/'output'/args.output
    proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1280x720','-r',str(10*args.playback),'-i','-','-an','-c:v','libx264','-preset','fast','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
    small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    terrain=task.terrain
    rgb=np.full((*terrain.heights.shape,3),[17,25,35],dtype=np.uint8)
    h=np.nan_to_num(terrain.heights);rgb[terrain.reachable]=np.c_[45+90*h[terrain.reachable],100+70*h[terrain.reachable],95+40*h[terrain.reachable]].clip(0,255).astype('uint8')
    base=Image.fromarray(rgb[::-1]).resize((300,555),Image.Resampling.NEAREST)
    def xy(p):return (int(20+p[0]/terrain.width*300),int(110+(terrain.height-p[1])/terrain.height*555))
    cam=mujoco.MjvCamera();cam.distance=2.4;cam.elevation=-33;cam.azimuth=140
    opt=mujoco.MjvOption();opt.geomgroup[3]=0
    arrived=0;count=0
    try:
        with mujoco.Renderer(task.model,height=600,width=920) as renderer:
            for step in range(800):
                task.step()
                cam.lookat[:]=task.mapper.pose[:3]+[0,0,.1]
                renderer.update_scene(task.data,camera=cam,scene_option=opt)
                for path,color in [(task.path,[.1,.85,1,1]),(task.local,[1,.5,.1,1])]:
                    for a,b in zip(path[:-1],path[1:]):
                        if renderer.scene.ngeom>=renderer.scene.maxgeom:break
                        g=renderer.scene.geoms[renderer.scene.ngeom]
                        mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
                        mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_CAPSULE,.018,a+[0,0,.05],b+[0,0,.05]);renderer.scene.ngeom+=1
                frame=Image.new('RGB',(1280,720),(10,17,27));frame.paste(base,(20,110));frame.paste(Image.fromarray(renderer.render()),(340,85))
                draw=ImageDraw.Draw(frame)
                draw.text((22,18),'RMUC 2023  |  MID-360 NAVIGATION',font=font,fill=(210,239,246))
                draw.text((22,54),f'Omnidirectional DWA + PID  |  LiDAR / encoder localization  |  {args.playback:g}x playback',font=small,fill=(142,183,198))
                for path,color,width in [(task.path,(65,215,245),3),(task.trail,(245,245,245),2),(task.local,(255,174,100),4)]:
                    if len(path)>1:draw.line([xy(p) for p in path],fill=color,width=width)
                for p,color in [(task.mapper.pose,(255,255,255)),([goal['x'],goal['y']],(255,170,90))]:
                    x,y=xy(p);draw.ellipse((x-5,y-5,x+5,y+5),fill=color)
                status='ARRIVED' if task.status=='已到达' else 'NAVIGATING'
                draw.text((355,95),f'{status}  |  target terrain {goal["z"]:.2f} m',font=font,fill=(20,40,45))
                draw.text((350,687),f't={task.data.time:4.1f}s  v={task.state()["speed"]:.2f}m/s  yaw={task.sensors.velocity[2]:+.2f}rad/s  P_est={task.power_w:.0f}/{task.drive.budget:.0f}W',font=small,fill=(215,233,240))
                draw.text((20,683),'Global / Local / Estimated trail',font=small,fill=(173,195,205))
                proc.stdin.write(frame.tobytes());count+=1
                if step%100==0:print(step,task.status,task.mapper.pose.round(2),flush=True)
                if task.goal is None:
                    if task.status!='已到达':raise RuntimeError(task.status)
                    arrived+=1
                    if arrived>=30:break
            if not arrived:raise RuntimeError('Goal not reached')
            error=float(np.linalg.norm(task.data.qpos[:2]-[goal['x'],goal['y']]))
            if error>.2 or task.data.warning.number.any():raise RuntimeError(f'Physical validation failed: error={error}')
            output.with_suffix('.json').write_text(json.dumps({'goal':goal,'truth_error_xy':error,'power_peak_w':task.power_peak_w,'playback':args.playback,'yaw_rate_request':args.yaw_rate},ensure_ascii=False,indent=2))
            frame.save(output.with_suffix('.png'))
    finally:
        proc.stdin.close()
        if proc.wait()!=0:raise RuntimeError('ffmpeg failed')
    print('saved',output,'frames',count,'warnings',task.data.warning.number.tolist())

if __name__=='__main__':main()
