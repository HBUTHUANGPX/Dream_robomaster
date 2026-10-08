"""Known-terrain navigation driven by wheel actuators and LiDAR/encoder estimates."""
from pathlib import Path
import math
import numpy as np
import mujoco
import trimesh
from navigation.terrain import Terrain,ROOT,RES
from navigation.sensors import SensorSuite
from navigation.slam import LidarMapper
from navigation.control import TrajectoryPID,PID,wrap
from navigation.dwa import DWA
from navigation.power import MecanumDrive
from simulate import wheel_speeds

class NavigationTask:
    def __init__(self,localization='prior',power_budget=120.):
        if localization not in ('prior','slam'):raise ValueError('Unknown localization mode')
        self.localization=localization
        self.terrain=Terrain()
        self.model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/navigation.xml'))
        mesh=trimesh.load(ROOT/'assets/arena/rmuc2023.stl')
        visible=mesh.submesh([np.flatnonzero((mesh.face_normals[:,2]>-.1)|(mesh.triangles_center[:,2]>.3))],append=True)
        self.prior,faces=trimesh.sample.sample_surface(visible,160000,seed=36)
        self.prior_normals=visible.face_normals[faces]
        self.sensors=SensorSuite(self.model)
        self.drive=MecanumDrive(float(self.model.body_subtreemass[self.model.body('chassis').id]),power_budget)
        self.planner=DWA(self.terrain,drive=self.drive);self.tracker=TrajectoryPID()
        self.speed_feedback=[PID(1.1,.6,0.,.7),PID(1.1,.6,0.,.7),PID(1.,.3,0.,.6)]
        self.reset()

    def reset(self):
        self.model.actuator_forcerange[:]=[-10.,10.]
        self.model.actuator_biasprm[:,0]=0.
        self.data=mujoco.MjData(self.model)
        for _ in range(500):mujoco.mj_step(self.model,self.data)
        self.mapper=LidarMapper(np.r_[self.terrain.spawn[:2],.176,0.],self.prior if self.localization=='prior' else None,self.prior_normals if self.localization=='prior' else None)
        self.path=np.empty((0,3));self.local=np.empty((0,3));self.trail=[]
        self.goal=None;self.status='请选择绿色可达区域';self.paused=False
        self.scan=np.empty((0,3));self.velocity=np.zeros(3);self.index=0
        self._last_progress=0.;self._stuck_time=0.;self._last_position=self.mapper.pose[:2].copy()
        self.localization_failures=0
        self.slip_seconds=0.
        self.yaw_rate_request=0.;self.power_w=0.;self.power_peak_w=0.;self.energy_j=0.;self.mechanical_w=0.
        self.tracker.reset();self.reference=None;self.measured_velocity=np.zeros(3)
        for pid in self.speed_feedback:pid.reset()
        self.planner.candidates=0;self.planner.admissible=0
        self.sense()

    def sense(self):
        cloud,odom,rotation=self.sensors.sample(self.data,.1)
        previous=self.mapper.pose.copy()
        self.mapper.update(cloud,odom)
        self.rotation=rotation

        c,s=math.cos(self.mapper.pose[3]),math.sin(self.mapper.pose[3])
        R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        # Wheel odometry is a propagation prior, not ground velocity during slip.
        lidar_velocity=((self.mapper.pose[:3]-previous[:3])@R)/.1
        self.measured_velocity[:2]=.5*self.measured_velocity[:2]+.5*lidar_velocity[:2]
        self.measured_velocity[2]=self.sensors.velocity[2]
        slip=np.linalg.norm(self.sensors.velocity[:2]-self.measured_velocity[:2])
        self.slip_seconds=(self.slip_seconds+.1 if slip>.25 and np.linalg.norm(self.velocity[:2])>.05 else 0.)
        self.scan=cloud@R.T+self.mapper.pose[:3]
        self.trail.append(self.mapper.pose[:3].tolist());self.trail=self.trail[-3000:]
        self.localization_failures=self.localization_failures+1 if self.mapper.matched<30 else 0

    def set_goal(self,x,y):
        if not np.isfinite([x,y]).all() or not(0<=x<self.terrain.width and 0<=y<self.terrain.height):
            raise ValueError('目标超出地图')
        newgoal=np.array([x,y,self.terrain.height_at(x,y)])
        newpath=self.terrain.plan(self.mapper.pose,newgoal)
        self.goal=np.array(self.terrain.point(self.terrain.cell(x,y)))
        self.path=newpath;self.index=0;self._stuck_time=self.data.time
        self._last_position=self.mapper.pose[:2].copy();self.status='导航中'
        self.tracker.reset();self.reference=None
        for pid in self.speed_feedback:pid.reset()

    def set_yaw_rate(self,rate):
        if not np.isfinite(rate) or abs(rate)>self.planner.max_yaw_rate:
            raise ValueError('Yaw rate must be within ±1.2 rad/s')
        self.yaw_rate_request=float(rate)

    def stop(self):
        self.tracker.reset();self.reference=None;self.local=np.empty((0,3))
        for pid in self.speed_feedback:pid.reset()
        return np.zeros(3)

    def control(self):
        if self.paused or self.goal is None:return self.stop()
        pose=self.mapper.pose
        if self.localization_failures>=8:
            self.status='定位失效：停车等待有效点云';return self.stop()
        self.status='导航中'
        remaining=np.linalg.norm(self.goal[:2]-pose[:2])
        if remaining<.13 and abs((pose[2]-.076)-self.goal[2])<.12 and np.linalg.norm(self.measured_velocity[:2])<.25:
            self.status='已到达';self.goal=None;return self.stop()
        # Do not skip across a distant parallel route.
        end=min(len(self.path),self.index+20)
        self.index+=int(np.argmin(np.linalg.norm(self.path[self.index:end,:2]-pose[:2],axis=1)))
        look=self.index
        while look<len(self.path)-1 and np.linalg.norm(self.path[look,:2]-pose[:2])<.4:look+=1
        target=self.path[look]
        slope=abs(target[2]-(pose[2]-.076))/max(np.linalg.norm(target[:2]-pose[:2]),.2)
        speed_limit=min(1.2 if slope<.1 else .55,max(.08,remaining*1.5))
        yaw_rate=self.yaw_rate_request
        # On an approaching slope, spread traction across all four wheels by
        # turning gradually toward longitudinal drive (forward OR reverse).
        # Translation remains independent; there is no turn-in-place phase.
        ahead=self.index
        while ahead<len(self.path)-1 and np.linalg.norm(self.path[ahead,:2]-pose[:2])<1.:ahead+=1
        rise=abs(self.path[ahead,2]-(pose[2]-.076))
        if abs(yaw_rate)<1e-6 and (rise>.06 or self.slip_seconds>.5):
            direction=self.path[ahead,:2]-pose[:2]
            error=wrap(math.atan2(direction[1],direction[0])-pose[3])
            error=(error+math.pi/2)%math.pi-math.pi/2
            moving=max(np.linalg.norm(self.measured_velocity[:2]),np.linalg.norm(self.velocity[:2]))
            yaw_rate=float(np.clip(error,-.6,.6))*min(1.,moving/.15)
        self.planner.gravity=9.81*self.rotation[2,:2]
        if np.linalg.norm(pose[:2]-self._last_position)>.12:
            self._last_position=pose[:2].copy();self._stuck_time=self.data.time
        elif self.data.time-self._stuck_time>15:
            self.status='无进展：已停车，请重新选择目标';self.goal=None;return self.stop()
        obstacles=np.empty((0,2))
        if len(self.scan):
            cols=np.clip((self.scan[:,0]/RES).astype(int),0,self.terrain.cols-1)
            rows=np.clip((self.scan[:,1]/RES).astype(int),0,self.terrain.rows-1)
            ground=self.terrain.heights[rows,cols]
            mask=(self.scan[:,2]>ground+.16)&(self.scan[:,2]<pose[2]+.6)
            obstacles=self.scan[mask,:2]
        nominal,trajectory=self.planner.plan(pose,self.measured_velocity,self.path[self.index:],self.goal,obstacles,speed_limit,yaw_rate)
        if not len(trajectory):
            self.status='局部障碍或制动距离不足：停车';return self.stop()
        reference=pose[[0,1,3]] if self.reference is None else self.reference
        proposed=self.tracker.update(pose,reference,nominal,.1)
        command,accepted=self.planner.constrain(pose,self.measured_velocity,proposed,nominal,obstacles,speed_limit)
        if not accepted:self.tracker.reset()
        else:self.tracker.accept(proposed,command)
        # Store the actually executed trajectory, including PID corrections.
        rollout,normal=self.planner.rollout(pose,command[None,:])
        trajectory=rollout[0,1:normal+1]
        self.reference=trajectory[0].copy()
        self.local=np.c_[trajectory[:,:2],[self.terrain.height_at(*p[:2])+.025 for p in trajectory]]
        return command

    def step(self,imu_callback=None):
        if self.paused:
            self.data.ctrl[:]=0;self.velocity=self.stop();return
        if self.data.warning.number.any() or self.rotation[2,2]<.7:
            self.data.ctrl[:]=0;self.velocity[:]=0;self.local=np.empty((0,3))
            self.paused=True;self.status='姿态或物理异常：请重置仿真';return
        self.velocity=self.control()
        motor_velocity=self.velocity.copy()
        if self.goal is not None and np.linalg.norm(self.velocity)>.01:
            for axis,pid in enumerate(self.speed_feedback):
                motor_velocity[axis]+=pid.update(self.velocity[axis]-self.measured_velocity[axis],.1)
        speeds=wheel_speeds(*motor_velocity)
        self.data.ctrl[:]=speeds/max(1.,np.max(abs(speeds))/35.)
        powers=[];mechanical=[]
        load=np.r_[self.drive.mass*9.81*self.rotation[2,:2],0.]@self.drive.force_to_torque.T
        self.model.actuator_biasprm[:,0]=load
        for i in range(100):
            omega=self.data.qvel[self.sensors.wheel_dofs].copy()
            requested=4.*(self.data.ctrl-omega)+load
            torque=self.drive.limit_torque(requested,omega)
            limit=abs(torque)
            self.model.actuator_forcerange[:,0]=-limit
            self.model.actuator_forcerange[:,1]=limit
            mujoco.mj_step(self.model,self.data)
            power=float(self.drive.electrical(self.data.actuator_force,omega))
            powers.append(power);mechanical.append(float(np.sum(self.data.actuator_force*omega)))
            self.energy_j+=power*self.model.opt.timestep
            if imu_callback is not None and (i+1)%5==0:
                mujoco.mj_forward(self.model,self.data)
                imu_callback(self.sensors.imu(self.data))
        self.power_w=float(np.mean(powers));self.mechanical_w=float(np.mean(mechanical))
        self.power_peak_w=max(self.power_peak_w,max(powers))
        self.sense()
        if self.data.warning.number.any() or self.rotation[2,2]<.7:
            self.data.ctrl[:]=0;self.velocity[:]=0;self.local=np.empty((0,3))
            self.paused=True;self.status='姿态或物理异常：请重置仿真'

    def state(self):
        # Ground truth is used ONLY for diagnostics, never control or localization.
        error=float(np.linalg.norm(self.mapper.pose[:3]-self.data.qpos[:3]))
        return {'time':round(self.data.time,2),'status':self.status,'pose':self.mapper.pose.tolist(),'goal':None if self.goal is None else self.goal.tolist(),'global_path':self.path.tolist(),'local_path':self.local.tolist(),'trail':self.trail[::max(1,len(self.trail)//800)],'cloud':self.scan[::max(1,len(self.scan)//1500)].tolist(),'slam_points':len(self.mapper.map_points),'scan_points':len(self.scan),'localization_error':error,'icp_rmse':self.mapper.rmse if math.isfinite(self.mapper.rmse) else None,'speed':float(np.linalg.norm(self.measured_velocity[:2])),'paused':self.paused,'progress':self.index/max(1,len(self.path)-1),'controller':'全向 DWA + PID','dwa_candidates':self.planner.candidates,'dwa_admissible':self.planner.admissible,'tracking_error':self.tracker.error.tolist(),'yaw_rate_request':self.yaw_rate_request,'body_velocity':self.velocity.tolist(),'power_w':self.power_w,'power_peak_w':self.power_peak_w,'power_budget_w':self.drive.budget,'energy_j':self.energy_j,'mechanical_power_w':self.mechanical_w}

    def presets(self):
        candidates=[('平地',7.5,7.),('红方高地',2.,6.),('红方公路高地',6.8,9.5),('中央起伏区',8.8,14.),('蓝方高地',13.,22.),('跨场巡航',7.5,24.),('坡道停驻',9.2,10.3)]
        out=[]
        for name,x,y in candidates:
            ys,xs=np.where(self.terrain.reachable)
            distance=(xs*RES+.05-x)**2+(ys*RES+.05-y)**2
            i=int(np.argmin(distance));p=self.terrain.point((ys[i],xs[i]))
            out.append(dict(name=name,x=p[0],y=p[1],z=p[2]))
        return out
