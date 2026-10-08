"""MID-360-inspired ray sensor. Simulator truth is confined to measurement synthesis.

Uses official FOV/range and configurable reduced ray count, not Livox's proprietary
scan pattern. Exposes body-relative points, ideal AHRS, gyro and wheel encoders.
"""
import math
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

class SensorSuite:
    def __init__(self,model,rays=1600):
        self.model=model;self.rays=rays;self.frame=0;self.rng=np.random.default_rng(36)
        self.wheel_dofs=[model.joint('wheel_'+name).dofadr[0] for name in ['fl','fr','rl','rr']]
        self.sites=[model.site(name+'_origin').id for name in ['mid360_top','mid360_ground']]
        self.chassis=model.body('chassis').id
        self.groups=np.array([1,1,0,0,0,0],dtype=np.uint8)

    def sample(self,data,dt):
        q=data.sensor('attitude').data.copy()
        rot=Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
        heading=math.atan2(rot[1,0],rot[0,0])
        c,s=math.cos(heading),math.sin(heading)
        unyaw=np.array([[c,s,0],[-s,c,0],[0,0,1]])
        leveling=unyaw@rot
        clouds=[];self.raw_clouds=[]
        for site in self.sites:
            k=np.arange(self.rays)+self.frame*self.rays
            az=(k*2.399963229728653)%(2*math.pi)
            el=np.deg2rad(-7+59*((k*.7548776662466927)%1))
            dirs=np.c_[np.cos(el)*np.cos(az),np.cos(el)*np.sin(az),np.sin(el)]
            sensor_rot=data.site_xmat[site].reshape(3,3)
            world_dirs=np.ascontiguousarray(dirs@sensor_rot.T)
            origin=data.site_xpos[site]+sensor_rot[:,2]*.008
            ids=np.full(self.rays,-1,dtype=np.int32);ranges=np.zeros(self.rays)
            mujoco.mj_multiRay(self.model,data,origin,world_dirs.ravel(),self.groups,True,-1,ids,ranges,None,self.rays,40.)
            good=(ranges>=.2)&(ranges<=40)&(ids>=0)
            # Robot returns are rejected AFTER intersection: actual self-occlusion remains.
            good &= self.model.geom_group[np.maximum(ids,0)]==0
            distances=ranges[good]+self.rng.normal(0,.008,good.sum())
            raw_points=dirs[good]*distances[:,None]
            imu=self.model.site('imu').id
            imu_rot=data.site_xmat[imu].reshape(3,3)
            self.raw_clouds.append(dict(stamp=float(data.time),frame=self.model.site(site).name,
                points=raw_points,rotation=imu_rot.T@sensor_rot,
                translation=(origin-data.site_xpos[imu])@imu_rot))
            world_points=origin+world_dirs[good]*distances[:,None]
            body_points=(world_points-data.xpos[self.chassis])@rot
            clouds.append(body_points@leveling.T)
        self.frame+=1
        points=np.vstack(clouds)
        w=data.qvel[self.wheel_dofs]
        vx=.076*np.sum(w)/4;vy=.076*(-w[0]+w[1]+w[2]-w[3])/4
        xyz=leveling@np.array([vx,vy,0])*dt
        gyro=data.sensor('gyro').data
        # Vertical angular velocity in the gravity-aligned frame.
        dyaw=float((leveling@gyro)[2])*dt
        self.velocity=np.array([vx,vy,dyaw/dt])
        return points,np.r_[xyz,dyaw],rot

    def imu(self,data):
        """Raw IMU at its mounting site. Specific force includes gravity at rest."""
        return dict(stamp=float(data.time),angular_velocity=data.sensor('gyro').data.copy(),
                    acceleration=data.sensor('accel').data.copy())
