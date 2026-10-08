"""Rotating-robot EKF inspired by rm_auto_aim (MIT).

State: xc,vx,yc,vy,z,vz,yaw,omega,r1,r2,dz. The nine core states are
augmented by alternate geometry to retain its covariance. yaw is the OUTWARD normal of
plate zero (our PnP convention), so plate = center + r*[cos(yaw),sin(yaw)].
Inputs are measured armor poses only; no simulator or renderer imports.
"""
from dataclasses import dataclass
import math
import numpy as np


def wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi


@dataclass
class ArmorObservation:
    position: np.ndarray
    yaw: float
    number: str
    confidence: float
    covariance: object=None
    yaw_std: float=.20
    detection: object=None


class RotorTracker:
    def __init__(self):
        self.x=None;self.P=np.eye(11);self.time=0.;self.last_seen=-1.
        self.state='LOST';self.number=None;self.count=0;self.switches=0
        self.index=0;self.rejected=0
        self.innovation=0.;self.last_observation=None

    @property
    def r2(self):
        return .255 if self.x is None else float(self.x[9])

    @property
    def dz(self):
        return 0. if self.x is None else float(self.x[10])

    def _initialize(self,obs,time):
        r=.255
        c=obs.position-np.array([r*math.cos(obs.yaw),r*math.sin(obs.yaw),0])
        self.x=np.array([c[0],0,c[1],0,c[2],0,obs.yaw,0,r,r,0.])
        self.P=np.diag([.03,1,.03,1,.01,.2,.15,2,.008,.008,.01])
        self.index=0;self.count=1
        self.time=self.last_seen=time;self.number=obs.number
        self.state='DETECTING';self.last_observation=obs

    def predict(self,time):
        if self.x is None:return
        dt=max(0.,time-self.time)
        F=np.eye(11)
        Q=np.zeros((11,11))
        for pos,vel,noise in ((0,1,4.),(2,3,4.),(4,5,.1),(6,7,.5)):
            F[pos,vel]=dt
            Q[pos,pos]=noise*dt**4/4
            Q[pos,vel]=Q[vel,pos]=noise*dt**3/2
            Q[vel,vel]=noise*dt**2
        Q[8,8]=Q[9,9]=.0001*dt
        Q[10,10]=.00001*dt
        self.x=F @ self.x;self.P=F @ self.P @ F.T+Q
        self.time=time

    def center(self,time):
        if self.x is None:return None
        dt=max(0.,time-self.time)
        return self.x[[0,2,4]]+self.x[[1,3,5]]*dt

    def armor_yaws(self,time):
        return self.x[6]+self.x[7]*max(0.,time-self.time)+np.arange(4)*math.pi/2

    def armor_positions(self,time):
        if self.x is None:return []
        center=self.center(time)
        result=[]
        for i,yaw in enumerate(self.armor_yaws(time)):
            radius=self.x[8] if i%2==0 else self.r2
            result.append(center+np.array([radius*math.cos(yaw),radius*math.sin(yaw),self.dz if i%2 else 0]))
        return result

    def position(self,time):
        return None if self.x is None else self.armor_positions(time)[self.index]

    def association(self,obs,time):
        if self.x is None:return 0.,0
        if obs.number!=self.number:return math.inf,0
        positions=self.armor_positions(time);yaws=self.armor_yaws(time)
        costs=[np.linalg.norm(obs.position-p)**2/.12**2+wrap(obs.yaw-yaw)**2/.45**2
               for p,yaw in zip(positions,yaws)]
        index=int(np.argmin(costs))
        return costs[index],index

    def update(self,observations,time):
        self.last_observation=None
        observations=[o for o in observations if o.number and o.confidence>=.8 and
                      np.isfinite(o.position).all() and math.isfinite(o.yaw)]
        if self.x is None or self.state=='LOST' or time-self.last_seen>.6:
            self.state='LOST';self.count=0
            if observations:
                self._initialize(observations[0],time)
                return observations[0]
            return None
        self.predict(time)
        best=None
        for obs in observations:
            cost,index=self.association(obs,time)
            if best is None or cost<best[0]:best=(cost,index,obs)
        accepted=False
        if best is not None and best[0]<30:
            _,index,obs=best
            yaw=self.x[6]+index*math.pi/2
            radius=self.x[8] if index%2==0 else self.r2
            predicted=np.r_[self.armor_positions(time)[index],yaw]
            measured=np.r_[obs.position,yaw+wrap(obs.yaw-yaw)]
            H=np.zeros((4,11));H[0,0]=H[1,2]=H[2,4]=H[3,6]=1
            H[0,6]=-radius*math.sin(yaw);H[1,6]=radius*math.cos(yaw)
            radial_column=9 if index%2 else 8
            H[0,radial_column]=math.cos(yaw);H[1,radial_column]=math.sin(yaw)
            if index%2:H[2,10]=1
            R=np.zeros((4,4));R[:3,:3]=obs.covariance if obs.covariance is not None else np.eye(3)*.015**2
            R[3,3]=obs.yaw_std**2
            residual=measured-predicted;S=H @ self.P @ H.T+R
            self.innovation=float(residual @ np.linalg.solve(S,residual))
            if self.innovation<40:
                K=np.linalg.solve(S,H @ self.P).T
                self.x+=K @ residual
                A=np.eye(11)-K @ H
                self.P=A @ self.P @ A.T+K @ R @ K.T
                self.P=(self.P+self.P.T)/2
                self.x[8:10]=np.clip(self.x[8:10],.12,.4)
                self.x[10]=np.clip(self.x[10],-.1,.1)
                if index!=self.index:self.switches+=1
                self.index=index;self.last_seen=time;self.count+=1
                self.last_observation=obs;accepted=True
        if accepted:
            self.state='TRACKING' if self.count>=4 else 'DETECTING'
        else:
            if observations:self.rejected+=1
            if time-self.last_seen>.6:
                self.state='LOST';self.count=0
            elif self.count>=4:self.state='TEMP_LOST'
        return self.last_observation

    def ready(self,time):
        return self.x is not None and self.count>=4 and time-self.last_seen<=.18 and self.state!='LOST'
