"""Dynamic Window Approach with measured-velocity bounds and a braking tail.

The circular robot footprint is already inflated in the CAD validity grid.
Scan obstacles get an independent footprint radius. This is a planar velocity
model over a 2.5D terrain, not a contact-dynamics prediction.
"""
import math
import numpy as np
from scipy.spatial import cKDTree
from navigation.power import MecanumDrive


class DWA:
    dt=.1
    acceleration=1.6
    angular_acceleration=2.4
    braking=1.0
    angular_braking=2.4
    radius=.40
    tracking_margin=.035
    max_speed=1.2
    max_yaw_rate=1.2

    def __init__(self,terrain,horizon=1.,drive=None):
        self.terrain=terrain;self.horizon=horizon
        self.drive=drive or MecanumDrive();self.gravity=np.zeros(2)
        self.candidates=0;self.admissible=0;self.best_cost=None

    def window(self,current,speed_limit):
        # Use measured velocity even outside nominal limits. When the reachable
        # interval misses a speed limit, brake as fast as the model permits.
        center=np.asarray(current)
        delta=np.array([self.acceleration,self.acceleration,self.angular_acceleration])*self.dt
        reachable_low=center-delta;reachable_high=center+delta
        limits_low=np.array([-speed_limit,-speed_limit,-self.max_yaw_rate])
        limits_high=np.array([speed_limit,speed_limit,self.max_yaw_rate])
        low=np.maximum(reachable_low,limits_low)
        high=np.minimum(reachable_high,limits_high)
        below=reachable_high<limits_low;above=reachable_low>limits_high
        low=np.where(below,reachable_high,low);high=np.where(below,reachable_high,high)
        low=np.where(above,reachable_low,low);high=np.where(above,reachable_low,high)
        return low,high

    def rollout(self,pose,commands):
        """Batch constant-command trajectories followed by deceleration to rest."""
        commands=np.atleast_2d(commands)
        count=len(commands);state=np.tile(pose[[0,1,3]],(count,1)).astype(float)
        v=commands[:,:2].copy();w=commands[:,2].copy()
        normal=max(1,round(self.horizon/self.dt))
        tail=math.ceil(max(np.max(np.linalg.norm(v,axis=1))/self.braking,np.max(abs(w))/self.angular_braking)/self.dt)+1
        points=[state.copy()]
        for i in range(normal+tail):
            if i>=normal:
                # Use the old speed over this interval: conservative stopping distance.
                next_v=v*np.maximum(0.,1-self.braking*self.dt/np.maximum(np.linalg.norm(v,axis=1),1e-12))[:,None]
                next_w=np.sign(w)*np.maximum(0.,abs(w)-self.angular_braking*self.dt)
            else:next_v=v;next_w=w
            yaw=state[:,2]+w*self.dt/2
            state[:,0]+=(v[:,0]*np.cos(yaw)-v[:,1]*np.sin(yaw))*self.dt
            state[:,1]+=(v[:,0]*np.sin(yaw)+v[:,1]*np.cos(yaw))*self.dt
            state[:,2]+=w*self.dt
            points.append(state.copy());v=next_v;w=next_w
        return np.stack(points,axis=1),normal

    def terrain_check(self,xy):
        t=self.terrain
        in_bounds=(xy[:,:,0]>=0)&(xy[:,:,0]<t.width)&(xy[:,:,1]>=0)&(xy[:,:,1]<t.height)
        cols=np.clip((xy[:,:,0]/.1).astype(int),0,t.cols-1)
        rows=np.clip((xy[:,:,1]/.1).astype(int),0,t.rows-1)
        valid=in_bounds&t.valid[rows,cols]
        valid[:,1:] &= t.valid[rows[:,:-1],cols[:,1:]]&t.valid[rows[:,1:],cols[:,:-1]]
        # Continuous distance to nearby invalid cell squares. Reserve room for
        # localization/tracking error instead of steering exactly along an edge.
        edge=np.minimum.reduce([xy[:,:,0],t.width-xy[:,:,0],xy[:,:,1],t.height-xy[:,:,1]])
        for dr in [-1,0,1]:
            for dc in [-1,0,1]:
                rr=rows+dr;cc=cols+dc
                bad=(rr<0)|(rr>=t.rows)|(cc<0)|(cc>=t.cols)
                bad |= ~t.valid[np.clip(rr,0,t.rows-1),np.clip(cc,0,t.cols-1)]
                dx=np.maximum(np.maximum(cc*.1-xy[:,:,0],xy[:,:,0]-(cc+1)*.1),0.)
                dy=np.maximum(np.maximum(rr*.1-xy[:,:,1],xy[:,:,1]-(rr+1)*.1),0.)
                edge=np.minimum(edge,np.where(bad,np.hypot(dx,dy),np.inf))
        travel=np.c_[np.zeros(len(xy)),np.cumsum(np.linalg.norm(np.diff(xy,axis=1),axis=2),axis=1)]
        # If the estimate is already near an edge, allow turning and escape that
        # increases clearance; never deliberately make that small margin worse.
        required=np.minimum(self.tracking_margin,edge[:,:1]+.05*travel)
        valid &= edge>=required-1e-9
        heights=t.heights[rows,cols]
        # Prevent shortcuts across a steep height transition at a cell boundary.
        dz=abs(np.diff(heights,axis=1))
        dc=np.hypot(np.diff(cols,axis=1),np.diff(rows,axis=1))*.1
        valid[:,1:] &= dz<=dc*math.tan(math.radians(28))+.008
        clearance=t.clearance[rows,cols].min(axis=1)
        return valid,clearance

    def line_clear(self,start,end):
        count=max(2,int(np.linalg.norm(end-start)/.025)+2)
        line=np.linspace(start,end,count)
        return bool(self.terrain_check(line[None,:])[0].all())

    def evaluate(self,pose,commands,obstacles):
        states,normal=self.rollout(pose,commands)
        t=self.terrain
        # Subdivide fast segments, then conservatively check both orthogonal
        # neighbours on every diagonal cell transition (same rule as A*).
        length=np.linalg.norm(np.diff(states[:,:,:2],axis=1),axis=2).max()
        subdivisions=max(1,math.ceil(length/.05))
        fraction=np.arange(subdivisions)/subdivisions
        xy=(states[:,:-1,None,:2]+np.diff(states[:,:,:2],axis=1)[:,:,None,:]*fraction[None,None,:,None]).reshape(len(states),-1,2)
        xy=np.concatenate([xy,states[:,-1:,:2]],axis=1)
        valid,clearance=self.terrain_check(xy)
        if len(obstacles):
            distances=cKDTree(obstacles).query(xy.reshape(-1,2))[0].reshape(xy.shape[:2])
            # Cover the swept segment between samples, not just sample endpoints.
            sweep=np.r_[0.,np.full(xy.shape[1]-1,self.dt/(2*subdivisions))]
            margin=np.linalg.norm(np.atleast_2d(commands)[:,:2],axis=1)[:,None]*sweep
            valid &= distances>self.radius+margin+1e-6
            clearance=np.minimum(clearance,distances.min(axis=1)-self.radius+.38)
        return valid.all(axis=1),states,normal,clearance

    def safe(self,pose,command,obstacles):
        if not np.isfinite(command).all():return False
        return bool(self.evaluate(pose,np.asarray(command)[None,:],obstacles)[0][0])

    def motion_valid(self,commands,current,speed_limit):
        speed=np.linalg.norm(commands[:,:2],axis=1)
        maximum=max(speed_limit,np.linalg.norm(current[:2])-self.acceleration*self.dt)
        return ((speed<=maximum+1e-8)
                &(np.linalg.norm(commands[:,:2]-current[:2],axis=1)<=self.acceleration*self.dt+1e-8)
                &self.drive.feasible(commands,current,self.dt,self.gravity))

    def plan(self,pose,current,path,goal,obstacles,speed_limit=1.2,yaw_rate=0.):
        low,high=self.window(current,speed_limit)
        # A local path carrot avoids attraction through a wall to a distant goal.
        distances=np.linalg.norm(path[:,:2]-pose[:2],axis=1)
        ahead=np.flatnonzero(distances>=max(.65,np.linalg.norm(current[:2])*self.horizon+.4))
        last=int(ahead[0]) if len(ahead) else len(path)-1
        target=path[0,:2]
        # A carrot behind an inflated corner attracts stationary rotations forever.
        # Back up to the furthest directly reachable point on the global path.
        for i in range(last,-1,-1):
            if self.line_clear(pose[:2],path[i,:2]):
                target=path[i,:2];break
        detour=False
        if len(obstacles):
            tree=cKDTree(obstacles)
            direct=np.linspace(pose[:2],target,max(2,int(np.linalg.norm(target-pose[:2])/.025)+2))
            if np.min(tree.query(direct)[0])<=self.radius+.02:
                # Scan-aware local carrot: escape the stop-only minimum in front
                # of an obstacle. Every candidate segment must remain traversable.
                heading=math.atan2(target[1]-pose[1],target[0]-pose[0])
                choices=[]
                for angle in np.linspace(-math.pi/2,math.pi/2,25):
                    for reach in [.65,.4,.25]:
                        point=pose[:2]+reach*np.array([math.cos(heading+angle),math.sin(heading+angle)])
                        line=np.linspace(pose[:2],point,max(2,int(reach/.025)+2))
                        if self.line_clear(pose[:2],point) and np.min(tree.query(line)[0])>self.radius+.02:
                            choices.append((np.linalg.norm(point-target)+.1*abs(angle),point))
                if choices:
                    target=min(choices,key=lambda item:item[0])[1];detour=True
        # Cartesian samples plus goal-aligned and braking samples. Translational
        # direction is independent of yaw; a goal behind the robot needs no U-turn.
        axes=[np.unique(np.r_[np.linspace(low[i],high[i],5 if i<2 else 7),np.clip(0.,low[i],high[i])]) for i in range(3)]
        grid=np.meshgrid(*axes,indexing='ij')
        commands=np.stack([v.ravel() for v in grid],axis=1)
        delta=target-pose[:2];c,s=math.cos(pose[3]),math.sin(pose[3])
        direction=np.array([c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]])
        desired=direction/max(np.linalg.norm(direction),1e-9)*min(speed_limit,np.linalg.norm(direction)/self.horizon)
        extra=[]
        for desired_xy in [desired,np.zeros(2)]:
            change=desired_xy-current[:2]
            xy=current[:2]+change*min(1.,self.acceleration*self.dt/max(np.linalg.norm(change),1e-9))
            for w in np.unique(np.r_[axes[2],np.clip(yaw_rate,low[2],high[2])]):
                extra.append([*xy,w])
        commands=np.unique(np.vstack([commands,np.clip(extra,low,high)]),axis=0)
        self.candidates=len(commands);self.admissible=0;self.best_cost=None
        commands=commands[self.motion_valid(commands,current,speed_limit)]
        if not len(commands):return np.zeros(3),np.empty((0,3))
        good,states,normal,clearance=self.evaluate(pose,commands,obstacles)
        self.admissible=int(good.sum())
        if not good.any():return np.zeros(3),np.empty((0,3))
        endpoints=states[:,normal,:2]
        path_distance=cKDTree(path[:,:2]).query(endpoints)[0]
        delta=target-endpoints
        power=self.drive.predict(commands,current,self.dt,self.gravity)[0]
        # Yaw has its own rate request, never a heading-to-path objective.
        cost=((.25 if detour else 3.)*path_distance+2.*np.linalg.norm(delta,axis=1)
              +.8*abs(commands[:,2]-yaw_rate)+.025/np.maximum(clearance,.05)
              +.03*power/self.drive.budget)
        if np.linalg.norm(goal[:2]-pose[:2])<.4:
            cost=3.*np.linalg.norm(endpoints-goal[:2],axis=1)+.8*abs(commands[:,2]-yaw_rate)+.03*power/self.drive.budget
        cost[~good]=np.inf
        best=int(np.argmin(cost));self.best_cost=float(cost[best])
        return commands[best],states[best,1:normal+1]

    def constrain(self,pose,current,proposed,nominal,obstacles,speed_limit):
        low,high=self.window(current,speed_limit)
        cmd=np.clip(proposed,low,high)
        if self.motion_valid(cmd[None,:],current,speed_limit)[0] and self.safe(pose,cmd,obstacles):return cmd,True
        return nominal.copy(),False
