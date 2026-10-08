"""Serialized 1v1 simulation, image-driven controllers, projectile referee."""
from dataclasses import dataclass,field
import math
import time as clock
import mujoco
import numpy as np
import cv2
from simulate import wheel_speeds
from duel.model import build_model,TEAMS
from duel.physics import Heat,Projectile,segment_box,segment_ellipsoid,segment_cylinder
from duel.rotor_tracking import RotorTracker,ArmorObservation
from duel.vision import Detector

FRAME_DT=.05
CONTROL_DT=.01
RADIUS=.0085


def wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi


@dataclass
class Robot:
    detector: Detector
    tracker: RotorTracker=field(default_factory=RotorTracker)
    heat: Heat=field(default_factory=Heat)
    hp: int=200
    shots: int=0
    hits: int=0
    last_shot: float=-10.
    detection: object=None
    aim: object=None
    flight: float=0.
    aim_error: float=180.
    aim_plate: int=0
    angles: np.ndarray=field(default_factory=lambda:np.array([0.,-.05]))
    last_damage: dict=field(default_factory=dict)
    reason: str='等待图像'
    fire_reason: str='自动开火关闭'
    fire_wait: float=0.
    aim_facing: float=0.
    aim_velocity: object=None
    last_aim: object=None
    last_aim_time: float=0.
    search_origin: np.ndarray=field(default_factory=lambda:np.array([0.,-.05]))
    search_time: float=0.


class Duel:
    def __init__(self):
        self.revision=0
        self.model=build_model()
        self.data=mujoco.MjData(self.model)
        self.settings={'auto_aim':True,'auto_fire':False,'enemy_fire':False,'vision':True,
                       'protect_heat':True,'enemy_motion':'static','speed':23.,'fire_rate':5.,
                       'target_speed':2.,'target_spin':2.,'invulnerable':True,'profile':'cooling','view':'overview'}
        self.drive=np.zeros(3)
        self.drive_deadline=0.
        self.raw=[None,None];self.annotated=[None,None]
        self.chassis=[self.model.body(f'{t}_chassis').id for t in TEAMS]
        self.cameras=[self.model.camera(f'{t}_camera').id for t in TEAMS]
        self.muzzles=[self.model.site(f'{t}_muzzle').id for t in TEAMS]
        self.drives=[[self.model.actuator(f'{t}_drive_{n}').id for n in ('fl','fr','rl','rr')] for t in TEAMS]
        self.servos=[[self.model.actuator(f'{t}_{a}_servo').id for a in ('yaw','pitch')] for t in TEAMS]
        self.joints=[[self.model.joint(f'{t}_{a}_joint').qposadr[0] for a in ('yaw','pitch')] for t in TEAMS]
        self.colliders=[[],[]]
        for gid in range(self.model.ngeom):
            name=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,gid) or ''
            kind=int(self.model.geom_type[gid])
            if kind not in (mujoco.mjtGeom.mjGEOM_BOX,mujoco.mjtGeom.mjGEOM_ELLIPSOID,mujoco.mjtGeom.mjGEOM_CYLINDER):
                continue
            if 'light_' in name:continue
            root=self.model.body_rootid[self.model.geom_bodyid[gid]]
            if root in self.chassis:
                owner=self.chassis.index(root)
                self.colliders[owner].append((gid,kind,'armor_collision' in name))
        self.reset()

    @property
    def time(self):
        return float(self.data.time-self.start)

    def reset(self):
        mujoco.mj_resetData(self.model,self.data)
        for i in range(2):
            self.data.qpos[self.joints[i][1]]=-.05
        mujoco.mj_forward(self.model,self.data)
        for _ in range(300):
            for i in range(2):
                self.data.ctrl[self.servos[i]]=[0,-.05]
            mujoco.mj_step(self.model,self.data)
        self.start=self.data.time
        self.robots=[Robot(Detector(enemy='red')),Robot(Detector(enemy='blue'))]
        for r in self.robots:
            r.heat=Heat(self.settings['profile'])
        self.projectiles=[];self.trails=[];self.events=[]
        self.paused=False;self.winner=None;self.drive[:]=0;self.drive_deadline=0
        self.raw=[None,None];self.annotated=[None,None]
        self.event('新回合：蓝方对红方 · 200 HP')

    def event(self,text):
        self.events.append({'time':round(self.time,2),'text':text})
        self.events=self.events[-30:]

    def camera_pose(self,i):
        # Only this robot's pose/encoders: ideal odometry + calibrated extrinsics.
        cam=self.cameras[i]
        return self.data.cam_xpos[cam].copy(),self.data.cam_xmat[cam].reshape(3,3) @ np.diag([1.,-1.,-1.])

    def muzzle_velocity(self,i):
        velocity=np.zeros(6)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_SITE,self.muzzles[i],velocity,0)
        return velocity[3:]

    def perceive(self,renderer,i):
        renderer.update_scene(self.data,camera=self.cameras[i])
        self.draw_projectiles(renderer)
        rgb=renderer.render().copy()
        self.raw[i]=rgb
        r=self.robots[i]
        candidates=r.detector.detect(rgb) if self.settings['vision'] and not r.heat.locked else []
        candidates=[d for d in candidates if d.number is not None]
        cam,R=self.camera_pose(i)
        observations=[]
        for d in candidates:
            alternatives=[]
            for rvec,tvec,error,projected in d.pose_candidates:
                normal=R @ (-cv2.Rodrigues(rvec)[0][:,2])
                position=cam+R @ tvec
                depth=tvec[2]
                covariance=R @ np.diag([max(.003,depth*.0015)**2]*2+[max(.015,.008*depth**2)**2]) @ R.T
                obs=ArmorObservation(position,math.atan2(normal[1],normal[0]),d.number,d.confidence,
                                     covariance,.25,d)
                # Small armor mounting tilt is known calibration, not target state.
                # IPPE's mirrored solution often has lower pixel error at 4 m;
                # use the upward 15-degree normal to disambiguate it.
                cost=error+((normal[2]-math.sin(math.radians(15)))/.2)**2
                if r.tracker.x is not None and r.tracker.state!='LOST':
                    cost+=r.tracker.association(obs,self.time)[0]*.1
                alternatives.append((cost,obs,rvec,tvec,error,projected))
            _,obs,d.rvec,d.tvec,d.error,d.projected=min(alternatives,key=lambda item:item[0])
            obs.yaw=r.detector.refine_yaw(d,obs.position,cam,R)
            observations.append(obs)
        r.detection=None
        accepted=r.tracker.update(observations,self.time)
        if accepted is not None:r.detection=accepted.detection
        self.plan_aim(i)
        prediction=None if r.aim is None else R.T @ (r.aim-cam)
        self.annotated[i]=r.detector.annotate(rgb,r.detection,prediction)
        if r.tracker.ready(self.time):
            frame=self.annotated[i]
            for index,point in enumerate(r.tracker.armor_positions(self.time+r.flight)):
                p=R.T @ (point-cam)
                if p[2]<=.1:continue
                xy=r.detector.K @ p;xy=xy[:2]/xy[2]
                if 0<=xy[0]<frame.shape[1] and 0<=xy[1]<frame.shape[0]:
                    color=(255,190,70) if index==r.aim_plate else (120,170,255)
                    cv2.drawMarker(frame,tuple(np.round(xy).astype(int)),color,cv2.MARKER_SQUARE,9,1)
            center=R.T @ (r.tracker.center(self.time)-cam)
            if center[2]>.1:
                xy=r.detector.K @ center;xy=xy[:2]/xy[2]
                if 0<=xy[0]<frame.shape[1] and 0<=xy[1]<frame.shape[0]:
                    cv2.drawMarker(frame,tuple(np.round(xy).astype(int)),(100,220,255),cv2.MARKER_CROSS,16,1)

    def plan_aim(self,i):
        r=self.robots[i]
        if r.tracker.ready(self.time):
            muzzle=self.data.site_xpos[self.muzzles[i]]
            r.aim=None
            choices=[]
            velocity=self.muzzle_velocity(i)
            for index in range(4):
                flight=np.linalg.norm(r.tracker.armor_positions(self.time)[index]-muzzle)/self.settings['speed']
                for _ in range(8):
                    target=r.tracker.armor_positions(self.time+flight)[index]
                    displacement=target-muzzle-velocity*flight+np.array([0,0,4.905])*flight**2
                    flight=np.linalg.norm(displacement)/self.settings['speed']
                yaw=r.tracker.armor_yaws(self.time+flight)[index]
                normal=np.array([math.cos(yaw),math.sin(yaw),0.])
                facing=float(normal @ (muzzle-target)/np.linalg.norm(muzzle-target))
                if facing>.35 and flight<2:
                    choices.append((facing+(.08 if index==r.aim_plate else 0),index,displacement,flight,facing))
            if choices:
                _,r.aim_plate,displacement,r.flight,r.aim_facing=max(choices,key=lambda item:item[0])
                r.aim=muzzle+displacement
        else:
            r.aim=None

    def draw_projectiles(self,renderer):
        for bullet in self.projectiles:
            if renderer.scene.ngeom>=renderer.scene.maxgeom:break
            geom=renderer.scene.geoms[renderer.scene.ngeom]
            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_SPHERE,np.full(3,RADIUS),
                               bullet.position,np.eye(3).ravel(),np.array([1.,.8,.15,1.],np.float32))
            renderer.scene.ngeom+=1
        for a,b,t,owner in self.trails[::5]:
            if renderer.scene.ngeom>=renderer.scene.maxgeom:break
            geom=renderer.scene.geoms[renderer.scene.ngeom]
            color=[.3,.7,1.,.65] if owner==0 else [1.,.5,.2,.65]
            mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),
                               np.eye(3).ravel(),np.array(color,np.float32))
            mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_CAPSULE,.003,a,b)
            renderer.scene.ngeom+=1

    def control(self,i,dt=CONTROL_DT):
        r=self.robots[i]
        auto=self.settings['auto_aim'] if i==0 else True
        if auto and r.aim is not None:
            direction=r.aim-self.data.site_xpos[self.muzzles[i]]
            local=self.data.xmat[self.chassis[i]].reshape(3,3).T @ direction
            desired=np.array([math.atan2(local[1],local[0]),math.atan2(local[2],np.linalg.norm(local[:2]))])
            # Velocity feedforward compensates the position servo's kv/kp lag.
            # Reset across plate switches/large jumps; never feed a discontinuity.
            if r.last_aim is not None and self.time>r.last_aim_time:
                delta=desired-r.last_aim;delta[0]=wrap(delta[0])
                rate=delta/(self.time-r.last_aim_time)
                r.aim_velocity=np.clip(rate,-2.,2.) if np.linalg.norm(delta)<.08 else np.zeros(2)
            else:r.aim_velocity=np.zeros(2)
            r.last_aim=desired.copy();r.last_aim_time=self.time
            desired+=r.aim_velocity*(5/65)
            current=self.data.qpos[self.joints[i]]
            desired[0]=current[0]+wrap(desired[0]-current[0])
            desired=np.clip(desired,[-6.28,-.45],[6.28,.5])
            r.angles += np.clip(desired-r.angles,-3.6*dt,3.6*dt)
            r.search_origin=r.angles.copy();r.search_time=self.time
        elif auto and self.time-r.tracker.last_seen > .6:
            r.last_aim=None
            # Search using only own camera state, never opponent coordinates.
            elapsed=max(0.,self.time-r.search_time-.6)
            amplitude=min(1.8,.2+.15*max(0,elapsed-3))
            r.angles[0]=np.clip(r.search_origin[0]+amplitude*math.sin(elapsed*2),-6.28,6.28)
            r.angles[1]=r.search_origin[1]
        self.data.ctrl[self.servos[i]]=r.angles
        if r.aim is not None:
            axis=self.data.site_xmat[self.muzzles[i]].reshape(3,3)[:,0]
            d=r.aim-self.data.site_xpos[self.muzzles[i]];d/=np.linalg.norm(d)
            r.aim_error=float(np.rad2deg(np.arccos(np.clip(axis @ d,-1,1))))
        else:
            r.aim_error=180.
        r.reason=('热量锁枪' if r.heat.locked else '跟踪中' if r.tracker.ready(self.time) else '搜索装甲')
        wants_fire=self.settings['auto_fire'] if i==0 else self.settings['enemy_fire']
        reason,wait=self.fire_gate(i,automatic=True)
        r.fire_reason=reason;r.fire_wait=wait
        if wants_fire and auto and not reason:
            self.shoot(i)
            r.fire_reason='已发射'
        elif not wants_fire:r.fire_reason='自动开火关闭'
        elif not auto:r.fire_reason='手动瞄准：使用单发'

    def fire_gate(self,i,automatic=False):
        r=self.robots[i]
        if self.paused:return '已暂停',0.
        if self.winner is not None or r.hp<=0:return '回合结束',0.
        if r.heat.locked:return '超热锁枪',r.heat.value/r.heat.rate
        if self.settings['protect_heat'] and r.heat.value+10>r.heat.limit+1e-8:
            return '热量冷却',(r.heat.value+10-r.heat.limit)/r.heat.rate
        remaining=1/self.settings['fire_rate']-(self.time-r.last_shot)
        if remaining>1e-8:return '发射间隔',remaining
        if automatic:
            if not r.tracker.ready(self.time):return '等待有效视觉跟踪',0.
            if r.aim is None:return '等待装甲转入正面',0.
            muzzle=self.data.site_xpos[self.muzzles[i]]
            local=self.data.site_xmat[self.muzzles[i]].reshape(3,3).T @ (r.aim-muzzle)
            # Window is measured in metres at the predicted plate, not one
            # fixed angle at every distance. Keep a projectile-radius margin.
            if local[0]<=0 or abs(local[1])>max(.018,.062*r.aim_facing-RADIUS) or abs(local[2])>.052-RADIUS:
                return '云台进入射击窗口中',0.
        return '',0.

    def shoot(self,i):
        r=self.robots[i]
        if self.paused or self.winner is not None or r.hp<=0:
            return False
        if self.time-r.last_shot < 1/self.settings['fire_rate']-1e-8:
            return False
        if not r.heat.fire(self.time,self.settings['protect_heat']):
            r.reason='等待冷却' if not r.heat.locked else '热量锁枪'
            return False
        axis=self.data.site_xmat[self.muzzles[i]].reshape(3,3)[:,0]
        p=self.data.site_xpos[self.muzzles[i]].copy()
        v=axis*self.settings['speed']+self.muzzle_velocity(i)
        self.projectiles.append(Projectile(i,p,v,self.time))
        r.shots+=1;r.last_shot=self.time
        if r.heat.locked:
            self.event(f'{TEAMS[i]} 超热：锁枪并中断第一视角，冷却至零恢复')
        return True

    def advance_projectiles(self,dt,previous_positions,previous_rotations):
        alive=[]
        for bullet in self.projectiles:
            old=bullet.position.copy()
            new=old+bullet.velocity*dt+np.array([0,0,-4.905])*dt*dt
            bullet.velocity[2]-=9.81*dt
            first=None
            owner=1-bullet.owner
            nearby=np.linalg.norm(new-self.data.xpos[self.chassis[owner]])<.8
            for gid,kind,armor in self.colliders[owner] if nearby else []:
                R=self.data.geom_xmat[gid].reshape(3,3)
                a=previous_rotations[gid].reshape(3,3).T @ (old-previous_positions[gid])
                b=R.T @ (new-self.data.geom_xpos[gid])
                half=self.model.geom_size[gid]+RADIUS
                if kind==mujoco.mjtGeom.mjGEOM_BOX:
                    fraction=segment_box(a,b,half)
                elif kind==mujoco.mjtGeom.mjGEOM_ELLIPSOID:
                    fraction=segment_ellipsoid(a,b,half)
                else:
                    fraction=segment_cylinder(a,b,half[0],half[1])
                if fraction is not None and (first is None or fraction<first[0]):
                    first=(fraction,gid,owner,armor,a,b,R)
            if first is not None:
                fraction,gid,owner,armor,a,b,R=first
                at=a+(b-a)*fraction
                # Front face only, with actual contact-normal relative speed.
                normal_speed=-(b[0]-a[0])/dt
                valid=armor and abs(at[0]-(self.model.geom_size[gid,0]+RADIUS)) < 1e-5 and normal_speed>12
                target=self.robots[owner]
                if valid and self.time-target.last_damage.get(gid,-10) >= .05-1e-8:
                    target.last_damage[gid]=self.time
                    if not self.settings['invulnerable']:target.hp=max(0,target.hp-20)
                    self.robots[bullet.owner].hits+=1
                    self.event(f'{TEAMS[bullet.owner]} 命中 {TEAMS[owner]} 装甲 · '+('无敌计分' if self.settings['invulnerable'] else '−20 HP'))
                    if target.hp==0:
                        self.winner=TEAMS[bullet.owner]
                        self.event(f'{self.winner} 获胜 · 重置开始下一回合')
                self.trails.append((old,new,self.time,bullet.owner))
                continue
            if new[2]<RADIUS or abs(new[0])>5.94 or abs(new[1])>3.94 or self.time-bullet.born>3:
                continue
            bullet.position=new
            alive.append(bullet)
            self.trails.append((old,new,self.time,bullet.owner))
        self.projectiles=alive
        self.trails=[trail for trail in self.trails if self.time-trail[2]<.10][-300:]

    def step(self,renderer):
        if self.paused or self.winner is not None:
            return
        for i in range(2):
            self.perceive(renderer,i)
        t=self.time
        v=self.settings['target_speed'];w=self.settings['target_spin'];motion=self.settings['enemy_motion']
        enemy={'static':[0,0,0],'strafe':[0,v*math.cos(t*.8),0],
               'circle':[v,0,w],'spin':[0,0,w]}[motion]
        own=self.drive if clock.monotonic()<self.drive_deadline else np.zeros(3)
        for i,velocity in enumerate((own,enemy)):
            # Boundary brake uses this robot's own position.
            pos=self.data.xpos[self.chassis[i]]
            velocity=np.array(velocity,float)
            if abs(pos[0])>5.3 or abs(pos[1])>3.3:
                velocity[:2]=self.data.xmat[self.chassis[i]].reshape(3,3)[:2,:2].T @ (-pos[:2]*.25)
            speed=wheel_speeds(*velocity)
            self.data.ctrl[self.drives[i]]=speed/max(1,np.max(np.abs(speed))/85)
        for tick in range(round(FRAME_DT/self.model.opt.timestep)):
            if tick%round(CONTROL_DT/self.model.opt.timestep)==0:
                for i in range(2):
                    self.plan_aim(i)
                    self.control(i)
            old_p=self.data.geom_xpos.copy();old_R=self.data.geom_xmat.copy()
            mujoco.mj_step(self.model,self.data)
            self.advance_projectiles(self.model.opt.timestep,old_p,old_R)
            for r in self.robots:
                r.heat.cool(self.time)
            if self.winner is not None:
                self.data.ctrl[:]=0
                break

    def command(self,name,args):
        result=self._command(name,args)
        self.revision+=1
        return {**result,'revision':self.revision}

    def _command(self,name,args):
        if not isinstance(args,dict):
            raise ValueError('命令参数必须是对象')
        if name=='reset':
            self.reset()
        elif name=='pause':
            if 'paused' in args and not isinstance(args['paused'],bool):raise ValueError('paused 必须为布尔值')
            self.paused=args.get('paused',not self.paused);self.drive[:]=0;self.drive_deadline=0
        elif name=='fire':
            reason,wait=self.fire_gate(0)
            return {'fired':self.shoot(0),'reason':reason,'wait':round(wait,2)}
        elif name=='drive':
            values=[args.get(k,0) for k in ('vx','vy','wz')]
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) for v in values):
                raise ValueError('底盘速度必须为数值标量')
            velocity=np.array(values,float)
            if not np.isfinite(velocity).all() or np.any(np.abs(velocity)>[1,1,1.5]):
                raise ValueError('底盘速度超范围')
            self.drive=velocity if not self.paused and self.winner is None else np.zeros(3)
            self.drive_deadline=clock.monotonic()+.35
        elif name=='aim':
            values=[args.get('yaw'),args.get('pitch')]
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) for v in values):
                raise ValueError('云台角度必须为数值标量')
            angles=np.array(values,float)
            if not np.isfinite(angles).all() or np.any(np.abs(angles)>[6.28,.45]):
                raise ValueError('云台角度超范围')
            self.robots[0].angles=angles
        elif name=='settings':
            candidate=self.settings.copy()
            for key,value in args.items():
                if key not in candidate:
                    raise ValueError('未知设置：'+key)
                if isinstance(candidate[key],bool):
                    if not isinstance(value,bool):raise ValueError(key+' 必须为布尔值')
                elif key in ('speed','fire_rate','target_speed','target_spin'):
                    limits={'speed':(13,25),'fire_rate':(1,20),'target_speed':(0,4),'target_spin':(0,4)}[key]
                    if isinstance(value,bool) or not isinstance(value,(float,int)) or not math.isfinite(value) or not limits[0]<=value<=limits[1]:
                        raise ValueError(key+' 超范围')
                elif key=='enemy_motion' and value not in ('static','strafe','circle','spin'):
                    raise ValueError('未知运动模式')
                elif key=='profile' and value not in ('cooling','burst'):
                    raise ValueError('未知热量配置')
                elif key=='view' and value not in ('overview','follow'):
                    raise ValueError('未知视角')
                candidate[key]=value
            if candidate['invulnerable'] and not self.settings['invulnerable']:
                for robot in self.robots:robot.hp=200
                self.winner=None
            if candidate['auto_aim']!=self.settings['auto_aim']:
                self.robots[0].angles=self.data.qpos[self.joints[0]].copy()
                self.robots[0].last_aim=None
                self.robots[0].search_origin=self.robots[0].angles.copy()
                self.robots[0].search_time=self.time
            if candidate['profile']!=self.settings['profile']:
                self.settings=candidate;self.reset()
            else:
                self.settings=candidate
        else:
            raise ValueError('未知命令')
        return {'ok':True}

    def state(self):
        result=[]
        for i,r in enumerate(self.robots):
            p=self.data.xpos[self.chassis[i]]
            R=self.data.xmat[self.chassis[i]].reshape(3,3)
            result.append({'team':TEAMS[i],'hp':r.hp,'heat':round(r.heat.value,2),'limit':r.heat.limit,
                           'locked':r.heat.locked,'shots':r.shots,'hits':r.hits,
                           'accuracy':round(100*r.hits/r.shots,1) if r.shots else 0,
                           'position':p.tolist(),'yaw':math.atan2(R[1,0],R[0,0]),
                           'gimbal':self.data.qpos[self.joints[i]].tolist(),
                           'tracking':r.tracker.ready(self.time),'reason':r.reason,
                           'fire_reason':('已暂停' if self.paused else r.fire_reason),'fire_wait':round(r.fire_wait,2),
                           'number':None if r.detection is None else r.detection.number,
                           'confidence':None if r.detection is None else round(r.detection.confidence,4),
                           'tracker_state':r.tracker.state,'switches':r.tracker.switches,
                           'armor_index':r.tracker.index,'aim_plate':r.aim_plate if r.aim is not None else None,
                           'center':None if r.tracker.x is None else r.tracker.center(self.time).tolist(),
                           'omega':None if r.tracker.x is None else float(r.tracker.x[7]),
                           'radii':None if r.tracker.x is None else [float(r.tracker.x[8]),r.tracker.r2],
                           'dz':r.tracker.dz,
                           'range':None if r.detection is None else round(float(np.linalg.norm(r.detection.tvec)),2),
                           'reprojection':None if r.detection is None else round(r.detection.error,3),
                           'aim_error':round(r.aim_error,3),'flight_ms':round(r.flight*1000,1)})
        return {'revision':self.revision,'time':round(self.time,2),'paused':self.paused,'winner':self.winner,'robots':result,
                'settings':self.settings.copy(),'events':self.events.copy(),'projectiles':len(self.projectiles)}
