"""Wheel-domain constraints and an explicitly uncalibrated electrical-power model.

No competition power limit or measured motor efficiency is implied by defaults.
"""
import numpy as np
from simulate import RADIUS,LEVER


class MecanumDrive:
    max_wheel_speed=35.
    max_torque=10.
    efficiency=.80
    torque_loss=.12  # W/(Nm)^2, placeholder copper/drive loss coefficient.
    speed_loss=.003  # W/(rad/s)^2, placeholder speed-dependent loss.
    idle=4.
    # Unequal rolling resistance is a model assumption, not a hardware measurement.
    rolling=np.array([.035,.070])
    J=np.array([[1,-1,-LEVER],[1,1,LEVER],[1,1,-LEVER],[1,-1,LEVER]])/RADIUS

    def __init__(self,mass=21.258,budget=120.):
        if not np.isfinite(budget) or budget<20:raise ValueError('Power budget must be at least 20 W')
        self.mass=mass;self.inertia=mass*(.57**2+.57**2)/12;self.budget=float(budget)
        self.force_to_torque=np.linalg.pinv(self.J.T)

    def wheels(self,commands):return np.asarray(commands)@self.J.T

    def electrical(self,torque,omega):
        torque=np.asarray(torque);omega=np.asarray(omega)
        return (np.maximum(torque*omega,0).sum(axis=-1)/self.efficiency
                +self.torque_loss*np.square(torque).sum(axis=-1)
                +self.speed_loss*np.square(omega).sum(axis=-1)+self.idle)

    def predict(self,commands,current,dt,gravity=None):
        commands=np.atleast_2d(np.asarray(commands,dtype=float))
        acceleration=(commands-np.asarray(current))/dt
        # Body-frame derivative plus rotating-frame term gives inertial acceleration.
        acceleration[:,:2]+=commands[:,2,None]*np.c_[-commands[:,1],commands[:,0]]
        force=np.empty_like(commands)
        force[:,:2]=self.mass*acceleration[:,:2]
        force[:,:2]+=self.mass*9.81*self.rolling*np.clip(commands[:,:2]/.05,-1,1)
        if gravity is not None:force[:,:2]+=self.mass*np.asarray(gravity)
        force[:,2]=self.inertia*acceleration[:,2]+.15*commands[:,2]
        torque=force@self.force_to_torque.T
        omega=self.wheels(commands)
        return self.electrical(torque,omega),torque,omega

    def feasible(self,commands,current,dt,gravity=None):
        power,torque,omega=self.predict(commands,current,dt,gravity)
        return ((power<=self.budget)&(abs(torque).max(axis=1)<=self.max_torque)
                &(abs(omega).max(axis=1)<=self.max_wheel_speed))

    def limit_torque(self,requested,omega):
        """Scale all requested wheel torques together; no regenerative power credit."""
        torque=np.clip(requested,-self.max_torque,self.max_torque)
        base=self.idle+self.speed_loss*np.square(omega).sum()
        linear=np.maximum(torque*omega,0).sum()/self.efficiency
        quadratic=self.torque_loss*np.square(torque).sum()
        available=max(0.,self.budget-base)
        if linear+quadratic<=available:return torque
        # Stable positive root of quadratic*s² + linear*s = available.
        denom=linear+np.sqrt(linear*linear+4*quadratic*available)
        scale=0. if denom==0 else min(1.,2*available/denom)
        return torque*scale
