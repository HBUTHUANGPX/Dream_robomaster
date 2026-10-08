"""PID tracking of the previous local trajectory's time-aligned reference."""
import math
import numpy as np


def wrap(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi


class PID:
    def __init__(self,kp,ki,kd,limit,derivative_tau=.15):
        self.kp=kp;self.ki=ki;self.kd=kd;self.limit=limit;self.tau=derivative_tau
        self.reset()

    def reset(self):
        self.integral=0.;self.integral_before=0.;self.previous=None;self.derivative=0.

    def update(self,error,dt):
        if not np.isfinite([error,dt]).all() or dt<=0:raise ValueError('Invalid PID input')
        raw=0. if self.previous is None else (error-self.previous)/dt
        self.derivative+=dt/(self.tau+dt)*(raw-self.derivative)
        self.previous=error
        self.integral_before=self.integral
        integral=self.integral+error*dt
        value=self.kp*error+self.ki*integral+self.kd*self.derivative
        # Conditional integration: do not accumulate error into output saturation.
        if abs(value)<=self.limit or value*error<0:self.integral=integral
        return float(np.clip(self.kp*error+self.ki*self.integral+self.kd*self.derivative,-self.limit,self.limit))


class TrajectoryPID:
    """Independent body X, body Y and yaw PID; heading does not steer translation."""
    def __init__(self):
        self.longitudinal=PID(1.4,.16,.06,.25)
        self.lateral=PID(1.4,.16,.06,.25)
        self.heading=PID(1.8,.08,.06,.30)
        self.error=np.zeros(3)

    def reset(self):
        self.longitudinal.reset();self.lateral.reset();self.heading.reset();self.error[:]=0.

    def update(self,pose,reference,feedforward,dt):
        delta=reference[:2]-pose[:2]
        c,s=math.cos(pose[3]),math.sin(pose[3])
        ex=c*delta[0]+s*delta[1];ey=-s*delta[0]+c*delta[1]
        yaw_error=wrap(reference[2]-pose[3])
        self.error[:]=[ex,ey,yaw_error]
        return np.array([feedforward[0]+self.longitudinal.update(ex,dt),
                         feedforward[1]+self.lateral.update(ey,dt),
                         feedforward[2]+self.heading.update(yaw_error,dt)])

    def accept(self,proposed,executed):
        """Freeze integration when the downstream dynamic window clips a command."""
        if abs(proposed[0]-executed[0])>1e-9:
            self.longitudinal.integral=self.longitudinal.integral_before
        if abs(proposed[1]-executed[1])>1e-9:
            self.lateral.integral=self.lateral.integral_before
        if abs(proposed[2]-executed[2])>1e-9:
            self.heading.integral=self.heading.integral_before
