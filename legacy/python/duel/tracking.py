"""Image/PnP EKF. No simulator imports or access to opponent state."""
import numpy as np


class Tracker:
    def __init__(self):
        self.x = None
        self.P = np.eye(6)
        self.time = 0.
        self.last_seen = -1.
        self.count = 0
        self.rejected = 0
        self.innovation = 0.

    def observe(self, tvec, camera_position, camera_rotation, time):
        z = np.asarray(tvec, float)
        if not np.isfinite(z).all() or z[2] <= .15:
            return False
        world = camera_position + camera_rotation @ z
        if self.x is None or time-self.last_seen > .4:
            self.x = np.r_[world, np.zeros(3)]
            self.P = np.diag([.08,.08,.08,1.,1.,1.])
            self.time = self.last_seen = time
            self.count = 1
            return True
        dt = max(0., time-self.time)
        F = np.eye(6); F[:3,3:] = np.eye(3)*dt
        G = np.vstack((np.eye(3)*dt**2/2, np.eye(3)*dt))
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * 9
        self.time = time
        c = camera_rotation.T @ (self.x[:3]-camera_position)
        if c[2] <= .1:
            return False
        # Fuse image bearing and PnP range with different noise magnitudes.
        measurement = np.array([z[0]/z[2], z[1]/z[2], z[2]])
        predicted = np.array([c[0]/c[2], c[1]/c[2], c[2]])
        J = np.array([[1/c[2],0,-c[0]/c[2]**2],
                      [0,1/c[2],-c[1]/c[2]**2], [0,0,1.]])
        H = np.zeros((3,6)); H[:,:3] = J @ camera_rotation.T
        R = np.diag([.0018**2, .0018**2, max(.025,.012*z[2]**2)**2])
        residual = measurement-predicted
        S = H @ self.P @ H.T + R
        self.innovation = float(residual @ np.linalg.solve(S,residual))
        if self.innovation > 20:
            self.rejected += 1
            return False
        K = np.linalg.solve(S,H @ self.P).T
        self.x += K @ residual
        A = np.eye(6)-K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.P = (self.P+self.P.T)/2
        self.last_seen = time
        self.count += 1
        return True

    def position(self, time):
        if self.x is None:
            return None
        return self.x[:3] + self.x[3:] * max(0.,time-self.time)

    def ready(self, time):
        return self.x is not None and self.count >= 4 and time-self.last_seen <= .18
