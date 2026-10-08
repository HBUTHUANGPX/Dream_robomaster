"""Competition heat and analytic projectile helpers (SI units)."""
from dataclasses import dataclass
import math
import numpy as np


class Heat:
    def __init__(self, profile='cooling'):
        self.limit, self.rate = {'cooling': (40., 12.), 'burst': (170., 5.)}[profile]
        self.value = 0.
        self.locked = False
        self.permanent = False
        self.tick = 0

    def cool(self, time):
        tick = math.floor(time*10 + 1e-8)
        self.value = max(0., self.value - max(0, tick-self.tick)*self.rate/10)
        self.tick = tick
        if self.value == 0 and not self.permanent:
            self.locked = False

    def fire(self, time, protect=True):
        self.cool(time)
        if self.locked or (protect and self.value+10 > self.limit+1e-8):
            return False
        self.value += 10
        self.locked = self.value > self.limit
        # Follow the official flowchart's >= boundary, not the prose's >.
        self.permanent = self.permanent or self.value >= self.limit+100
        return True


def segment_box(start, end, half_size):
    """First intersection fraction with a local axis-aligned box, or None."""
    lo, hi = 0., 1.
    for a, d, size in zip(start, end-start, half_size):
        if abs(d) < 1e-12:
            if abs(a) > size:
                return None
        else:
            u, v = sorted(((-size-a)/d, (size-a)/d))
            lo, hi = max(lo, u), min(hi, v)
            if lo > hi:
                return None
    return lo


def intercept(relative_position, target_velocity, speed, muzzle_velocity):
    """Low-arc solution including target motion and inherited muzzle velocity."""
    p = np.asarray(relative_position, float)
    v = np.asarray(target_velocity) - muzzle_velocity
    flight = max(.001, np.linalg.norm(p)/speed)
    for _ in range(20):
        displacement = p + v*flight + np.array([0., 0., 4.905*flight*flight])
        next_flight = np.linalg.norm(displacement)/speed
        if next_flight > 3 or not np.isfinite(next_flight):
            return None, None
        if abs(next_flight-flight) < 1e-8:
            break
        flight = next_flight
    return displacement/np.linalg.norm(displacement), flight


@dataclass
class Projectile:
    owner: int
    position: np.ndarray
    velocity: np.ndarray
    born: float


def segment_ellipsoid(start, end, radii):
    a=start/radii;d=(end-start)/radii
    c=a @ a-1
    if c<=0:return 0.
    aa=d @ d;b=a @ d
    discriminant=b*b-aa*c
    if aa<1e-15 or discriminant<0:return None
    fraction=(-b-math.sqrt(discriminant))/aa
    return float(fraction) if 0<=fraction<=1 else None


def segment_cylinder(start, end, radius, height):
    """Finite local Z cylinder; height is its half-length."""
    delta=end-start
    if np.linalg.norm(start[:2])<=radius and abs(start[2])<=height:return 0.
    candidates=[]
    a=delta[:2] @ delta[:2];b=start[:2] @ delta[:2]
    c=start[:2] @ start[:2]-radius*radius
    disc=b*b-a*c
    if a>1e-15 and disc>=0:
        for f in ((-b-math.sqrt(disc))/a,(-b+math.sqrt(disc))/a):
            if 0<=f<=1 and abs(start[2]+f*delta[2])<=height:candidates.append(f)
    if abs(delta[2])>1e-15:
        for z in (-height,height):
            f=(z-start[2])/delta[2]
            if 0<=f<=1 and np.linalg.norm(start[:2]+f*delta[:2])<=radius:candidates.append(f)
    return float(min(candidates)) if candidates else None
