"""Model-based checks: true rotation, plate association, ID and loss gating."""
import math
import numpy as np


def observation(t,plate=0,number='3'):
    from duel.rotor_tracking import ArmorObservation
    angle=.9*t+plate*math.pi/2
    center=np.array([3.+.15*t,.2,.25])
    position=center-[.255*math.cos(angle),.255*math.sin(angle),0]
    # Measured outward normal points from center to plate.
    return ArmorObservation(position,angle+math.pi,number,.99)


def test_recovers_center_spin_and_switches_across_pi():
    from duel.rotor_tracking import RotorTracker
    tracker=RotorTracker()
    for i in range(200):
        t=i*.05
        tracker.update([observation(t,int(t//1.3)%4)],t)
    assert tracker.ready(9.95)
    assert np.linalg.norm(tracker.center(9.95)-[4.4925,.2,.25]) < .06
    assert abs(tracker.x[7]-.9)<.1
    assert tracker.switches>=3
    assert np.linalg.eigvalsh(tracker.P).min()>-1e-9
    assert len(tracker.armor_positions(10.1))==4


def test_wrong_id_and_dropouts_cannot_trigger_fire():
    from duel.rotor_tracking import RotorTracker
    tracker=RotorTracker()
    for i in range(20):tracker.update([observation(i*.05)],i*.05)
    before=tracker.last_seen
    tracker.update([observation(1.,number='4')],1.)
    assert tracker.last_seen==before
    tracker.update([],1.3)
    assert not tracker.ready(1.3)
    tracker.update([],2.)
    assert tracker.state=='LOST'


def test_future_plate_motion_is_curved():
    from duel.rotor_tracking import RotorTracker
    tracker=RotorTracker()
    for i in range(100):tracker.update([observation(i*.05)],i*.05)
    t=5.2
    expected=observation(t).position
    assert min(np.linalg.norm(p-expected) for p in tracker.armor_positions(t))<.05


def test_alternate_geometry_does_not_become_chassis_motion():
    from duel.rotor_tracking import RotorTracker,ArmorObservation
    tracker=RotorTracker()
    for i in range(400):
        t=i*.05;index=int(t//1)%4;yaw=.8*t+index*math.pi/2
        radius=.32 if index%2 else .20
        point=np.array([3+radius*math.cos(yaw),radius*math.sin(yaw),.33 if index%2 else .25])
        tracker.update([ArmorObservation(point,yaw,'3',.99)],t)
    assert np.linalg.norm(tracker.center(t)-[3,0,.25])<.04
    assert abs(tracker.x[8]-.20)<.025
    assert abs(tracker.r2-.32)<.025
    assert abs(tracker.dz-.08)<.015


def test_long_frame_gap_requires_reacquisition():
    from duel.rotor_tracking import RotorTracker
    tracker=RotorTracker()
    for i in range(4):tracker.update([observation(i*.05)],i*.05)
    tracker.update([observation(10)],10)
    assert tracker.state=='DETECTING'
    assert not tracker.ready(10)
