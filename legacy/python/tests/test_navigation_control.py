"""Controller behavior on geometry independent of the competition CAD."""
import numpy as np
from navigation.control import PID,TrajectoryPID
from navigation.dwa import DWA


class FlatTerrain:
    width=height=10.
    rows=cols=100
    heights=np.full((100,100),.1)
    valid=np.ones((100,100),dtype=bool)
    clearance=np.full((100,100),2.)
    def cell(self,x,y):return int(np.clip(y/.1,0,99)),int(np.clip(x/.1,0,99))
    def height_at(self,x,y):return .1


def test_pid_saturation_does_not_wind_up_and_reset_clears_history():
    pid=PID(1.,.5,.1,limit=.4)
    for _ in range(200):assert abs(pid.update(10.,.1))<=.4
    assert abs(pid.integral)<.01
    pid.reset()
    assert pid.update(0.,.1)==0.
    assert pid.update(-.1,.1)<0.


def test_tracker_corrects_position_and_wraps_heading():
    tracker=TrajectoryPID()
    current=np.array([2.,2.,.176,np.pi-.01])
    reference=np.array([1.95,2.,-np.pi+.01])
    command=tracker.update(current,reference,np.array([.2,0.,.1]),.1)
    assert command[0]>.2
    assert .1<command[2]<.3
    assert abs(command[1])<.002


def test_dwa_acceleration_window_and_progress():
    planner=DWA(FlatTerrain())
    pose=np.array([2.,2.,.176,0.])
    path=np.c_[np.linspace(2,4,21),np.full(21,2.),np.full(21,.1)]
    command,rollout=planner.plan(pose,np.zeros(3),path,path[-1],np.empty((0,2)))
    assert 0<command[0]<=planner.acceleration*.1+1e-9
    assert abs(command[2])<=planner.angular_acceleration*.1+1e-9
    assert len(rollout)>5 and rollout[-1,0]>pose[0]
    assert planner.candidates>4


def test_dwa_checks_braking_beyond_nominal_horizon():
    planner=DWA(FlatTerrain(),horizon=.2)
    pose=np.array([2.,2.,.176,0.])
    # At 0.45 m/s: horizon ends x=2.09, but stopping travel exceeds x=2.16.
    obstacles=np.array([[2.55,2.]])
    assert not planner.safe(pose,np.array([.45,0.,0.]),obstacles)


def test_dwa_rejects_boundary_even_when_terrain_cell_clamps():
    planner=DWA(FlatTerrain())
    assert not planner.safe(np.array([9.99,2.,.176,0.]),np.array([.4,0.,0.]),np.empty((0,2)))


def test_dwa_obstacle_changes_motion():
    planner=DWA(FlatTerrain())
    pose=np.array([2.,2.,.176,0.])
    path=np.c_[np.linspace(2,4,21),np.full(21,2.),np.full(21,.1)]
    current=np.array([.25,0.,0.])
    clear,_=planner.plan(pose,current,path,path[-1],np.empty((0,2)))
    blocked,_=planner.plan(pose,current,path,path[-1],np.array([[2.75,2.12]]))
    assert not np.allclose(clear,blocked)
    assert planner.safe(pose,blocked,np.array([[2.75,2.12]]))


def test_dwa_corner_target_does_not_point_through_blocked_cells():
    terrain=FlatTerrain()
    terrain.valid=terrain.valid.copy();terrain.valid[:30,:20]=False
    planner=DWA(terrain)
    pose=np.array([2.01,2.75,.176,2.8])
    path=np.array([[2.05,2.75,.1],[2.05,2.85,.1],[2.05,2.95,.1],
                   [2.05,3.05,.1],[1.95,3.15,.1],[1.85,3.15,.1],
                   [1.75,3.15,.1],[1.65,3.15,.1],[1.55,3.15,.1]])
    command,_=planner.plan(pose,np.zeros(3),path,path[-1],np.empty((0,2)))
    # Follow the visible passage; do not drive through its inflated corner.
    current=np.zeros(3)
    for _ in range(120):
        nearest=int(np.argmin(np.linalg.norm(path[:,:2]-pose[:2],axis=1)))
        command,_=planner.plan(pose,current,path[nearest:],path[-1],np.empty((0,2)))
        rollout,_=planner.rollout(pose,command[None,:])
        pose[[0,1,3]]=rollout[0,1];current=command
    assert pose[1]>3.0


def test_dwa_passes_an_isolated_obstacle_without_entering_footprint():
    planner=DWA(FlatTerrain());pose=np.array([2.,2.,.176,0.]);current=np.zeros(3)
    path=np.c_[np.linspace(2,5,31),np.full(31,2.),np.full(31,.1)]
    obstacles=np.array([[3.,2.1]])
    for _ in range(240):
        nearest=int(np.argmin(np.linalg.norm(path[:,:2]-pose[:2],axis=1)))
        command,trajectory=planner.plan(pose,current,path[nearest:],path[-1],obstacles)
        assert len(trajectory)>0
        rollout,_=planner.rollout(pose,command[None,:])
        pose[[0,1,3]]=rollout[0,1];current=command
        assert np.linalg.norm(pose[:2]-obstacles[0])>planner.radius
        if np.linalg.norm(pose[:2]-path[-1,:2])<.13:break
    assert np.linalg.norm(pose[:2]-path[-1,:2])<.13


def test_tracker_freezes_integral_after_dynamic_window_saturation():
    tracker=TrajectoryPID()
    pose=np.array([2.,2.,.176,0.])
    for _ in range(100):
        proposed=tracker.update(pose,np.array([2.02,2.,.01]),np.array([.3,0.,.2]),.1)
        tracker.accept(proposed,np.array([.2,0.,0.]))
    assert tracker.longitudinal.integral==0.
    assert tracker.heading.integral==0.


def test_dwa_swept_segment_cannot_cross_an_invalid_corner_cell():
    terrain=FlatTerrain();terrain.valid=terrain.valid.copy();terrain.valid[20,19]=False
    planner=DWA(terrain)
    assert not planner.safe(np.array([1.98,1.99,.176,np.pi/4]),np.array([.45,0.,0.]),np.empty((0,2)))


def test_dwa_overspeed_window_preserves_measured_acceleration_limits():
    planner=DWA(FlatTerrain())
    for velocity in [[1.6,.3,1.5],[-.3,-1.6,-1.5],[.3,.2,.4]]:
        low,high=planner.window(np.array(velocity),.45)
        for bound in [low,high]:
            assert abs(bound[0]-velocity[0])<=planner.acceleration*.1+1e-9
            assert abs(bound[1]-velocity[1])<=planner.acceleration*.1+1e-9
            assert abs(bound[2]-velocity[2])<=planner.angular_acceleration*.1+1e-9
        assert np.all(low<=high)


def test_dwa_reserves_tracking_margin_and_allows_motion_away_from_edge():
    terrain=FlatTerrain();terrain.valid=terrain.valid.copy();terrain.valid[:20,:]=False
    planner=DWA(terrain)
    assert not planner.safe(np.array([2.,2.06,.176,-np.pi/2]),np.array([.035,0.,0.]),np.empty((0,2)))
    assert planner.safe(np.array([2.,2.01,.176,np.pi/2]),np.array([.035,0.,0.]),np.empty((0,2)))
