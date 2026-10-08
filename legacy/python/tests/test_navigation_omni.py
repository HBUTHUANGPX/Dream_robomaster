import numpy as np
from navigation.control import TrajectoryPID
from navigation.dwa import DWA
from navigation.power import MecanumDrive


class FlatTerrain:
    width=height=20.;rows=cols=200
    heights=np.full((200,200),.1);valid=np.ones((200,200),bool);clearance=np.full((200,200),3.)
    def height_at(self,x,y):return .1


def path_to(start,end):
    return np.c_[np.linspace(start,end,51),np.full(51,.1)]


def test_tracker_uses_lateral_pid_without_turning_to_face_position_error():
    tracker=TrajectoryPID()
    cmd=tracker.update(np.array([5.,5.,.176,0.]),np.array([5.,5.1,0.]),np.zeros(3),.1)
    assert cmd[1]>0 and abs(cmd[2])<1e-8


def test_dwa_moves_sideways_and_backwards_immediately():
    planner=DWA(FlatTerrain());pose=np.array([5.,5.,.176,0.])
    for target,axis,sign in [([5.,8.],1,1),([2.,5.],0,-1)]:
        path=path_to(pose[:2],target)
        cmd,tr=planner.plan(pose,np.zeros(3),path,path[-1],np.empty((0,2)))
        assert cmd[axis]*sign>.02 and abs(cmd[2])<.05
        assert len(tr)>0


def test_dwa_can_translate_and_rotate_in_one_cycle():
    planner=DWA(FlatTerrain());pose=np.array([5.,5.,.176,0.]);path=path_to(pose[:2],[8.,5.])
    cmd,tr=planner.plan(pose,np.zeros(3),path,path[-1],np.empty((0,2)),yaw_rate=.6)
    assert cmd[0]>.02 and cmd[2]>.02
    assert tr[-1,0]>pose[0] and tr[-1,2]>pose[3]


def test_lateral_braking_tail_is_checked():
    planner=DWA(FlatTerrain(),horizon=.2)
    assert not planner.safe(np.array([5.,5.,.176,0.]),np.array([0.,1.,0.]),np.array([[5.,5.85]]))


def test_power_allocation_enforces_budget_including_stall_losses():
    drive=MecanumDrive(budget=120.)
    for speed in [[25,25,25,25],[0,0,0,0],[-20,20,20,-20]]:
        torque=np.array([10.,10.,10.,10.])
        limited=drive.limit_torque(torque,np.array(speed,dtype=float))
        assert drive.electrical(limited,np.array(speed))<=drive.budget+1e-8
        assert np.all(abs(limited)<=10)
    small=MecanumDrive(budget=20.)
    assert np.max(abs(small.limit_torque(np.full(4,10.),np.zeros(4))))<10.


def test_power_model_accounts_for_direction_and_combined_wheel_limits():
    drive=MecanumDrive()
    forward=np.array([1.,0.,0.]);lateral=np.array([0.,1.,0.])
    assert drive.predict(lateral,lateral,.1)[0]>drive.predict(forward,forward,.1)[0]
    # Pure translation may fit while translation+strafe+yaw exceeds a wheel's RPM.
    assert not drive.feasible(np.array([[1.4,1.4,1.2]]),np.zeros(3),.1)[0]


def test_physical_sideways_goal_does_not_require_heading_alignment():
    from navigation.task import NavigationTask
    task=NavigationTask();task.set_goal(7.5,5.0)
    headings=[]
    for _ in range(70):
        task.step();headings.append(task.mapper.pose[3])
        if task.goal is None:break
    assert task.status=='已到达'
    assert max(abs(np.asarray(headings)))<.3
    assert task.power_peak_w<=task.drive.budget+1e-7


def test_physical_simultaneous_rotation_and_translation():
    from navigation.task import NavigationTask
    task=NavigationTask();task.set_goal(7.5,6.0);task.set_yaw_rate(.6)
    moving_spin=0
    for _ in range(80):
        task.step()
        moving_spin+=int(np.linalg.norm(task.measured_velocity[:2])>.2 and task.measured_velocity[2]>.2)
        if task.goal is None:break
    assert task.status=='已到达'
    assert moving_spin>=10
    assert abs(task.mapper.pose[3])>.8
    assert task.power_peak_w<=task.drive.budget+1e-7
    assert not task.data.warning.number.any()
