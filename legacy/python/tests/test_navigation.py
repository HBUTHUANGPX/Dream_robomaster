import numpy as np
import pytest
from navigation.terrain import Terrain
from navigation.task import NavigationTask

@pytest.fixture(scope='module')
def terrain():return Terrain()

def test_connected_flat_ramps_highlands(terrain):
    for goal in [[1.95,5.95],[6.75,9.45],[8.85,13.85],[12.95,21.95],[7.45,23.95]]:
        path=terrain.plan(terrain.spawn,goal)
        assert np.linalg.norm(path[-1,:2]-goal)<.15
        assert np.max(np.abs(np.diff(path[:,2])))<.085
        assert all(terrain.valid[terrain.cell(*p[:2])] for p in path)
    assert terrain.plan(terrain.spawn,[6.75,9.45])[:,2].max()>.69

def test_reject_isolated_roof(terrain):
    with pytest.raises(ValueError):terrain.plan(terrain.spawn,[1.2,1.35])

def test_graph_random_destinations(terrain):
    y,x=np.where(terrain.reachable)
    rng=np.random.default_rng(7)
    for i in rng.choice(len(y),50,replace=False):
        path=terrain.plan(terrain.spawn,terrain.point((y[i],x[i])))
        assert len(path)>0

def test_sensors_and_flat_goal():
    task=NavigationTask()
    assert len(task.scan)>700  # catches inverted sensor mast self-occlusion
    assert task.mapper.matched>30
    task.set_goal(8.,4.)
    for _ in range(70):
        task.step()
        if task.goal is None:break
    assert task.status=='已到达'
    assert np.linalg.norm(task.data.qpos[:2]-[8.05,4.05])<.2
    assert task.state()['localization_error']<.10
    assert not task.data.warning.number.any()

@pytest.fixture(scope='module')
def task_fixture():return NavigationTask()

def test_fault_cannot_resume_motion(task_fixture):
    task=task_fixture;task.reset();task.set_goal(8.,4.)
    task.data.warning[0].number=1
    task.paused=False
    before=task.data.time
    task.step()
    assert task.data.time==before
    assert task.paused and not task.data.ctrl.any()
    assert not task.velocity.any() and len(task.local)==0
    task.reset()

def test_obstacle_stops_rotation_and_clears_rollout(task_fixture):
    task=task_fixture;task.reset();task.set_goal(8.4,4.25)
    pose=task.mapper.pose.copy()
    task.scan=np.array([[pose[0]+.4,pose[1],pose[2]+.3]])
    velocity=task.control()
    assert np.allclose(velocity,0)
    assert len(task.local)==0
    assert '障碍' in task.status
    task.scan=np.empty((0,3))
    assert task.control()[0]>0
    assert task.status=='导航中'

def test_localization_recovery_status(task_fixture):
    task=task_fixture;task.reset();task.set_goal(8.4,4.05)
    task.localization_failures=8
    assert np.allclose(task.control(),0)
    assert '定位失效' in task.status
    task.localization_failures=0;task.scan=np.empty((0,3))
    assert task.control()[0]>0
    assert task.status=='导航中'


def test_persistent_obstacle_exits_goal_after_progress_timeout(task_fixture):
    task=task_fixture;task.reset();task.set_goal(8.4,4.25)
    pose=task.mapper.pose.copy()
    task.scan=np.array([[pose[0]+.4,pose[1],pose[2]+.3]])
    task.data.time+=16.
    assert np.allclose(task.control(),0)
    assert task.goal is None and '无进展' in task.status
