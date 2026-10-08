"""Behavioral acceptance tests: physical motion, stability and armor placement."""
import math
import numpy as np
import pytest
import mujoco
from simulate import load, advance, yaw, demo_velocity, command

@pytest.mark.parametrize('velocity,axis', [((.4,0,0),0),((-.4,0,0),0),((0,.4,0),1),((0,-.4,0),1),((0,0,.7),2),((0,0,-.7),2)])
def test_motion(velocity,axis):
    model,data=load()
    advance(model,data,(0,0,0),.5)
    origin=data.qpos[:2].copy()
    heading=yaw(data)
    advance(model,data,velocity,2)
    result=np.r_[data.qpos[:2]-origin, yaw(data)-heading]
    expected=2*velocity[axis]
    assert result[axis] == pytest.approx(expected,abs=.12)
    assert np.max(np.abs(np.delete(result,axis))) < .06
    assert .065 < data.qpos[2] < .09
    assert np.linalg.norm(data.qpos[4:6]) < .04
    assert not data.warning.number.any()
    assert np.isfinite(data.qpos).all()

def test_rest_and_demo_stability():
    model,data=load()
    advance(model,data,(0,0,0),10)
    assert np.linalg.norm(data.qpos[:2]) < .01
    assert np.linalg.norm(data.qvel[:6]) < .01
    for i in range(12000):
        command(model,data,demo_velocity(i*.001))
        mujoco.mj_step(model,data)
        assert .06 < data.qpos[2] < .095
        assert np.linalg.norm(data.qpos[4:6]) < .05
    assert not data.warning.number.any()

def test_armor_geometry():
    model,data=load()
    advance(model,data,(0,0,0),.5)
    heights=[]
    for name,direction in [('front',[1,0]),('left',[0,1]),('rear',[-1,0]),('right',[0,-1])]:
        body=model.body(f'armor_{name}').id
        rotation=data.xmat[body].reshape(3,3)
        normal=rotation[:,0]
        assert normal[2] == pytest.approx(math.sin(math.pi/12),abs=.001)
        assert normal[:2]/np.linalg.norm(normal[:2]) == pytest.approx(direction,abs=.001)
        height=data.xpos[body,2]-.0625*math.cos(math.pi/12)-.0095*math.sin(math.pi/12)
        assert 0 < height < .4
        heights.append(height)
        assert model.body_jntnum[body] == 0
    assert max(heights)-min(heights) < .001
    assert model.nu==4
    assert model.nv==6+4+48
