"""Distinguish accelerated actuator references from achieved physical motion."""
import numpy as np
import pytest


@pytest.mark.parametrize('speed', [0, -1, float('nan'), float('inf')])
def test_invalid_speed_is_rejected(speed):
    from dual_gripper_cube import DualGripperCube
    with pytest.raises(ValueError):
        DualGripperCube(speed=speed)


def test_wrist_reaches_sixteen_times_peak_speed_with_existing_actuator():
    from dual_gripper_cube import DualGripperCube
    from gripper_plan import compile_moves
    peaks = []
    for speed in [1, 16]:
        cube = DualGripperCube(speed=speed)
        cube.initialize_grasps()
        cube.execute(compile_moves(['R']))
        turn = next(m for m in cube.motion_checks if m['actuator'] == 'A_yaw_drive')
        peaks.append(turn['peak_joint_velocity_rad_s'])
        assert abs(turn['actual_at_ramp_end']+np.pi/2) < .025
        assert turn['peak_actuator_force'] <= 15
    assert 15 < peaks[1]/peaks[0] < 17


def test_fast_jaw_reference_does_not_bypass_physical_clearance_wait():
    from dual_gripper_cube import DualGripperCube
    cube = DualGripperCube(speed=16)
    cube.initialize_grasps()
    cube._release('A')
    cube._move('A_fingers_actuator', 0, .3)
    opening = cube.motion_checks[-1]
    assert .018 < opening['duration_s'] < .020
    assert opening['actual_at_ramp_end'] < cube.required_open_gap
    cube._wait_open('A')
    assert cube.open_gap('A') >= cube.required_open_gap
    motor = cube.model.actuator('A_fingers_actuator').id
    np.testing.assert_array_equal(cube.model.actuator_forcerange[motor], [-5, 5])
