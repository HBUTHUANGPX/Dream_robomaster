"""High-performance finger variant: independent timing and physical feedback."""
import numpy as np
import pytest


def test_independent_speed_validation():
    from dual_gripper_cube import DualGripperCube
    for options in [{'wrist_speed': 0}, {'jaw_speed': -1}, {'jaw_speed': float('nan')}]:
        with pytest.raises(ValueError):
            DualGripperCube(**options)


def test_physical_jaw_stroke_and_wrist_at_independent_rates():
    from dual_gripper_cube import DualGripperCube
    from gripper_plan import compile_moves
    cube = DualGripperCube(wrist_speed=8, jaw_speed=32)
    cube.initialize_grasps()
    cube.execute(compile_moves(['R']))
    wrist = [m for m in cube.motion_checks if m['actuator'].endswith('yaw_drive')]
    fingers = [m for m in cube.motion_checks if m['actuator'].endswith('fingers_actuator')]
    assert all(abs(m['duration_s']-.85/8) < .0001 for m in wrist)
    assert all(25 < m['peak_joint_velocity_rad_s'] < 30 for m in wrist)
    assert all(m['first_reached_s'] <= .45/32 for m in fingers)
    assert all(m['peak_actuator_force'] <= 40.0001 for m in fingers)
    assert cube.clearance_report['forbidden_contacts'] == 0
    assert cube.pose_error()[0] < .0002
    assert cube.pose_error()[1] < .02
    assert not cube.data.warning.number.any()
    assert not np.any(cube.data.xfrc_applied)


def test_fast_solver_restores_scramble_with_no_external_grasp_constraints():
    import mujoco
    from dual_gripper_cube import DualGripperCube
    from cube_solver import solve_facelets, facelets_from_cube, SOLVED
    from gripper_plan import compile_moves, optimize_plan
    cube = DualGripperCube(wrist_speed=8, jaw_speed=32)
    for move in "R U F' L2".split():
        cube.turn(move)
    solution = solve_facelets(facelets_from_cube(cube))
    cube.history.clear()
    for i in range(cube.model.neq):
        if cube.model.eq_type[i] in (mujoco.mjtEq.mjEQ_WELD, mujoco.mjtEq.mjEQ_CONNECT):
            names = [cube.model.body(int(b)).name for b in
                     (cube.model.eq_obj1id[i], cube.model.eq_obj2id[i])]
            assert names[0].startswith(('A_', 'B_')) == names[1].startswith(('A_', 'B_'))
    cube.initialize_grasps()
    cube.execute(optimize_plan(compile_moves(solution)))
    assert facelets_from_cube(cube) == SOLVED
    assert cube.is_solved()
    assert cube.clearance_report['forbidden_contacts'] == 0
    assert np.max(np.abs(cube.data.actuator_force[:6])) == 0
    assert not cube.data.warning.number.any()
