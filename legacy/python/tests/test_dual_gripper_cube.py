"""Integration checks for physical wrists, grasp handoffs and solver execution."""
import numpy as np
import pytest


def test_two_perpendicular_wrists_grasp_a_free_cube():
    from dual_gripper_cube import DualGripperCube
    cube = DualGripperCube()
    cube.initialize_grasps()
    a = cube.data.xaxis[cube.model.joint('A_yaw').id]
    b = cube.data.xaxis[cube.model.joint('B_yaw').id]
    assert abs(np.dot(a, b)) < 1e-10
    assert cube.grasped == {'A': 'core', 'B': 'core'}
    assert not cube.data.eq_active[cube.model.equality('loading_fixture').id]
    assert all(len(cube.pad_contacts(h)) == 2 for h in ['A', 'B'])
    assert not cube.data.warning.number.any()


def test_wrist_drives_face_without_cube_motors():
    from dual_gripper_cube import DualGripperCube
    from gripper_plan import compile_moves
    from cube_solver import facelets_from_cube
    from kociemba.pykociemba.cubiecube import CubieCube, moveCube
    cube = DualGripperCube()
    cube.initialize_grasps()
    cube.execute(compile_moves(['R']))
    expected = CubieCube()
    expected.multiply(moveCube[1])
    assert facelets_from_cube(cube) == expected.toFaceCube().to_String()
    assert np.max(np.abs(cube.data.actuator_force[:6])) == 0
    assert cube.pose_error()[0] < .0002
    assert cube.pose_error()[1] < .02


def test_whole_rotation_changes_orientation_but_not_facelets():
    from dual_gripper_cube import DualGripperCube
    from gripper_plan import reorient
    from cube_solver import facelets_from_cube, SOLVED
    cube = DualGripperCube()
    cube.initialize_grasps()
    cube.execute(reorient('B', 1))
    assert facelets_from_cube(cube) == SOLVED
    core = cube.model.body('core').id
    np.testing.assert_allclose(cube.data.xmat[core].reshape(3, 3),
                               [[0, 0, -1], [0, 1, 0], [1, 0, 0]], atol=.02)
    assert not cube.data.warning.number.any()


def test_solver_plan_restores_cube_through_multiple_handoffs():
    from dual_gripper_cube import DualGripperCube
    from gripper_plan import compile_moves
    from cube_solver import facelets_from_cube, solve_facelets, SOLVED
    cube = DualGripperCube()
    for move in "R U F' L2 D B R' U2 F D'".split():
        cube.turn(move)
    state = facelets_from_cube(cube)
    cube.history.clear()
    solution = solve_facelets(state)
    cube.initialize_grasps()
    cube.execute(compile_moves(solution))
    assert facelets_from_cube(cube) == SOLVED
    assert cube.is_solved()
    assert cube.solved_pose_error()[0] < .0002
    assert cube.solved_pose_error()[1] < .02
    assert not cube.data.warning.number.any()


def test_missing_wrist_torque_cannot_complete_a_face_turn():
    from dual_gripper_cube import DualGripperCube
    from gripper_plan import compile_moves
    cube = DualGripperCube()
    cube.initialize_grasps()
    motor = cube.model.actuator('A_yaw_drive').id
    cube.model.actuator_gainprm[motor] = 0
    cube.model.actuator_biasprm[motor] = 0
    with pytest.raises(RuntimeError, match='tracking'):
        cube.execute(compile_moves(['R']))
    assert cube.history == []
