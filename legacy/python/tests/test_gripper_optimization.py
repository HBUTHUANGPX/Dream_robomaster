"""Post-compilation optimizations preserve robot state and cube operations."""
import numpy as np
import pytest


def test_conversion_contains_no_axial_travel():
    from gripper_plan import compile_moves
    plan = compile_moves("U R2 F' D B L2".split())
    assert not any(a.kind == 'slide' for a in plan)


def test_post_pass_removes_redundant_regrips_without_changing_turns():
    from gripper_plan import compile_moves, optimize_plan, replay_plan
    raw = compile_moves("U R2 F' D B L2".split())
    original = list(raw)
    optimized = optimize_plan(raw)
    assert raw == original
    assert len(optimized) < len(raw)
    assert sum(a.kind == 'jaw' for a in optimized) < sum(a.kind == 'jaw' for a in raw)
    assert replay_plan(optimized) == replay_plan(raw)
    assert [(a.hand, a.target, a.mode) for a in optimized if a.kind == 'yaw'] == [
        (a.hand, a.target, a.mode) for a in raw if a.kind == 'yaw']
    assert optimize_plan(optimized) == optimized


def test_raw_and_optimized_plans_agree_for_every_initial_orientation():
    from gripper_plan import compile_moves, optimize_plan, replay_plan
    from tests.test_gripper_plan import orientations
    for orientation in orientations():
        raw = compile_moves("R U2 F' L B2 D".split(), orientation)
        optimized = optimize_plan(raw, orientation)
        assert replay_plan(raw, orientation) == replay_plan(optimized, orientation)


def test_optimizer_preserves_last_support_and_final_grasps():
    from gripper_plan import compile_moves, optimize_plan, replay_plan
    for moves in [[], ['R'], ['U2'], ['F', 'R', 'B']]:
        result = replay_plan(optimize_plan(compile_moves(moves)))
        assert result['grasped'] == ['A', 'B']
        assert result['yaw_rad'] == {'A': 0., 'B': 0.}
        assert result['moves'] == moves


def test_scene_has_no_translation_axis():
    import mujoco
    from dual_gripper_cube import DualGripperCube
    cube = DualGripperCube()
    assert not np.any(cube.model.jnt_type == mujoco.mjtJoint.mjJNT_SLIDE)


def test_opening_clears_cube_and_empty_wrist_stays_in_place():
    from dual_gripper_cube import DualGripperCube
    from gripper_plan import compile_moves, optimize_plan
    cube = DualGripperCube()
    cube.initialize_grasps()
    origins = [cube.data.xpos[cube.model.body(h+'_wrist').id].copy() for h in ['A', 'B']]
    cube.execute(optimize_plan(compile_moves(['R', 'U'])))
    for h, origin in zip(['A', 'B'], origins):
        np.testing.assert_allclose(cube.data.xpos[cube.model.body(h+'_wrist').id], origin, atol=1e-12)
    assert cube.clearance_report['rotation_steps_checked'] > 1000
    assert cube.clearance_report['forbidden_contacts'] == 0
    assert cube.clearance_report['min_open_gap_m'] >= cube.required_open_gap


def test_replay_rejects_old_slide_actions():
    from gripper_plan import Action, replay_plan
    with pytest.raises(ValueError):
        replay_plan([Action('release', 'A'), Action('jaw', 'A', 0), Action('slide', 'A', .075)])


def test_optimized_and_raw_execution_match_physical_cube():
    from dual_gripper_cube import DualGripperCube
    from gripper_plan import compile_moves, optimize_plan
    from cube_solver import facelets_from_cube
    raw = compile_moves(['R', 'U', "F'"])
    optimized = optimize_plan(raw)
    results = []
    for plan in [raw, optimized]:
        cube = DualGripperCube()
        cube.initialize_grasps()
        cube.execute(plan)
        results.append((facelets_from_cube(cube), cube.orientation.copy(), cube.grasped.copy()))
        assert cube.pose_error()[0] < .0002
        assert cube.pose_error()[1] < .02
        assert cube.clearance_report['forbidden_contacts'] == 0
    assert results[0][0] == results[1][0]
    np.testing.assert_array_equal(results[0][1], results[1][1])
    assert results[0][2] == results[1][2] == {'A': 'core', 'B': 'core'}
