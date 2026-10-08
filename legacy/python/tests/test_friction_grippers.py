"""Friction-only grasping, actuator drive and loss-of-friction controls."""
import mujoco
import numpy as np


def test_gripper_cube_has_no_cross_body_grasp_constraints():
    from dual_gripper_cube import DualGripperCube
    cube = DualGripperCube()
    for i in range(cube.model.neq):
        if cube.model.eq_type[i] not in (mujoco.mjtEq.mjEQ_WELD, mujoco.mjtEq.mjEQ_CONNECT):
            continue
        names = [cube.model.body(int(b)).name for b in
                 (cube.model.eq_obj1id[i], cube.model.eq_obj2id[i])]
        hands = [name.startswith(('A_', 'B_')) for name in names]
        assert hands[0] == hands[1], names
    cube.initialize_grasps()
    assert not cube.data.eq_active[cube.model.equality('loading_fixture').id]
    before = cube.data.xpos[cube.model.body('core').id].copy()
    cube.advance(2)
    assert np.linalg.norm(cube.data.xpos[cube.model.body('core').id]-before) < .001
    assert all(len(cube.pad_contacts(h)) == 2 for h in ['A', 'B'])


def test_friction_supports_gravity_and_zero_friction_slips():
    from dual_gripper_cube import DualGripperCube
    drops = []
    for remove_friction in [False, True]:
        cube = DualGripperCube()
        cube.initialize_grasps()
        cube._release('A')
        cube._move('A_fingers_actuator', 0, .3)
        cube._wait_open('A')
        # Rotate B so its opposing pad normals are horizontal. Gravity must now
        # be carried tangentially by friction, rather than by the bottom pad.
        cube._move('B_yaw_drive', np.pi/2, 1.2, joint_name='B_yaw')
        cube.advance(.5)
        core = cube.model.body('core').id
        height = float(cube.data.xpos[core, 2])
        if remove_friction:
            cube.model.geom_friction[:] = 0
        cube.advance(1)
        drops.append(height-float(cube.data.xpos[core, 2]))
    assert abs(drops[0]) < .001
    assert drops[1] > .01
