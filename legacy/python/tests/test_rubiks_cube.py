"""Physical acceptance tests for face reassignment and reversible cube moves."""
import numpy as np
import pytest


@pytest.mark.parametrize('move,initial,expected', [
    ('R', [1, -1, 1], [1, 1, 1]), ('L', [-1, 1, -1], [-1, 1, 1]),
    ('U', [1, -1, 1], [-1, -1, 1]), ('D', [-1, 1, -1], [-1, -1, -1]),
    ('F', [1, -1, 1], [1, -1, -1]), ('B', [-1, 1, -1], [1, 1, -1]),
])
def test_face_turn_moves_real_corner(move, initial, expected):
    from rubiks_cube import RubiksCube, PITCH, ORIGIN
    cube = RubiksCube()
    cube.turn(move)
    # Hand-computed corner positions, clockwise viewed from the selected face.
    piece = cube.piece_at_initial(initial)
    np.testing.assert_allclose(cube.data.xpos[piece],
                               ORIGIN + PITCH*np.array(expected), atol=2e-5)
    assert cube.pose_error()[0] < 2e-5
    assert cube.pose_error()[1] < 2e-3
    assert not cube.data.warning.number.any()


def test_mid_turn_is_continuous_rigid_motion():
    from rubiks_cube import RubiksCube, ORIGIN, PITCH
    cube = RubiksCube()
    corner = cube.piece_at_initial((1, -1, 1))
    other = cube.piece_at_initial((-1, -1, 1))
    corner_trace, other_trace, angles = [], [], []

    def observe(current):
        corner_trace.append(current.data.xpos[corner].copy())
        other_trace.append(current.data.xpos[other].copy())
        angles.append(float(current.data.qpos[current.model.joint('hinge_R').qposadr[0]]))

    cube.turn('R', observe)
    # A finite-gain motor lags its command. Check the actual hinge's circular arc,
    # independently of the production rotation helper, as well as tracking error.
    theta = angles[374]
    assert theta == pytest.approx(-np.pi/4, abs=.05)
    np.testing.assert_allclose(corner_trace[374],
                               ORIGIN + PITCH*np.array([1, -np.cos(theta)-np.sin(theta),
                                                        np.cos(theta)-np.sin(theta)]),
                               atol=2e-5)
    assert np.max(np.linalg.norm(np.diff(corner_trace, axis=0), axis=1)) < .0003
    np.testing.assert_allclose(np.array(other_trace),
                               np.broadcast_to(ORIGIN+PITCH*np.array([-1, -1, 1]),
                                               np.shape(other_trace)), atol=2e-5)


def test_missing_motor_force_cannot_fake_a_completed_turn():
    from rubiks_cube import RubiksCube
    cube = RubiksCube()
    cube.model.actuator_gainprm[:, 0] = 0
    cube.model.actuator_biasprm[:, 1:3] = 0
    with pytest.raises(RuntimeError, match='alignment'):
        cube.turn('R')
    assert cube.history == []


@pytest.mark.parametrize('face', list('RLUDFB'))
def test_four_quarter_turns_restore_physical_pose(face):
    from rubiks_cube import RubiksCube
    cube = RubiksCube()
    for _ in range(4):
        cube.turn(face)
    assert cube.is_solved()
    assert cube.solved_pose_error()[0] < 2e-5
    assert cube.solved_pose_error()[1] < 2e-3


def test_scramble_and_inverse_restore_every_piece():
    from rubiks_cube import RubiksCube, inverse_moves
    cube = RubiksCube()
    moves = "R U F' L2 D B R' U2 F D'".split()
    for move in moves:
        cube.turn(move)
    assert not cube.is_solved()
    assert cube.solved_pose_error()[0] > .01
    for move in inverse_moves(moves):
        cube.turn(move)
    assert cube.is_solved()
    assert cube.solved_pose_error()[0] < 2e-5
    assert cube.solved_pose_error()[1] < 2e-3
    assert not cube.data.warning.number.any()


def test_cross_axis_moves_do_not_commute():
    from rubiks_cube import RubiksCube
    a, b = RubiksCube(), RubiksCube()
    for move in ['R', 'U']:
        a.turn(move)
    for move in ['U', 'R']:
        b.turn(move)
    assert not np.array_equal(a.slots, b.slots)
    assert np.max(np.abs(a.data.xpos-a.model.body_pos)) > .01
    assert np.max(np.abs(a.data.xpos-b.data.xpos)) > .01


@pytest.mark.parametrize('move', ['', 'X', 'R3', 'RR', "U2'"])
def test_invalid_moves_leave_state_unchanged(move):
    from rubiks_cube import RubiksCube
    cube = RubiksCube()
    before = cube.data.qpos.copy()
    with pytest.raises(ValueError):
        cube.turn(move)
    np.testing.assert_array_equal(cube.data.qpos, before)
