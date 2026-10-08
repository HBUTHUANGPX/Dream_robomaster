"""Solver boundary checks independent of scramble reversal."""
import numpy as np
import pytest

from rubiks_cube import RubiksCube


def test_solved_cube_needs_no_moves():
    from cube_solver import facelets_from_cube, solve_facelets, SOLVED
    cube = RubiksCube()
    assert facelets_from_cube(cube) == SOLVED
    assert solve_facelets(SOLVED) == []


@pytest.mark.parametrize('face', list('URFDLB'))
def test_facelet_encoding_matches_independent_kociemba_cubie_model(face):
    from cube_solver import facelets_from_cube
    from kociemba.pykociemba.cubiecube import CubieCube, moveCube
    cube = RubiksCube()
    cube.turn(face)
    reference = CubieCube()
    reference.multiply(moveCube['URFDLB'.index(face)])
    assert facelets_from_cube(cube) == reference.toFaceCube().to_String()


def test_solver_restores_scrambled_cube_without_history():
    from cube_solver import facelets_from_cube, solve_facelets, SOLVED
    cube = RubiksCube()
    for move in "R U F' L2 D B R' U2 F D'".split():
        cube.turn(move)
    cube.history.clear()
    solution = solve_facelets(facelets_from_cube(cube))
    assert solution
    for move in solution:
        cube.turn(move)
    assert facelets_from_cube(cube) == SOLVED
    assert cube.solved_pose_error()[0] < 2e-5
    assert cube.solved_pose_error()[1] < .002


def test_facelets_use_physical_pose_not_stale_logical_arrays():
    from cube_solver import facelets_from_cube, SOLVED
    cube = RubiksCube()
    cube.turn('R')
    cube.slots[:] = cube.initial_slots
    cube.orientations[:] = np.eye(3, dtype=int)
    assert facelets_from_cube(cube) != SOLVED


@pytest.mark.parametrize('state', ['', 'U'*54, 'X'*54,
    'UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBB'])
def test_reject_invalid_facelets(state):
    from cube_solver import solve_facelets
    with pytest.raises(ValueError):
        solve_facelets(state)


def test_reject_physically_impossible_single_flipped_edge():
    from cube_solver import solve_facelets, SOLVED
    state = list(SOLVED)
    state[5], state[10] = state[10], state[5]  # flip just the UR edge
    with pytest.raises(ValueError):
        solve_facelets(''.join(state))
