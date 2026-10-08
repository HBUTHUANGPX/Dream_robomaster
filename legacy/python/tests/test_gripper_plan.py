"""Verify fixed-rule action compilation across all cube orientations."""
import numpy as np
import pytest

from rubiks_cube import FACES


def orientations():
    from itertools import permutations, product
    for perm in permutations(range(3)):
        for signs in product([-1, 1], repeat=3):
            matrix = np.eye(3, dtype=int)[:, perm]*signs
            if round(np.linalg.det(matrix)) == 1:
                yield matrix


@pytest.mark.parametrize('face', list('URFDLB'))
def test_each_face_is_presented_to_a_from_all_24_orientations(face):
    from gripper_plan import compile_moves, replay_plan
    for initial in orientations():
        plan = compile_moves([face+"'"], initial)
        result = replay_plan(plan, initial)
        assert result['moves'] == [face+"'"]
        assert result['grasped'] == ['A', 'B']
        assert result['yaw_rad'] == {'A': 0., 'B': 0.}
        axis, sign = FACES[face]
        np.testing.assert_array_equal(np.array(result['orientation'])@np.eye(3, dtype=int)[axis]*sign,
                                      [1, 0, 0])


def test_plan_rejects_unsupported_rotation_and_dropping_cube():
    from gripper_plan import Action, replay_plan
    with pytest.raises(ValueError, match='support'):
        replay_plan([Action('release', 'A'), Action('release', 'B')])
    with pytest.raises(ValueError, match='clear'):
        replay_plan([Action('yaw', 'A', 1.5707963267948966, mode='whole')])


def test_long_sequence_preserves_exact_symbolic_moves():
    from gripper_plan import compile_moves, replay_plan
    moves = "U R2 F' D B2 L U' R F2 D' L2 B".split()
    plan = compile_moves(moves)
    result = replay_plan(plan)
    assert result['moves'] == moves
    assert len(plan) > len(moves)
    assert all(action.as_dict()['kind'] for action in plan)


def test_empty_solution_and_invalid_orientation():
    from gripper_plan import compile_moves
    assert compile_moves([]) == []
    with pytest.raises(ValueError):
        compile_moves(['R'], -np.eye(3, dtype=int))


def test_non_quarter_reorientation_is_not_silently_rounded():
    from gripper_plan import wrist_rotation
    with pytest.raises(ValueError, match='quarter'):
        wrist_rotation('A', .1)
