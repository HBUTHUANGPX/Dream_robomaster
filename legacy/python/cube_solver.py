"""Read aligned physical facelets and solve with muodov/kociemba."""
from collections import Counter

import numpy as np

from rubiks_cube import FACES, PITCH, parse_move

ORDER = 'URFDLB'
SOLVED = ''.join(face*9 for face in ORDER)
# Each face is viewed from outside; vectors point right and down on its diagram.
FACE_BASIS = {
    'U': ((1, 0, 0), (0, -1, 0)), 'R': ((0, 1, 0), (0, 0, -1)),
    'F': ((1, 0, 0), (0, 0, -1)), 'D': ((1, 0, 0), (0, 1, 0)),
    'L': ((0, -1, 0), (0, 0, -1)), 'B': ((-1, 0, 0), (0, 0, -1)),
}
NORMAL_TO_FACE = {tuple(np.eye(3, dtype=int)[axis]*sign): face
                  for face, (axis, sign) in FACES.items()}


def encode_facelets(initial_slots, slots, orientations):
    facelets = ['?']*54
    for index, face in enumerate(ORDER):
        facelets[index*9+4] = face
    for initial, slot, orientation in zip(initial_slots, slots, orientations, strict=True):
        for axis, sign in enumerate(initial):
            if not sign:
                continue
            original_normal = np.eye(3, dtype=int)[axis]*sign
            color = NORMAL_TO_FACE[tuple(original_normal)]
            face = NORMAL_TO_FACE[tuple(orientation@original_normal)]
            right, down = FACE_BASIS[face]
            col, row = int(np.dot(slot, right))+1, int(np.dot(slot, down))+1
            index = 9*ORDER.index(face)+3*row+col
            if not (0 <= row < 3 and 0 <= col < 3) or facelets[index] != '?':
                raise ValueError('Overlapping or off-grid facelets')
            facelets[index] = color
    state = ''.join(facelets)
    validate_facelets(state)
    return state


def facelets_from_cube(cube):
    """Measure poses in the core frame, independent of the move history/cache."""
    core = cube.model.body('core').id
    core_rotation = cube.data.xmat[core].reshape(3, 3)
    positions = (cube.data.xpos[cube.piece_ids]-cube.data.xpos[core])@core_rotation/PITCH
    rotations = core_rotation.T@cube.data.xmat[cube.piece_ids].reshape(-1, 3, 3)
    slots = np.rint(positions).astype(int)
    orientations = np.rint(rotations).astype(int)
    if np.max(np.abs(positions-slots)) > .02 or np.max(np.abs(rotations-orientations)) > .02:
        raise ValueError('Cube is not aligned; finish the current turn before reading facelets')
    return encode_facelets(cube.initial_slots, slots, orientations)


def validate_facelets(state):
    if not isinstance(state, str) or len(state) != 54 or Counter(state) != Counter(SOLVED):
        raise ValueError('Expected 54 URFDLB facelets, nine of each color')
    if ''.join(state[9*i+4] for i in range(6)) != ORDER:
        raise ValueError('Facelet centers must be ordered URFDLB')


def solve_facelets(state):
    validate_facelets(state)
    if state == SOLVED:
        return []
    import kociemba
    from kociemba.pykociemba.facecube import FaceCube
    from kociemba.pykociemba.cubiecube import moveCube
    moves = kociemba.solve(state).split()
    # Independently replay the returned solution before emitting robot commands.
    check = FaceCube(state).toCubieCube()
    for move in moves:
        face, count = parse_move(move)
        for _ in range(count % 4):
            check.multiply(moveCube[ORDER.index(face)])
    if check.toFaceCube().to_String() != SOLVED:
        raise RuntimeError('The solver returned a sequence that does not solve this state')
    return moves
