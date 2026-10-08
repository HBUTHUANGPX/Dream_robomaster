"""Convert official STEP files to metre OBJ meshes; runtime needs no CAD library."""
from pathlib import Path
import json
import math
import cadquery as cq
import numpy as np

ROOT = Path(__file__).resolve().parents[1]

def write_obj(path, vertices, faces):
    with path.open('w') as f:
        for v in vertices:
            f.write('v ' + ' '.join(f'{x:.9f}' for x in v) + '\n')
        for face in faces:
            f.write('f ' + ' '.join(str(i + 1) for i in face) + '\n')

def main():
    metadata = {}
    for name in ['armor_am02', 'armor_frame_a']:
        shape = cq.importers.importStep(str(ROOT / 'assets/official' / f'{name}.step')).val()
        vertices, faces = shape.tessellate(0.25, 0.15)
        v = np.array([p.toTuple() for p in vertices])
        if name == 'armor_am02':
            # Front is native +X; native +Y is up and -Z is left.
            v = (v - [-17.40697118333, 63.89617963739, -45.00439550208]) @ np.array([[1,0,0],[0,0,1],[0,-1,0]])
        else:
            # Native +Z is the mating face normal. Rotate so bottom feet are horizontal.
            s, c = math.sin(math.pi/12), math.cos(math.pi/12)
            v[:,1] -= 32.0
            v = v @ np.array([[0,1,0],[-s,0,c],[c,0,s]])
        v *= 0.001
        write_obj(ROOT / 'assets/meshes' / f'{name}.obj', v, faces)
        metadata[name] = {'min_m': v.min(axis=0).tolist(), 'max_m': v.max(axis=0).tolist(), 'triangles': len(faces)}
    (ROOT / 'assets/meshes/metadata.json').write_text(json.dumps(metadata,indent=2))
    print(json.dumps(metadata,indent=2))

if __name__ == '__main__':
    main()
