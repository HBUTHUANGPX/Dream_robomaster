"""Exercise original ONNX weights; no mocked logits or scene-ID shortcut."""
from pathlib import Path
import cv2
import numpy as np


def test_upstream_texture_classifies_as_three_and_blank_is_rejected():
    from duel.classifier import NumberClassifier
    image=cv2.imread(str(Path(__file__).parents[1]/'assets/armor_labels/3.png'))
    gray=cv2.cvtColor(cv2.resize(image,(20,28)),cv2.COLOR_BGR2GRAY)
    classifier=NumberClassifier()
    label,confidence,roi=classifier.classify_roi(gray)
    assert label=='3' and confidence>.8
    label,confidence,_=classifier.classify_roi(np.zeros((28,20),np.uint8))
    assert label is None


def test_actual_render_has_classifiable_number():
    import os
    os.environ.setdefault('MUJOCO_GL','egl')
    import mujoco
    from duel.task import Duel
    game=Duel()
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        game.perceive(renderer,0)
    d=game.robots[0].detection
    assert d is not None
    assert d.number=='3' and d.confidence>=.8
    assert d.number_roi.shape==(28,20)


def test_blank_label_cannot_initialize_tracker():
    import os
    os.environ.setdefault('MUJOCO_GL','egl')
    import mujoco
    from duel.task import Duel
    game=Duel()
    game.model.tex_data[:]=0
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        for _ in range(10):game.step(renderer)
    assert game.robots[0].tracker.x is None


def test_all_eight_labels_are_upright_from_outside():
    import mujoco
    from duel.task import Duel
    game=Duel()
    source=cv2.imread(str(Path(__file__).parents[1]/'assets/armor_labels/3.png'),cv2.IMREAD_GRAYSCALE)
    reference=cv2.resize(source,(160,160))>127
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        for team in ('blue','red'):
            for side in ('front','left','rear','right'):
                # Test camera placement only; no truth enters perception.
                body=game.data.body(f'{team}_armor_{side}');rotation=body.xmat.reshape(3,3)
                normal=rotation[:,0];origin=body.xpos+rotation @ np.array([.0105,0,0])
                camera=game.cameras[0];game.data.cam_xpos[camera]=origin+normal*.5
                right=np.cross(-normal,[0,0,1]);right/=np.linalg.norm(right)
                game.data.cam_xmat[camera]=np.column_stack((right,np.cross(normal,right),normal)).ravel()
                renderer.update_scene(game.data,camera=camera)
                crop=cv2.cvtColor(renderer.render()[220:380,320:480],cv2.COLOR_RGB2GRAY)>127
                correct=np.mean(crop==reference)
                assert correct>.9
                assert correct>np.mean(crop==reference[::-1,:])+.015
                assert correct>np.mean(crop==reference[:,::-1])+.1
