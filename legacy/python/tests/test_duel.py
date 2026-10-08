"""Tests catch incorrect heat rules, tunnelling, PnP and estimator gating."""
import numpy as np
import pytest


def test_heat_tick_lock_and_recovery():
    from duel.physics import Heat
    h = Heat('cooling')
    for _ in range(4):
        assert h.fire(0, protect=False)
    assert h.value == 40 and not h.locked
    assert not h.fire(0, protect=True)
    assert h.fire(0, protect=False) and h.locked
    assert not h.fire(0, protect=False)
    h.cool(.099)
    assert h.value == 50
    h.cool(.1)
    assert h.value == pytest.approx(48.8)
    h.cool(4.2)
    assert h.value == 0 and not h.locked


def test_swept_box_hits_between_endpoints():
    from duel.physics import segment_box
    assert segment_box(np.array([-1., 0, 0]), np.array([1., 0, 0]), np.array([.01,.07,.06])) == pytest.approx(.495)
    assert segment_box(np.array([-1., .1, 0]), np.array([1., .1, 0]), np.array([.01,.07,.06])) is None


def test_tracker_converges_rejects_outlier_and_expires():
    from duel.tracking import Tracker
    tr = Tracker()
    rng = np.random.default_rng(2)
    for i in range(90):
        t = i / 30
        p = np.array([.35*t, 0, 4.])
        tr.observe(p + rng.normal(0, .008, 3), np.zeros(3), np.eye(3), t)
    assert np.linalg.norm(tr.position(3.1) - [1.085, 0, 4]) < .06
    assert tr.ready(3.)
    tr.observe(np.array([5., 4., 2.]), np.zeros(3), np.eye(3), 3.)
    assert np.linalg.norm(tr.position(3.1) - [1.085, 0, 4]) < .1
    assert not tr.ready(3.6)


def test_pnp_from_colored_pixels_and_blank():
    import cv2
    from duel.vision import Detector
    detector = Detector(800, 600, 45, 'red')
    frame = np.zeros((600,800,3), np.uint8)
    # Known 2 m frontal plate; RGB image (not OpenCV BGR).
    cv2.rectangle(frame, (376,290), (379,309), (255,20,20), -1)
    cv2.rectangle(frame, (421,290), (424,309), (255,20,20), -1)
    ds = detector.detect(frame)
    assert ds and abs(ds[0].tvec[2] - 2.) < .2
    assert abs(ds[0].tvec[0]) < .02
    assert not detector.detect(np.zeros_like(frame))
    assert not Detector(800,600,45,'blue').detect(frame)


def test_intercept_meets_moving_target():
    from duel.physics import intercept
    pos = np.array([4., 0, -.2]); vel = np.array([0, .6, 0])
    direction, flight = intercept(pos, vel, 23., np.array([.3,0,0]))
    hit = (23*direction + [.3,0,0])*flight + np.array([0,0,-4.905])*flight**2
    assert np.linalg.norm(hit - (pos + vel*flight)) < .001


def test_rotating_camera_does_not_create_target_motion():
    from duel.tracking import Tracker
    tr=Tracker()
    target=np.array([.3,.05,4.])
    for i in range(60):
        angle=.003*i
        rotation=np.array([[np.cos(angle),0,np.sin(angle)],[0,1,0],[-np.sin(angle),0,np.cos(angle)]])
        camera=np.array([.002*i,0,0])
        tr.observe(rotation.T @ (target-camera),camera,rotation,i/30)
    assert np.linalg.norm(tr.position(2.1)-target) < .001
    assert np.linalg.norm(tr.x[3:]) < .001


def test_round_shapes_absorb_center_but_not_corner_misses():
    from duel.physics import segment_cylinder,segment_ellipsoid
    a=np.array([-1.,0,0]);b=np.array([1.,0,0])
    assert segment_cylinder(a,b,.1,.2)==pytest.approx(.45)
    assert segment_cylinder(a+[0,.11,0],b+[0,.11,0],.1,.2) is None
    assert segment_ellipsoid(a,b,np.array([.1,.2,.3]))==pytest.approx(.45)
    assert segment_ellipsoid(a+[0,.3,0],b+[0,.3,0],np.array([.1,.2,.3])) is None
