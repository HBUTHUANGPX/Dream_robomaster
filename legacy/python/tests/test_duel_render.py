"""Integration checks use actual MuJoCo RGB frames, never segmentation."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import mujoco


def test_real_render_detects_enemy_and_blank_stops_tracking():
    from duel.task import Duel
    game=Duel()
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        for _ in range(8):
            game.step(renderer)
        assert game.robots[0].tracker.ready(game.time)
        d=game.robots[0].detection
        assert d is not None
        cam=game.camera_pose(0)
        estimate=cam[0]+cam[1] @ d.tvec
        target=game.data.site('red_armor_face_front').xpos
        assert np.linalg.norm(estimate-target) < .35
        game.command('settings', {'vision':False,'auto_fire':True})
        shots=game.robots[0].shots
        for _ in range(12):
            game.step(renderer)
        assert not game.robots[0].tracker.ready(game.time)
        assert game.robots[0].shots <= shots+1
        assert not game.data.warning.number.any()


def test_stationary_closed_loop_hits_and_reset():
    from duel.task import Duel
    game=Duel()
    game.command('settings',{'auto_fire':True,'invulnerable':False})
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        for _ in range(70):
            game.step(renderer)
    assert game.robots[0].hits >= 1
    assert game.robots[1].hp < 200
    game.command('reset',{})
    assert game.robots[1].hp == 200 and not game.projectiles
    assert game.robots[0].shots == 0 and game.robots[0].tracker.x is None


def test_duel_wheels_move_laterally_without_self_collision():
    from duel.task import Duel
    game=Duel()
    game.command('settings',{'enemy_motion':'strafe','target_speed':.45})
    origin=game.data.xpos[game.chassis[1]].copy()
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        for _ in range(20):game.step(renderer)
    position=game.data.xpos[game.chassis[1]]
    assert position[1] < origin[1]-.25
    assert abs(position[0]-origin[0]) < .12


def test_turret_absorbs_projectile_without_armor_damage():
    from duel.task import Duel
    from duel.physics import Projectile
    game=Duel()
    pitch=game.data.body('red_pitch').xpos.copy()
    game.projectiles=[Projectile(0,pitch+[-.3,0,0],np.array([25.,0,0]),0)]
    for _ in range(20):
        game.advance_projectiles(.001,game.data.geom_xpos.copy(),game.data.geom_xmat.copy())
    assert not game.projectiles
    assert game.robots[1].hp==200


def test_rotating_robot_tracks_center_and_plate_changes_from_rgb():
    from duel.task import Duel
    from duel.rotor_tracking import RotorTracker
    game=Duel()
    assert isinstance(game.robots[0].tracker,RotorTracker)
    game.command('settings',{'enemy_motion':'spin','target_spin':.8})
    tracked=[];rates=[]
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        for _ in range(110):
            game.step(renderer)
            tr=game.robots[0].tracker
            tracked.append(tr.ready(game.time))
            if tr.ready(game.time):rates.append(tr.x[7])
    assert sum(tracked)/len(tracked)>.6
    assert tr.switches>=1
    assert abs(float(np.median(rates[-15:]))-.8)<.35
    truth=game.data.xpos[game.chassis[1]]
    assert np.linalg.norm(tr.center(game.time)[:2]-truth[:2])<.15
