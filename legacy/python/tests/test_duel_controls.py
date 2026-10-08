import os
os.environ.setdefault('MUJOCO_GL','egl')
import numpy as np
import pytest
from duel.task import Duel


def test_speed_defaults_limits_and_revision():
    g=Duel()
    assert g.settings['target_speed']==2
    assert g.settings['target_spin']==2
    revision=g.state()['revision']
    reply=g.command('settings',{'target_speed':4,'target_spin':4})
    assert reply['revision']>revision
    for key in ('target_speed','target_spin'):
        for value in (-.1,4.1,float('nan')):
            with pytest.raises(ValueError):g.command('settings',{key:value})


def test_pause_is_idempotent_and_blocks_motion_commands():
    g=Duel()
    g.command('pause',{'paused':True});g.command('pause',{'paused':True})
    assert g.paused
    g.command('drive',{'vx':.5})
    assert not g.drive.any()
    g.command('pause',{'paused':False})
    assert not g.paused


def test_invulnerable_preserves_hits_and_hp():
    from duel.physics import Projectile
    g=Duel();assert g.settings['invulnerable']
    for enabled in (True,False):
        g.command('settings',{'invulnerable':enabled})
        face=g.data.site('red_armor_face_front').xpos.copy()
        normal=g.data.site('red_armor_face_front').xmat.reshape(3,3)[:,0]
        g.projectiles=[Projectile(0,face+normal*.1,-normal*23,0)]
        g.robots[1].last_damage.clear()
        for _ in range(8):g.advance_projectiles(.001,g.data.geom_xpos.copy(),g.data.geom_xmat.copy())
        assert g.robots[1].hp==(200 if enabled else 180)
    assert g.robots[0].hits==2
    assert not g.winner


def test_auto_fire_has_prompt_first_shot_and_explains_cooling():
    import mujoco
    g=Duel();g.command('settings',{'auto_fire':True})
    first=None
    with mujoco.Renderer(g.model,height=600,width=800) as renderer:
        for _ in range(30):
            g.step(renderer)
            if first is None and g.robots[0].shots:first=g.time
    assert first is not None and first<.7
    assert g.state()['robots'][0]['fire_reason']


def test_reenable_auto_search_starts_from_manual_view():
    g=Duel();g.command('settings',{'auto_aim':False})
    g.data.qpos[g.joints[0]]=[1.,.1]
    g.command('settings',{'auto_aim':True})
    g.control(0)
    assert np.allclose(g.robots[0].angles,[1.,.1],atol=.001)
