"""Reject invalid commands atomically and keep pause/reset semantics."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import pytest


@pytest.fixture(scope='module')
def game():
    from duel.task import Duel
    return Duel()


@pytest.mark.parametrize('payload',[{'speed':float('nan')},{'speed':26},
    {'auto_fire':'false'},{'enemy_motion':'teleport'},{'fire_rate':0},
    {'target_speed':4.1},{'profile':'invalid'}])
def test_reject_invalid_settings(game,payload):
    before=game.settings.copy()
    with pytest.raises(ValueError):game.command('settings',payload)
    assert game.settings==before


def test_pause_and_invalid_command(game):
    with pytest.raises(ValueError):game.command('settings',[])
    with pytest.raises(ValueError):game.command('drive',{'vx':float('inf')})
    game.command('pause',{})
    t=game.time
    game.step(None)
    assert game.time==t
    assert not game.command('fire',{})['fired']
    game.command('reset',{})
    assert not game.paused


def test_http_rejects_non_object_and_cross_origin(game):
    from duel.server import make_server
    import threading,urllib.request,urllib.error,json
    class App:
        error=None
        latest=game.state()
        frames={}
        lock=threading.Lock()
        def request(self,name,args):return game.command(name,args)
    server=make_server(App(),0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    url=f'http://127.0.0.1:{server.server_port}'
    try:
        for body,origin,expected in [('[]',None,400),('{}','http://evil.example',403),('{"speed":NaN}',None,400)]:
            headers={'Content-Type':'application/json'}
            if origin:headers['Origin']=origin
            req=urllib.request.Request(url+'/api/settings',data=body.encode(),headers=headers)
            with pytest.raises(urllib.error.HTTPError) as error:urllib.request.urlopen(req)
            assert error.value.code==expected
        with urllib.request.urlopen(url+'/api/state') as response:
            assert len(json.load(response)['robots'])==2
    finally:
        server.shutdown();server.server_close();thread.join()


@pytest.mark.parametrize('name,payload',[
    ('aim',{'yaw':[0],'pitch':[0]}),('aim',{'yaw':'0','pitch':0}),
    ('aim',{'yaw':True,'pitch':0}),('drive',{'vx':[0],'vy':[0],'wz':[0]}),
    ('drive',{'vx':False}),('drive',{'vx':None})])
def test_reject_non_scalar_controls_without_poisoning_state(game,name,payload):
    angles=game.robots[0].angles.copy();drive=game.drive.copy()
    with pytest.raises(ValueError):game.command(name,payload)
    assert (game.robots[0].angles==angles).all()
    assert (game.drive==drive).all()


def test_delayed_drive_command_expires_in_queue(game):
    from duel.server import Application
    import queue,time
    app=Application()
    reply=queue.Queue()
    app.commands.put(('drive',{'vx':.5},reply,time.monotonic()-.4))
    app.process_commands()
    assert not reply.get_nowait()[0]
    assert not app.game.drive.any()


@pytest.mark.parametrize('host,origin,expected',[
    ('localhost:54232','http://localhost:54232',200),
    ('127.0.0.1:54232','http://127.0.0.1:54232',200),
    ('[::1]:54232','http://[::1]:54232',200),
    ('LOCALHOST:54232','http://localhost:54232',200),
    ('localhost:54232','http://localhost:54233',403),
    ('localhost:54232','http://evil.example',403),
    ('evil.example:54232','http://evil.example:54232',403),
    ('localhost.evil.example:54232',None,403),
    ('localhost:99999',None,403),
    ('evil@localhost:54232',None,403),
])
def test_forwarded_loopback_port_and_origin(game,host,origin,expected):
    from duel.server import make_server
    import threading,urllib.request,urllib.error,json
    class App:
        def request(self,name,args):return game.command(name,args)
    server=make_server(App(),0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        headers={'Content-Type':'application/json','Host':host}
        if origin:headers['Origin']=origin
        req=urllib.request.Request(f'http://127.0.0.1:{server.server_port}/api/settings',
            data=b'{"auto_aim":true}',headers=headers)
        if expected==200:
            with urllib.request.urlopen(req) as response:
                assert response.status==200
                assert json.load(response)['ok']
            assert game.settings['auto_aim']
        else:
            with pytest.raises(urllib.error.HTTPError) as error:urllib.request.urlopen(req)
            assert error.value.code==expected
    finally:
        server.shutdown();server.server_close();thread.join()
