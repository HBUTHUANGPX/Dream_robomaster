"""Local navigation UI, serialized simulation loop, safe HTTP command queue."""
import argparse
import io
import json
import os
os.environ.setdefault('MUJOCO_GL','egl')
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import queue
import threading
import time
from urllib.parse import urlsplit
import mujoco
import numpy as np
from PIL import Image
from navigation.task import NavigationTask
from navigation.terrain import ROOT,RES


def allowed_origin(origin):
    """Permit browser loopback origins across local port-forwarding ports."""
    if origin is None:return True  # Non-browser clients may omit Origin.
    try:
        parsed=urlsplit(origin)
        port=parsed.port
        return (parsed.scheme in ('http','https')
                and parsed.hostname in ('localhost','127.0.0.1','::1')
                and parsed.username is None and parsed.password is None
                and not(parsed.path or parsed.query or parsed.fragment)
                and (port is None or 1<=port<=65535))
    except ValueError:
        return False

class Application:
    def __init__(self,localization='prior',power_budget=120.):
        self.task=NavigationTask(localization,power_budget);self.commands=queue.Queue();self.lock=threading.Lock()
        self.latest=self.task.state();self.frame=b'';self.error=None;self.running=True
        t=self.task.terrain
        self.map={'localization':localization,'width':t.width,'height':t.height,'resolution':RES,'cols':t.cols,'rows':t.rows,'heights':np.nan_to_num(t.heights,nan=-1).ravel().tolist(),'reachable':t.reachable.astype(int).ravel().tolist(),'presets':self.task.presets(),'spawn':t.spawn.tolist()}

    def run(self):
        try:
            with mujoco.Renderer(self.task.model,height=600,width=960) as renderer:
                cam=mujoco.MjvCamera();cam.distance=2.4;cam.elevation=-38;cam.azimuth=135
                opt=mujoco.MjvOption();opt.geomgroup[3]=0
                count=0
                while self.running:
                    start=time.monotonic()
                    while not self.commands.empty():
                        name,args,result=self.commands.get()
                        try:
                            if name=='goal':self.task.set_goal(float(args['x']),float(args['y']))
                            elif name=='pause':self.task.paused=not self.task.paused
                            elif name=='reset':self.task.reset()
                            elif name=='yaw':self.task.set_yaw_rate(float(args['rate']))
                            result.put({'ok':True,'message':self.task.status})
                        except (ValueError,KeyError,TypeError) as e:result.put({'ok':False,'message':str(e)})
                    self.task.step()
                    if count%3==0:
                        cam.lookat[:]=self.task.mapper.pose[:3]+[0,0,.1]
                        renderer.update_scene(self.task.data,camera=cam,scene_option=opt)
                        for path,color,width in [(self.task.path,[.1,.85,1,1],.018),(self.task.local,[1,.5,.1,1],.032)]:
                            for a,b in zip(path[::2][:-1],path[::2][1:]):
                                if renderer.scene.ngeom>=renderer.scene.maxgeom:break
                                g=renderer.scene.geoms[renderer.scene.ngeom]
                                mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_CAPSULE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array(color,dtype=np.float32))
                                mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_CAPSULE,width,a+[0,0,.06],b+[0,0,.06]);renderer.scene.ngeom+=1
                        buf=io.BytesIO();Image.fromarray(renderer.render()).save(buf,format='JPEG',quality=85)
                        with self.lock:self.frame=buf.getvalue()
                    with self.lock:self.latest=self.task.state()
                    count+=1
                    time.sleep(max(0,.1-(time.monotonic()-start)))
        except Exception as e:
            import traceback
            traceback.print_exc();self.error=str(e);self.running=False

    def request(self,name,args):
        if not self.running:raise RuntimeError(self.error or 'Simulation stopped')
        response=queue.Queue(maxsize=1);self.commands.put((name,args,response))
        return response.get(timeout=10)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8765);parser.add_argument('--localization',choices=['prior','slam'],default='prior');parser.add_argument('--power-budget',type=float,default=120.);args=parser.parse_args()
    app=Application(args.localization,args.power_budget)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,data,mime='application/json',status=200):
            if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def do_GET(self):
            p=urlsplit(self.path).path
            if p=='/':
                html=(ROOT/'web/navigation/index.html').read_text()
                initial='<script>window.NAV_INITIAL_MAP='+json.dumps(app.map,allow_nan=False)+'</script>'
                self.send(html.replace('<script>',initial+'<script>',1).encode(),'text/html; charset=utf-8')
            elif p=='/api/map':self.send(app.map)
            elif p=='/api/state':
                with app.lock:state=app.latest.copy()
                if app.error:state['status']='仿真异常：'+app.error
                self.send(state)
            elif p=='/api/frame.jpg':
                with app.lock:frame=app.frame
                self.send(frame,'image/jpeg',200 if frame else 503)
            else:self.send({'error':'not found'},status=404)
        def do_POST(self):
            # Local-only service; reject cross-origin form submissions.
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':
                self.send({'ok':False,'message':'JSON required'},status=415);return
            origin=self.headers.get('Origin')
            if not allowed_origin(origin):
                self.send({'ok':False,'message':'origin rejected'},status=403);return
            name=urlsplit(self.path).path.removeprefix('/api/')
            if name not in ['goal','pause','reset','yaw']:
                self.send({'ok':False,'message':'not found'},status=404);return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<=length<=4096:raise ValueError('Request too large')
                data=json.loads(self.rfile.read(length) or b'{}')
                self.send(app.request(name,data))
            except (ValueError,TypeError,queue.Empty,RuntimeError) as e:self.send({'ok':False,'message':str(e)},status=400)
    threading.Thread(target=app.run,daemon=True).start()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'导航界面 http://127.0.0.1:{args.port}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:app.running=False;server.server_close()

if __name__=='__main__':main()
