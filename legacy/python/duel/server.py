"""Local browser UI; simulation and renderer have one owning thread."""
import argparse
import io
import ipaddress
import json
import os
os.environ.setdefault('MUJOCO_GL','egl')
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import queue
import threading
import time
from urllib.parse import urlsplit
import cv2
import mujoco
import numpy as np
from PIL import Image
from duel.model import ROOT
from duel.task import Duel,FRAME_DT


def local_request_allowed(host,origin):
    """SSH forwards preserve the browser's Host, including its local port."""
    try:
        address=urlsplit('//'+(host or ''))
        if not address.hostname or address.username is not None or address.password is not None:
            return False
        if address.path or address.query or address.fragment:
            return False
        if address.port is not None and not 1<=address.port<=65535:
            return False
        if address.hostname!='localhost' and not ipaddress.ip_address(address.hostname).is_loopback:
            return False
        if origin is not None:
            source=urlsplit(origin)
            if source.scheme not in ('http','https') or source.netloc.lower()!=host.lower():
                return False
            if source.path or source.query or source.fragment:
                return False
        return True
    except ValueError:
        return False


class Application:
    def __init__(self):
        self.game=Duel()
        self.lock=threading.Lock();self.commands=queue.Queue(maxsize=64)
        self.latest=self.game.state();self.frames={};self.error=None;self.running=True

    def request(self,name,args):
        if not self.running:raise RuntimeError(self.error or '仿真已停止')
        response=queue.Queue(maxsize=1)
        self.commands.put_nowait((name,args,response,time.monotonic()))
        return response.get(timeout=5)

    def process_commands(self):
        for _ in range(64):
            try:name,args,response,created=self.commands.get_nowait()
            except queue.Empty:break
            try:
                expiry=.35 if name=='drive' else 4.
                if time.monotonic()-created>expiry:raise ValueError('命令已过期，请重试')
                result=self.game.command(name,args)
                if name=='drive':self.game.drive_deadline=created+.35
                response.put((True,result))
            except (ValueError,TypeError,KeyError,OverflowError) as error:
                response.put((False,str(error)))

    def run(self):
        try:
            with mujoco.Renderer(self.game.model,height=600,width=800) as renderer:
                camera=mujoco.MjvCamera();camera.distance=7.7;camera.elevation=-48;camera.azimuth=90
                camera.lookat[:]=[0,0,.1]
                while self.running:
                    begin=time.monotonic()
                    self.process_commands()
                    self.game.step(renderer)
                    if self.game.settings['view']=='follow':
                        camera.lookat[:]=self.game.data.xpos[self.game.chassis[0]]+[0,0,.15]
                        rotation=self.game.data.xmat[self.game.chassis[0]].reshape(3,3)
                        camera.azimuth=np.rad2deg(np.arctan2(rotation[1,0],rotation[0,0]))+180
                        camera.distance=2.8;camera.elevation=-25
                    else:
                        camera.lookat[:]=[0,0,.1]
                        camera.distance=7.7;camera.elevation=-48;camera.azimuth=90
                    renderer.update_scene(self.game.data,camera=camera)
                    self.game.draw_projectiles(renderer)
                    frames={'scene':renderer.render().copy()}
                    detection=self.game.robots[0].detection
                    frames['number']=(detection.number_roi if detection is not None and
                                      not self.game.robots[0].heat.locked else np.zeros((28,20),np.uint8))
                    for key,frame in [('camera',self.game.annotated[0]),('raw',self.game.raw[0])]:
                        if frame is not None:
                            if self.game.robots[0].heat.locked:
                                frame=np.zeros_like(frame)
                                cv2.putText(frame,'OVERHEAT - VIDEO LOCKED',(150,300),cv2.FONT_HERSHEY_SIMPLEX,1,(255,160,80),2)
                            frames[key]=frame
                    encoded={}
                    for key,frame in frames.items():
                        buf=io.BytesIO();Image.fromarray(frame).save(buf,format='JPEG',quality=88)
                        encoded[key]=buf.getvalue()
                    state=self.game.state()
                    state['compute_ms']=round((time.monotonic()-begin)*1000,1)
                    with self.lock:self.frames=encoded;self.latest=state
                    time.sleep(max(0,FRAME_DT-(time.monotonic()-begin)))
        except Exception as error:
            import traceback
            traceback.print_exc()
            with self.lock:self.error=str(error)
            self.running=False


def make_server(app,port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass

        def send(self,data,mime='application/json; charset=utf-8',status=200):
            if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass

        def do_GET(self):
            path=urlsplit(self.path).path
            if path=='/':self.send((ROOT/'web/duel/index.html').read_bytes(),'text/html; charset=utf-8')
            elif path=='/app.js':self.send((ROOT/'web/duel/app.js').read_bytes(),'text/javascript; charset=utf-8')
            elif path=='/api/state':
                with app.lock:state={**app.latest,'error':app.error}
                self.send(state)
            elif path in ('/api/scene.jpg','/api/camera.jpg','/api/raw.jpg','/api/number.jpg'):
                key=path.split('/')[-1].split('.')[0]
                with app.lock:frame=app.frames.get(key,b'')
                self.send(frame,'image/jpeg',200 if frame else 503)
            else:self.send({'error':'not found'},status=404)

        def do_POST(self):
            host=self.headers.get('Host')
            if not local_request_allowed(host,None):
                self.send({'error':'Host rejected'},status=403);return
            origin=self.headers.get('Origin')
            if not local_request_allowed(host,origin):
                self.send({'error':'Origin rejected'},status=403);return
            if self.headers.get('Content-Type','').split(';')[0]!='application/json':
                self.send({'error':'JSON required'},status=415);return
            name=urlsplit(self.path).path.removeprefix('/api/')
            if name not in ('settings','drive','aim','fire','pause','reset'):
                self.send({'error':'not found'},status=404);return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=4096:raise ValueError('请求大小错误')
                def reject_constant(value):raise ValueError('禁止非有限数值')
                args=json.loads(self.rfile.read(length),parse_constant=reject_constant)
                if not isinstance(args,dict):raise ValueError('参数必须是 JSON 对象')
                result=app.request(name,args)
                if isinstance(result,tuple):
                    ok,result=result
                    if not ok:raise ValueError(result)
                self.send(result)
            except (ValueError,TypeError,KeyError,OverflowError) as error:
                self.send({'error':str(error)},status=400)
            except (queue.Empty,queue.Full,RuntimeError) as error:
                self.send({'error':str(error) or '仿真繁忙，请重试'},status=503)
    return ThreadingHTTPServer(('127.0.0.1',port),Handler)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',type=int,default=8766)
    args=parser.parse_args()
    app=Application();server=make_server(app,args.port)
    worker=threading.Thread(target=app.run,daemon=True);worker.start()
    print(f'1v1 图像自瞄 http://127.0.0.1:{server.server_port}',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        app.running=False;server.server_close();worker.join(timeout=5)


if __name__=='__main__':main()
