"""Real-render deterministic acceptance scenarios; saves evidence in output/duel."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse
import json
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from PIL import Image
from duel.task import Duel


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=8)
    args=parser.parse_args()
    out=Path('output/duel');out.mkdir(parents=True,exist_ok=True)
    game=Duel();report={}
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        for motion in ('static','strafe','circle','spin','mutual'):
            game.command('settings',{'enemy_motion':motion if motion!='mutual' else 'strafe','auto_fire':True,'invulnerable':False,'target_speed':.45,'target_spin':.8,'enemy_fire':motion=='mutual','profile':'burst'})
            game.reset();tracked=0;frames=0;errors=[];center_errors=[];classified=0;begin=time.monotonic()
            for frame in range(round(args.seconds/.05)):
                game.step(renderer);frames+=1
                robot=game.robots[0];tracked+=robot.tracker.ready(game.time)
                classified+=robot.detection is not None
                if robot.tracker.ready(game.time):
                    prediction=robot.tracker.position(game.time)
                    # Referee-only evaluation. Never used as controller input.
                    truth=[game.data.site(f'red_armor_face_{side}').xpos for side in ('front','left','rear','right')]
                    errors.append(min(np.linalg.norm(prediction-p) for p in truth))
                    center_errors.append(np.linalg.norm(robot.tracker.center(game.time)[:2]-game.data.xpos[game.chassis[1]][:2]))
                if frame==25:
                    Image.fromarray(game.annotated[0]).save(out/f'{motion}-cv.png')
                    Image.fromarray(game.raw[0]).save(out/f'{motion}-raw.png')
                    if robot.detection is not None:
                        Image.fromarray(robot.detection.number_roi).save(out/f'{motion}-number.png')
                    camera=mujoco.MjvCamera();camera.lookat[:]=[0,0,.15];camera.distance=7;camera.azimuth=90;camera.elevation=-42
                    renderer.update_scene(game.data,camera=camera);game.draw_projectiles(renderer)
                    Image.fromarray(renderer.render()).save(out/f'{motion}-arena.png')
                if game.winner:break
            report[motion]={'state':game.state(),'tracked_fraction':tracked/frames,
                            'classified_fraction':classified/frames,
                            'center_xy_rmse_m':float(np.sqrt(np.mean(np.square(center_errors)))) if center_errors else None,
                            'position_rmse_m':float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                            'wall_seconds':time.monotonic()-begin,'warnings':game.data.warning.number.tolist()}
            print(motion,report[motion]['state']['robots'][0],flush=True)
    (out/'validation.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
