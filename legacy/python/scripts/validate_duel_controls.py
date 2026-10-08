"""Five-second invulnerable runs at the new default and maximum speeds."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import collections,json,time
import mujoco
from duel.task import Duel


def main():
    report={};game=Duel()
    with mujoco.Renderer(game.model,height=600,width=800) as renderer:
        for motion,speed,spin in [('static',2,2),('spin',2,2),('spin',2,4),('strafe',2,2),('strafe',4,2),('circle',2,2)]:
            game.command('settings',{'enemy_motion':motion,'target_speed':speed,'target_spin':spin,'auto_fire':True,'invulnerable':True,'profile':'burst'})
            game.reset();first=None;reasons=collections.Counter();ready=0;start=time.monotonic()
            for frame in range(100):
                game.step(renderer);robot=game.robots[0]
                reasons[robot.fire_reason]+=1;ready+=robot.tracker.ready(game.time)
                if first is None and robot.shots:first=round(game.time,3)
            key=f'{motion}-{speed}-{spin}'
            report[key]={'first_shot_s':first,'hits':robot.hits,'shots':robot.shots,'ready_fraction':ready/100,
                         'reasons':dict(reasons),'warnings':game.data.warning.number.tolist(),'wall_seconds':time.monotonic()-start}
            print(key,report[key],flush=True)
    Path('output/duel').mkdir(parents=True,exist_ok=True)
    Path('output/duel/control-validation.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))

if __name__=='__main__':main()
