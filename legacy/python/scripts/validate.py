"""Save reproducible physical-motion measurements to output/validation.json."""
from pathlib import Path
import json
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulate import ROOT, load, advance, yaw

results=[]
for name,velocity in [('rest',(0,0,0)),('forward',(.4,0,0)),('backward',(-.4,0,0)),('left',(0,.4,0)),('right',(0,-.4,0)),('ccw',(0,0,.7)),('cw',(0,0,-.7))]:
    model,data=load()
    advance(model,data,(0,0,0),.5)
    origin=data.qpos[:3].copy()
    initial_yaw=yaw(data)
    advance(model,data,velocity,2)
    results.append({'case':name,'command':velocity,'duration_s':2,'displacement_m':(data.qpos[:3]-origin).tolist(),'yaw_change_rad':yaw(data)-initial_yaw,'warnings':int(data.warning.number.sum())})
report={'mass_kg':float(model.body_mass.sum()),'nq':model.nq,'nv':model.nv,'actuators':model.nu,'results':results}
(ROOT/'output').mkdir(exist_ok=True)
(ROOT/'output/validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
