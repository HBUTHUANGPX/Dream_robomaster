"""Run real closed-loop goals; record truth ONLY for independent evaluation."""
from pathlib import Path
import sys,json,time,argparse,hashlib
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial import cKDTree
from navigation.task import NavigationTask
from navigation.terrain import ROOT
p=argparse.ArgumentParser();p.add_argument('--cases',type=int,nargs='*',default=[0,1,2,3,4,5]);p.add_argument('--seconds',type=float,default=180);p.add_argument('--output',default='navigation_validation.json');p.add_argument('--localization',choices=['prior','slam'],default='prior');p.add_argument('--yaw-rate',type=float,default=0.);a=p.parse_args()
sources={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in ['navigation/task.py','navigation/control.py','navigation/dwa.py','navigation/sensors.py','navigation/slam.py','navigation/power.py']}
t=NavigationTask(a.localization);results=[]
for case in a.cases:
 t.reset();preset=t.presets()[case];t.set_goal(preset['x'],preset['y']);t.set_yaw_rate(a.yaw_rate)
 maxerr=0.;maxz=0.;peak_speed=0.;in_place=0;moving_spin=0;start=time.time();log=[];cross_track=[];step_wall=[];path_tree=cKDTree(t.path[:,:2])
 for i in range(int(a.seconds*10)):
  tick=time.perf_counter();t.step();step_wall.append(time.perf_counter()-tick);s=t.state();maxerr=max(maxerr,s['localization_error']);maxz=max(maxz,float(t.data.qpos[2]-.076));cross_track.append(float(path_tree.query(t.data.qpos[:2])[0]))
  peak_speed=max(peak_speed,float(np.linalg.norm(t.data.qvel[:2])))
  in_place+=int(np.linalg.norm(t.velocity[:2])<.05 and abs(t.velocity[2])>.1)
  moving_spin+=int(np.linalg.norm(t.velocity[:2])>.15 and abs(t.velocity[2])>.1)
  if i%100==0:print(preset['name'],i/10,t.mapper.pose.round(3),t.data.qpos[:3].round(3),t.status,flush=True)
  if i%10==0:log.append([float(t.data.time),*t.data.qpos[:3].tolist(),*t.mapper.pose.tolist()])
  if t.goal is None or t.paused:break
 result={'goal':preset,'status':t.status,'sim_seconds':i/10,'wall_seconds':time.time()-start,'truth_error_xy':float(np.linalg.norm(t.data.qpos[:2]-[preset['x'],preset['y']])),'max_localization_error_m':maxerr,'max_ground_height_m':maxz,'warnings':t.data.warning.number.tolist(),'trajectory':log,'controller':t.state()['controller'],'peak_truth_speed_m_s':peak_speed,'power_peak_w':t.power_peak_w,'power_budget_w':t.drive.budget,'energy_j':t.energy_j,'in_place_turn_seconds':in_place*.1,'moving_spin_seconds':moving_spin*.1,'source_sha256':sources,'cross_track_rmse_m':float(np.sqrt(np.mean(np.square(cross_track)))),'step_wall_p50_ms':float(np.percentile(step_wall,50)*1000),'step_wall_p95_ms':float(np.percentile(step_wall,95)*1000)}
 results.append(result);print({k:v for k,v in result.items() if k!='trajectory'},flush=True)
 (ROOT/'output'/a.output).write_text(json.dumps(results,ensure_ascii=False,indent=2))

if any(r['status']!='已到达' or r['truth_error_xy']>.20 or any(r['warnings']) or r['power_peak_w']>r['power_budget_w']+1e-6 for r in results):
 raise SystemExit('Navigation validation failed; see saved results')
