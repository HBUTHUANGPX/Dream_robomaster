"""Record raw LiDAR/IMU for upstream FAST-LIO snapshot input, without ROS."""
from pathlib import Path
import argparse,json,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mujoco
import numpy as np
from navigation.task import NavigationTask
from navigation.rosbag_export import RawBag
from navigation.terrain import ROOT


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=ROOT/'output/lio_input.bag')
    parser.add_argument('--case',type=int,default=6)
    parser.add_argument('--seconds',type=float,default=100.)
    args=parser.parse_args()
    task=NavigationTask();bag=RawBag(args.output)
    ground_truth=[];goal=task.presets()[args.case]
    try:
        # Two seconds at rest initialize gravity and gyro bias before wheel motion.
        mujoco.mj_forward(task.model,task.data)
        bag.imu(task.sensors.imu(task.data))
        bag.cloud(task.sensors.raw_clouds[0])
        for i in range(int((args.seconds+2)*10)):
            if i==20:task.set_goal(goal['x'],goal['y'])
            task.step(imu_callback=bag.imu)
            bag.cloud(task.sensors.raw_clouds[0])
            # Separate file: truth never enters the bag consumed by the estimator.
            site=task.model.site('imu').id
            ground_truth.append([float(task.data.time),*task.data.site_xpos[site].tolist(),*task.data.sensor('attitude').data.tolist()])
            if i%100==0:print(i/10,task.status,task.mapper.pose.round(3),flush=True)
            if i>=20 and (task.goal is None or task.paused):break
        # End with more IMU messages to release the estimator's final LiDAR packet.
        for _ in range(5):
            mujoco.mj_step(task.model,task.data)
        mujoco.mj_forward(task.model,task.data)
        bag.imu(task.sensors.imu(task.data))
    finally:bag.close()
    metadata=dict(status=task.status,goal=goal,counts=bag.count,imu_hz=200,lidar_hz=10,
        scan_model='instantaneous snapshot; all points share scan timestamp; FAST-LIO lidar_type=4',
        sensors='top LiDAR only; ideal raw accelerometer and gyro; no AHRS input',
        extrinsic_T=task.sensors.raw_clouds[0]['translation'].tolist(),
        extrinsic_R=task.sensors.raw_clouds[0]['rotation'].tolist(),
        truth_columns=['stamp','imu_x','imu_y','imu_z','qw','qx','qy','qz'],ground_truth=ground_truth)
    args.output.with_suffix('.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
    print(args.output,bag.count,task.status,flush=True)
    if task.status!='已到达':raise SystemExit('Recording saved, but navigation did not reach the goal')


if __name__=='__main__':main()
