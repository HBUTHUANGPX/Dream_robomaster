"""Optional ROS1 recording. Imports rosbags only when explicitly requested."""
from pathlib import Path
import numpy as np


class RawBag:
    def __init__(self,path):
        from rosbags.rosbag1 import Writer
        from rosbags.typesys import Stores,get_typestore
        self.store=get_typestore(Stores.ROS1_NOETIC)
        self.types=self.store.types
        self.writer=Writer(Path(path));self.writer.open()
        self.lidar=self.writer.add_connection('/sim/lidar','sensor_msgs/msg/PointCloud2',typestore=self.store)
        self.imu_connection=self.writer.add_connection('/sim/imu','sensor_msgs/msg/Imu',typestore=self.store)
        self.seq={'lidar':0,'imu':0};self.count={'lidar':0,'imu':0}

    def header(self,stamp,kind,frame):
        ns=round(stamp*1e9)
        time=self.types['builtin_interfaces/msg/Time'](ns//10**9,ns%10**9)
        header=self.types['std_msgs/msg/Header'](self.seq[kind],time,frame)
        self.seq[kind]+=1
        return header,ns

    def cloud(self,packet):
        header,ns=self.header(packet['stamp'],'lidar','mid360_top')
        points=np.c_[packet['points'],np.ones(len(packet['points']))].astype('<f4')
        field=self.types['sensor_msgs/msg/PointField']
        fields=[field(name,i*4,7,1) for i,name in enumerate(['x','y','z','intensity'])]
        msg=self.types['sensor_msgs/msg/PointCloud2'](header,1,len(points),fields,False,16,16*len(points),points.view(np.uint8).ravel(),True)
        self.writer.write(self.lidar,ns,self.store.serialize_ros1(msg,msg.__msgtype__))
        self.count['lidar']+=1

    def imu(self,packet):
        header,ns=self.header(packet['stamp'],'imu','imu')
        vector=self.types['geometry_msgs/msg/Vector3'];quat=self.types['geometry_msgs/msg/Quaternion']
        unknown=np.zeros(9);unknown[0]=-1.
        # Ideal raw gyro/accelerometer; NO orientation or position estimate supplied.
        msg=self.types['sensor_msgs/msg/Imu'](header,quat(0.,0.,0.,1.),unknown,
            vector(*map(float,packet['angular_velocity'])),np.zeros(9),
            vector(*map(float,packet['acceleration'])),np.zeros(9))
        self.writer.write(self.imu_connection,ns,self.store.serialize_ros1(msg,msg.__msgtype__))
        self.count['imu']+=1

    def close(self):self.writer.close()
