import numpy as np
import pytest
pytest.importorskip('rosbags')
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore
from navigation.rosbag_export import RawBag


def test_raw_bag_roundtrip_keeps_timestamps_units_and_no_pose(tmp_path):
    path=tmp_path/'sensor.bag';bag=RawBag(path)
    bag.imu(dict(stamp=1.005,angular_velocity=np.array([.1,.2,.3]),acceleration=np.array([0,0,9.81])))
    bag.cloud(dict(stamp=1.1,points=np.array([[1.,2.,3.],[4.,5.,6.]])))
    bag.close()
    store=get_typestore(Stores.ROS1_NOETIC)
    with Reader(path) as reader:
        assert {c.topic for c in reader.connections}=={'/sim/imu','/sim/lidar'}
        messages=[(c.topic,t,store.deserialize_ros1(raw,c.msgtype)) for c,t,raw in reader.messages()]
    _,stamp,imu=messages[0]
    assert stamp==1005000000
    assert imu.orientation_covariance[0]==-1
    assert imu.linear_acceleration.z==9.81 and imu.angular_velocity.x==.1
    _,stamp,cloud=messages[1]
    assert stamp==1100000000
    assert cloud.header.frame_id=='mid360_top'
    assert [f.name for f in cloud.fields]==['x','y','z','intensity']
    assert cloud.width==2 and cloud.point_step==16
    np.testing.assert_array_equal(cloud.data.view('<f4').reshape(-1,4)[:,:3],[[1,2,3],[4,5,6]])
