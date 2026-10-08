import mujoco
import numpy as np
from navigation.sensors import SensorSuite
from navigation.terrain import ROOT


def test_raw_packets_have_sensor_frame_coordinates_and_si_imu():
    model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/navigation.xml'))
    data=mujoco.MjData(model)
    for _ in range(500):mujoco.mj_step(model,data)
    mujoco.mj_forward(model,data)
    sensor=SensorSuite(model,rays=200)
    sensor.sample(data,.1)
    assert len(sensor.raw_clouds)==2
    for packet in sensor.raw_clouds:
        assert packet['stamp']==data.time
        assert packet['points'].shape[1]==3
        assert np.isfinite(packet['points']).all()
        radii=np.linalg.norm(packet['points'],axis=1)
        assert (radii>.15).all() and (radii<40.1).all()
    np.testing.assert_allclose(sensor.raw_clouds[0]['translation'],[0,0,.438],atol=1e-6)
    np.testing.assert_allclose(sensor.raw_clouds[1]['translation'],[-.11,0,.332],atol=1e-6)
    packet=sensor.imu(data)
    assert packet['stamp']==data.time
    np.testing.assert_allclose(packet['acceleration'],[0,0,9.81],atol=.03)
    assert np.linalg.norm(packet['angular_velocity'])<.01
    assert 'orientation' not in packet  # No ideal AHRS enters the LIO input.
