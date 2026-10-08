#!/usr/bin/env python3
"""Offline ROS1 replay, supervised recording, and nonempty-output validation."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rosbag
import rosgraph

INPUT = Path('/data/input.bag')
OUTPUT = Path('/data/lio_output.bag')


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
        raise RuntimeError('ROS process did not shut down cleanly')


def start(*args):
    return subprocess.Popen(args, start_new_session=True)


def wait_ready(predicate, processes, description):
    deadline = time.monotonic() + 30
    while time.monotonic() < deadline:
        if any(p.poll() is not None for p in processes):
            raise RuntimeError('ROS process exited while waiting for ' + description)
        try:
            if predicate():
                return
        except (OSError, rosgraph.MasterException):
            pass  # Master may still be starting.
        time.sleep(0.1)
    raise RuntimeError('Timed out waiting for ' + description)


def main():
    if not INPUT.is_file():
        raise RuntimeError('Missing /data/input.bag')
    if OUTPUT.exists() or Path(str(OUTPUT) + '.active').exists():
        raise RuntimeError('Refusing to overwrite previous recording')
    with rosbag.Bag(str(INPUT)) as bag:
        topics = bag.get_type_and_topic_info().topics
        for topic, kind in [('/sim/lidar', 'sensor_msgs/PointCloud2'),
                            ('/sim/imu', 'sensor_msgs/Imu')]:
            if topic not in topics or topics[topic].msg_type != kind or not topics[topic].message_count:
                raise RuntimeError('Missing or invalid input topic: ' + topic)
        for _, cloud, _ in bag.read_messages(topics=['/sim/lidar']):
            if not {'x', 'y', 'z', 'intensity'}.issubset({f.name for f in cloud.fields}):
                raise RuntimeError('MARSIM requires PointXYZI fields')
            break
        print('Input:', {k: topics[k].message_count for k in ['/sim/lidar', '/sim/imu']}, flush=True)

    master = rosgraph.Master('/offline_supervisor')
    launch = record = player = None
    try:
        launch = start('roslaunch', '/opt/fast_lio/offline.launch')

        def mapping_ready():
            pubs, subs, _ = master.getSystemState()
            return all('/laserMapping' in dict(subs).get(topic, [])
                       for topic in ['/sim/lidar', '/sim/imu']) and '/Odometry' in dict(pubs)

        wait_ready(mapping_ready, [launch], 'mapping subscribers')
        record = start('rosbag', 'record', '-O', str(OUTPUT), '/Odometry', '__name:=lio_recorder')
        wait_ready(lambda: '/lio_recorder' in dict(master.getSystemState()[1]).get('/Odometry', []),
                   [launch, record], 'odometry recorder')
        # Slower than real time reduces backlog. --clock changes ROS clock only;
        # original cloud and IMU header stamps and payloads remain untouched.
        player = start('rosbag', 'play', '--clock', '--rate', '0.5', '--delay', '2',
                       str(INPUT), '--topics', '/sim/lidar', '/sim/imu')
        while player.poll() is None:
            if launch.poll() is not None or record.poll() is not None:
                raise RuntimeError('Mapping or recorder exited during playback')
            time.sleep(0.1)
        if player.returncode:
            raise RuntimeError('rosbag play failed: ' + str(player.returncode))
        # Allow callbacks and recorder buffers to drain using wall time, because
        # simulated time stops at EOF. Recorded stamp range is reported below.
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if launch.poll() is not None or record.poll() is not None:
                raise RuntimeError('Mapping or recorder exited at end of playback')
            time.sleep(0.1)
    finally:
        # Recorder gets SIGINT while mapping/master are still available so its
        # index is finalized. Always stop every child even if one stop fails.
        try:
            stop(player)
        finally:
            try:
                stop(record)
            finally:
                stop(launch)
    count = 0
    first_stamp = last_stamp = None
    with rosbag.Bag(str(OUTPUT)) as bag:
        for _, msg, _ in bag.read_messages(topics=['/Odometry']):
            values = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                      msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError('Nonfinite odometry')
            stamp = msg.header.stamp.to_sec()
            if last_stamp is not None and stamp < last_stamp:
                raise RuntimeError('Odometry stamps regress')
            first_stamp = stamp if first_stamp is None else first_stamp
            last_stamp = stamp
            count += 1
    if not count:
        raise RuntimeError('FAST-LIO produced no /Odometry messages')
    print(f'Recorded {count} finite odometry messages, stamps {first_stamp}..{last_stamp}. '
          'This is a recording check, not an accuracy evaluation.', flush=True)


def interrupted(*_):
    raise KeyboardInterrupt


if __name__ == '__main__':
    # Turn container SIGTERM into an exception so child cleanup finalizes bags.
    signal.signal(signal.SIGTERM, interrupted)
    main()
