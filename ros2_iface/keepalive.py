"""Keeps the realsense2_camera streams running: the node publishes the aligned depth only while somebody is subscribed, and it can take a while to start.
Holds the subscriptions for the whole run (restarting itself if no frames arrive) and writes a flag file once colour + aligned depth are flowing."""
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_ros import COLOR, DEPTH   # noqa: E402

FLAG = '/tmp/hw7_camera_ready'
RAW = '/camera/camera/depth/image_rect_raw'
INFO = '/camera/camera/aligned_depth_to_color/camera_info'
RETRY_S = 6.0


def subscribe(n, cnt):
    inc = lambda k: (lambda m: cnt.__setitem__(k, cnt[k] + 1))
    return [n.create_subscription(Image, COLOR, inc('color'), qos_profile_sensor_data),
            n.create_subscription(Image, DEPTH, inc('depth'), qos_profile_sensor_data),
            n.create_subscription(Image, RAW, inc('raw'), qos_profile_sensor_data),
            n.create_subscription(CameraInfo, INFO, lambda m: None, qos_profile_sensor_data)]


if os.path.exists(FLAG):
    os.remove(FLAG)
rclpy.init()
n = Node('hw7_camera_keepalive')
cnt = {'color': 0, 'depth': 0, 'raw': 0}
subs, t_sub = subscribe(n, cnt), time.time()
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
        if cnt['color'] > 5 and cnt['depth'] > 5:
            if not os.path.exists(FLAG):
                open(FLAG, 'w').write('ok')
        elif time.time() - t_sub > RETRY_S:
            print('no frames yet: restarting this process', flush=True)       # a new DDS participant recovers; re-subscribing in this one does not
            os.execv(sys.executable, [sys.executable] + sys.argv)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    if os.path.exists(FLAG):
        os.remove(FLAG)
