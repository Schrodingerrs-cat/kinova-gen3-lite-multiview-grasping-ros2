"""Keeps the realsense2_camera streams running: the node publishes the aligned depth only while somebody is subscribed, and it takes it a while to
start after the first subscriber. This holds a subscription for the whole run and writes a flag file once colour + aligned depth are flowing."""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_ros import COLOR, DEPTH   # noqa: E402

FLAG = '/tmp/hw6_camera_ready'
if os.path.exists(FLAG):
    os.remove(FLAG)
rclpy.init()
n = Node('hw6_camera_keepalive')
cnt = {'color': 0, 'depth': 0, 'raw': 0}
RAW = '/camera/camera/depth/image_rect_raw'
n.create_subscription(Image, COLOR, lambda m: cnt.__setitem__('color', cnt['color'] + 1), qos_profile_sensor_data)
n.create_subscription(Image, DEPTH, lambda m: cnt.__setitem__('depth', cnt['depth'] + 1), qos_profile_sensor_data)
n.create_subscription(Image, RAW, lambda m: cnt.__setitem__('raw', cnt['raw'] + 1), qos_profile_sensor_data)      # the aligned stream only starts reliably with the raw depth subscribed too
n.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', lambda m: None, qos_profile_sensor_data)
try:
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
        if cnt['color'] > 5 and cnt['depth'] > 5 and not os.path.exists(FLAG):
            open(FLAG, 'w').write('ok')
finally:
    if os.path.exists(FLAG):
        os.remove(FLAG)
