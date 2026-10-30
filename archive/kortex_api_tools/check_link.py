"""Read-only link test: arm state + one D405 frame. Sends NO motion commands."""
import sys
sys.path.insert(0, '/home/aryans-cat/Documents/GitHub/VBRM_insertion/Kinova-kortex2_Gen3_G3L/api_python/examples')

import numpy as np
import pyrealsense2 as rs
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from utilities import DeviceConnection

IP = '192.168.1.10'

with DeviceConnection(IP, port=DeviceConnection.TCP_PORT, credentials=('admin', 'admin')) as router:
    base = BaseClient(router)
    q = base.GetMeasuredJointAngles()
    print('joints (deg):', [round(a.value, 1) for a in q.joint_angles])
    p = base.GetMeasuredCartesianPose()
    print('tool pose  xyz(m)=', round(p.x, 3), round(p.y, 3), round(p.z, 3),
          ' theta xyz(deg)=', round(p.theta_x, 1), round(p.theta_y, 1), round(p.theta_z, 1))

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
prof = pipe.start(cfg)
for _ in range(15):
    frames = pipe.wait_for_frames()
d = frames.get_depth_frame()
scale = prof.get_device().first_depth_sensor().get_depth_scale()
z = np.asanyarray(d.get_data()) * scale
valid = z[z > 0]
print('depth scale', scale, '| valid px', valid.size, '| range m', valid.min(), valid.max())
i = d.profile.as_video_stream_profile().intrinsics
print('intrinsics fx fy cx cy', i.fx, i.fy, i.ppx, i.ppy)
pipe.stop()
