"""ros2 launch ros2_iface/launch/pick.launch.py [extra_args:='--no-delay']

Starts the camera keepalive, the four stage nodes (scan, perception, planner, executor) and the orchestrator; stops everything when the run ends.
Needs the ros2_kortex driver, MoveIt and realsense2_camera already running (other terminals), all with RMW_IMPLEMENTATION=rmw_cyclonedds_cpp.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PY = os.path.join(ROOT, '.venv', 'bin', 'python')
NODES = os.path.join(ROOT, 'ros2_iface', 'hw7_nodes.py')


def generate_launch_description():
    def proc(name, cmd):
        return ExecuteProcess(cmd=cmd, cwd=ROOT, output='screen', name=name)
    wait = 'for i in $(seq 1 90); do [ -f /tmp/hw7_camera_ready ] && break; sleep 1; done; exec '       # open the camera only once the keepalive has the streams flowing
    stage = [proc(r, ['bash', '-c', wait + f'{PY} {NODES} {r}']) for r in ('scan', 'perception', 'planner', 'executor')]
    keepalive = proc('camera_keepalive', ['/usr/bin/python3', os.path.join(ROOT, 'ros2_iface', 'keepalive.py')])
    manager = proc('manager', [PY, os.path.join(ROOT, 'run_pick.py'), LaunchConfiguration('extra_args')])
    return LaunchDescription([
        DeclareLaunchArgument('extra_args', default_value='--no-delay', description='arguments for run_pick.py (--no-delay, --single)'),
        keepalive, *stage, manager,
        RegisterEventHandler(OnProcessExit(target_action=manager, on_exit=[EmitEvent(event=Shutdown())])),
    ])
