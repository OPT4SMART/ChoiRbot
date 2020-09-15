from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import sys
import argparse

def generate_launch_description():
    # program parameters (we must avoid -r -p -e)
    ap = argparse.ArgumentParser(prog='ros2 launch robot_spawner gazebo.launch.py',
        description='Launch Gazebo simulator.')
    ap.add_argument("--use-system-time", help="don't use simulation time according to the /clock topic", default=False, action='store_true')
    ap.add_argument("--param-wait", help="seconds before setting use_sim_time parameter", default=5, type=int)
    
    # parse arguments (exception thrown on error)
    known_args, _ = ap.parse_known_args(sys.argv)
    args = vars(known_args)
    
    use_system_time = args['use_system_time']
    wait_time = args['param_wait']

    # launch gazebo
    list_description = [ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen')]

    if not use_system_time:
        action = ExecuteProcess(cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', 'true'], output='screen')
        list_description.append(TimerAction(period=float(wait_time), actions=[action]))

    return LaunchDescription(list_description)
