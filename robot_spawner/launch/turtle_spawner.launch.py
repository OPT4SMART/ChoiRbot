from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
import sys
import os
import argparse

X_LIM = Y_LIM = [-5, 5]

def generate_launch_description():
    # program parameters (we must avoid -r -p -e)
    ap = argparse.ArgumentParser(prog='ros2 launch robot_spawner turtle_spawner.launch.py',
        description='Spawn Turtlebot3 robots.')
    ap.add_argument("-n", "--number", help="number of robots", default=3, type=int)
    ap.add_argument("--use-system-time", help="don't use simulation time according to the /clock topic", default=False, action='store_true')
    ap.add_argument("-sg", "--start_gazebo", help="whether or not to start Gazebo process", default=False, action='store_true')
    ap.add_argument("--param-wait", help="seconds before setting use_sim_time parameter", default=5, type=int)
    
    # parse arguments (exception thrown on error)
    known_args, _ = ap.parse_known_args(sys.argv[4:])
    args = vars(known_args)

    N = args['number']
    start_gazebo = args['start_gazebo']

    list_description = []

    # launch gazebo
    if start_gazebo:
        launch_file_dir = os.path.join(get_package_share_directory('robot_spawner'), 'launch')

        list_description.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource([launch_file_dir, '/gazebo.launch.py'])
            ))

    # spawn turtlebots
    for i in range(N):
        pos_x = np.random.uniform(X_LIM[0], X_LIM[1])
        pos_y = np.random.uniform(Y_LIM[0], Y_LIM[1])
        pos_z = 0
        position = [pos_x, pos_y, pos_z]

        list_description.append(Node(
            package='robot_spawner', node_executable='spawn_turtlebot', output='screen',
            parameters=[{'robot_namespace': 'agent_{}'.format(i), 'robot_pose': position}]))

    return LaunchDescription(list_description)
