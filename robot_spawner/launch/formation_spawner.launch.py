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
    ap.add_argument("--param-wait", help="seconds before setting use_sim_time parameter", default=5, type=int)
    
    # parse arguments (exception thrown on error)
    known_args, _ = ap.parse_known_args(sys.argv[4:])
    args = vars(known_args)

    N = args['number']

    list_description = []


    launch_file_dir = os.path.join(get_package_share_directory('robot_spawner'), 'launch')

    list_description.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/gazebo.launch.py'])
        ))

    D= 4.0
    a = D/4.0
    b = np.sqrt(3)*a

    P = np.array([
        [-b, a , 0],
        [0, 2.0*a, 0],      
        [b, a, 0],
        [b, -a, 0],
        [0, -2.0*a, 0],
        [-b, -a, 0]
    ])
    
    P += 3*np.random.rand(6,3)
    # spawn turtlebots
    for i in range(N):
        position = P[i, :].tolist()

        list_description.append(Node(
            package='robot_spawner', node_executable='spawn_turtlebot', output='screen',
            parameters=[{'robot_namespace': 'agent_{}'.format(i), 'robot_pose': position}]))

    return LaunchDescription(list_description)
