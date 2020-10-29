from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
import sys
import argparse
import os


def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch choirbot_examples formationcontrol.launch.py')
    ap.add_argument("-l", "--length", help="length of hexagon sides", default=3, type=float)
    ap.add_argument("-s", "--seed", help="seed for initial positions", default=5, type=float)

    # parse arguments (exception thrown on error)
    args, _ = ap.parse_known_args(sys.argv)
    L = float(args.length)

    # set rng seed
    np.random.seed(args.seed)

    # communication matrix
    N = 6
    Adj = np.zeros((N,N))
    Adj[1::2,::2] = 1 # alternated zeros and ones
    Adj[::2,1::2] = 1

    # generate matrix of desired inter-robot distances
    W = Adj*L # adjacent robots have distance L
    W.ravel()[3:18:7] = 2*L # opposite robots has distance 2L
    W.ravel()[18::7]  = 2*L

    # generate coordinates of hexagon with center in the origin
    a = L/2
    b = np.sqrt(3)*a

    P = np.array([
        [-b, a , 0],
        [0, 2.0*a, 0],      
        [b, a, 0],
        [b, -a, 0],
        [0, -2.0*a, 0],
        [-b, -a, 0]
    ])
    
    # initial positions have a perturbation of at most L/3
    P += np.random.uniform(-L/3, L/3, (6,3))

    # initialize launch description
    robot_launch = []       # launched after 10 sec (to let Gazebo open)
    launch_description = [] # launched immediately (will contain robot_launch)

    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        weights = W[i,:].tolist()
        position = P[i, :].tolist()

        # guidance
        robot_launch.append(Node(
            package='choirbot_examples', node_executable='choirbot_formationcontrol_guidance', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors, 'weights': weights}]))
        
        # controller
        robot_launch.append(Node(
            package='choirbot_examples', node_executable='choirbot_formationcontrol_controller', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))
        
        # turtlebot spawner
        launch_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_turtlebot_spawner', output='screen',
            parameters=[{'namespace': 'agent_{}'.format(i), 'position': position}]))
    
    # include launcher for gazebo
    gazeebo_launcher = os.path.join(get_package_share_directory('choirbot_examples'), 'gazebo.launch.py')
    launch_description.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gazeebo_launcher)))
    
    # include delayed robot executables
    timer_action = TimerAction(period=10.0, actions=[LaunchDescription(robot_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
