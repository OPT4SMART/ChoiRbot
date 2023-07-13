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
    L=3
    seed=5
    for arg in sys.argv:
        if arg.startswith("L:="):
            L = int(arg.split(":=")[1])
        if arg.startswith("seed:="):
            seed = int(arg.split(":=")[1])

    # set rng seed
    np.random.seed(seed)

    # communication matrix
    N = 6
    Adj = np.array([ # alternated zeros and ones
        [0, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0],
        [0, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 1, 0]
    ])

    # generate matrix of desired inter-robot distances
    # adjacent robots have distance L
    # opposite robots have distance 2L
    W = np.array([
        [0,   L,   0,   2*L, 0,   L],
        [L,   0,   L,   0,   2*L, 0],
        [0,   L,   0,   L,   0,   2*L],
        [2*L, 0,   L,   0,   L,   0],
        [0,   2*L, 0,   L,   0,   L],
        [L,   0,   2*L, 0,   L,   0]
    ])

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
            package='choirbot_examples', executable='choirbot_formationcontrol_guidance', output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors, 'weights': weights}]))
        
        # controller
        robot_launch.append(Node(
            package='choirbot_examples', executable='choirbot_formationcontrol_controller', output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))
        
        # turtlebot spawner
        launch_description.append(Node(
            package='choirbot_examples', executable='choirbot_turtlebot_spawner', output='screen',
            parameters=[{'namespace': 'agent_{}'.format(i), 'position': position}]))
    
    # include launcher for gazebo
    gazebo_launcher = os.path.join(get_package_share_directory('choirbot_examples'), 'gazebo.launch.py')
    launch_description.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launcher)))
    
    # include delayed robot executables
    timer_action = TimerAction(period=10.0, actions=[LaunchDescription(robot_launch)])
    launch_description.append(timer_action)

    return LaunchDescription(launch_description)
