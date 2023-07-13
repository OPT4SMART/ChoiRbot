from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from disropt.utils.graph_constructor import binomial_random_graph
import numpy as np
import sys
import argparse
import os


def generate_launch_description():
    N=4
    seed=3
    for arg in sys.argv:
        if arg.startswith("N:="):
            N = int(arg.split(":=")[1])
        if arg.startswith("seed:="):
            seed = int(arg.split(":=")[1])

    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, 0.2, seed=seed)

    # generate initial positions in [-3, 3] with z = 0
    P = np.zeros((N, 3))
    P[:, 0:2] = np.random.randint(-3, 3, (N, 2))

    # initialize launch description
    robot_launch = []       # launched after 10 sec (to let Gazebo open)
    launch_description = [] # launched immediately (will contain robot_launch)

    # add task table executable
    robot_launch.append(Node(
            package='choirbot_examples', executable='choirbot_taskassignment_table', output='screen',
            prefix=['xterm -hold -e'],
            parameters=[{'N': N}]))

    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        position = P[i, :].tolist()

        # guidance
        robot_launch.append(Node(
            package='choirbot_examples', executable='choirbot_taskassignment_guidance', output='screen',
            prefix=['xterm -hold -e'],
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors}]))
        
        # planner
        robot_launch.append(Node(
            package='choirbot_examples', executable='choirbot_taskassignment_planner', output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))
        
        # controller
        robot_launch.append(Node(
            package='choirbot_examples', executable='choirbot_taskassignment_controller', output='screen',
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
