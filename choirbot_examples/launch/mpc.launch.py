from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from disropt.utils.graph_constructor import ring_graph
import numpy as np
import sys
import argparse
import os


def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch choirbot_examples mpc.launch.py')
    ap.add_argument("-n", "--number", help="number of robots", default=4, type=int)
    ap.add_argument("-s", "--seed", help="seed for initial positions", default=10, type=int)

    # parse arguments (exception thrown on error)
    args, _ = ap.parse_known_args(sys.argv)
    N = args.number

    # set rng seed
    np.random.seed(args.seed)
    
    # generate communication graph
    Adj = ring_graph(N)
    Adj[:, 0] = 1 # let all robots send information to robot 0

    # generate initial positions with x in [5, 5.5] and y = z = 0
    P = np.zeros((N, 3))
    P[:, 0] = np.random.uniform(5, 5.5, N)

    # initialize launch description with rviz executable
    rviz_config_dir = get_package_share_directory('choirbot_examples')
    rviz_config_file = os.path.join(rviz_config_dir, 'rvizconf.rviz')

    launch_description = [Node(package='rviz2', node_executable='rviz2', output='screen',
        arguments=['-d', rviz_config_file])]

    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        
        initial_pos = P[i, :].tolist()

        # guidance
        launch_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_mpc_guidance', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors}]))

        # integrator
        launch_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_mpc_integrator', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'init_pos': initial_pos}]))

        # rviz
        launch_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_mpc_rviz', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

    return LaunchDescription(launch_description)
