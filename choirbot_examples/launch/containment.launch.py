from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from disropt.utils.graph_constructor import binomial_random_graph
import numpy as np
import sys
import argparse
import os


def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch choirbot_examples containment.launch.py')
    ap.add_argument("-n", "--number", help="number of robots", default=6, type=int)
    ap.add_argument("-s", "--seed", help="seed for initial positions", default=3, type=int)

    # parse arguments (exception thrown on error)
    args, _ = ap.parse_known_args(sys.argv)
    N = args.number # shorthand

    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, p=0.2, seed=args.seed)

    # generate initial positions in [-3, 3] with z = 0
    P = np.zeros((N, 3))
    P[:, 0:2] = np.random.randint(-3, 3, (N, 2))

    # initialize launch description with rviz executable
    rviz_config_dir = get_package_share_directory('choirbot_examples')
    rviz_config_file = os.path.join(rviz_config_dir, 'rvizconf.rviz')

    launch_description = [Node(package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_config_file])]
    
    # add executables for each robot
    for i in range(N):

        in_neighbors = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()

        is_leader = True if i < N/2 else False
        initial_pos = P[i, :].tolist()

        # guidance
        launch_description.append(Node(
            package='choirbot_examples', executable='choirbot_containment_guidance', output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors, 'is_leader': is_leader}]))

        # integrator
        launch_description.append(Node(
            package='choirbot_examples', executable='choirbot_containment_integrator', output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'init_pos': initial_pos}]))

        # rviz
        launch_description.append(Node(
            package='choirbot_examples', executable='choirbot_containment_rviz', output='screen',
            namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

    return LaunchDescription(launch_description)
