from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from disropt.utils.graph_constructor import binomial_random_graph
import numpy as np
import sys
import os


def generate_launch_description():
    N=6
    seed=3
    for arg in sys.argv:
        if arg.startswith("N:="):
            N = int(arg.split(":=")[1])
        if arg.startswith("seed:="):
            seed = int(arg.split(":=")[1])

    # generate communication graph (this function also sets the seed)
    Adj = binomial_random_graph(N, p=0.2, seed=seed)

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
