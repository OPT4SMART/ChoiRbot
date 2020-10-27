from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from disropt.utils.graph_constructor import ring_graph
import numpy as np
import sys
import argparse
import os

def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch choirbot mpc.launch.py')
    ap.add_argument("-n", "--nodes", help="number of nodes", default=4, type=int)

    try:
        args = vars(ap.parse_args(sys.argv[4:])) # skip "ros2 launch choirbot mpc.launch.py"
    except:
        return None
    
    #######################
    N = args['nodes']
    Adj = ring_graph(N)
    Adj[:, 0] = 1 # make agent 0 out-neighbor of everyone

    P = np.zeros((N, 3))
    P[:, 0] = np.random.uniform(5, 5.5, N)

    #######################

    rviz_config_dir = get_package_share_directory('choirbot_examples')
    rviz_config_file = os.path.join(rviz_config_dir, 'rvizconf.rviz')

    list_description = [Node(
            package='rviz2', node_executable='rviz2', output='screen',
            arguments=['-d', rviz_config_file])]

    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        
        initial_pos = P[i, :].tolist()

        list_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_singleintegrator', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i, 'init_pos': initial_pos}]))

        list_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_mpc', output='screen',
            node_namespace='agent_{}'.format(i),
            prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors}]))
        
        list_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_rviz', output='screen',
            node_namespace='agent_{}'.format(i),
            parameters=[{'agent_id': i}]))

    return LaunchDescription(list_description)
