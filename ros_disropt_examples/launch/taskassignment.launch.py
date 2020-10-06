from launch import LaunchDescription
from launch_ros.actions import Node
from disropt.utils.graph_constructor import binomial_random_graph
import numpy as np
import sys
import argparse

def generate_launch_description():
    # we must avoid the ROS arguments: -r, -p, -e
    ap = argparse.ArgumentParser(prog='ros2 launch ros_disropt taskassignment.launch.py')
    ap.add_argument("-n", "--nodes", help="number of agents", default=3, type=int)
    ap.add_argument("-ep", "--edge-probability", help="edge probability", default=0.2, type=float)

    # parse arguments (exception thrown on error)
    known_args, _ = ap.parse_known_args(sys.argv)
    args = vars(known_args)

    N = args['nodes']
    p = args['edge_probability']
    
    #######################
    
    Adj = binomial_random_graph(N, p)

    table = Node(
            package='ros_disropt_examples', node_executable='ros_disropt_table', output='screen',
            prefix=['xterm -hold -e'], parameters=[{'N': N}])

    list_description = []

    list_description.append(table)

    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()

        list_description.append(Node(
            package='ros_disropt_examples', node_executable='ros_disropt_task_guidance', output='screen',
            node_namespace='agent_{}'.format(i),
            prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors}]))

        list_description.append(Node(
            package='ros_disropt_examples', node_executable='ros_disropt_planner', output='screen',
            node_namespace='agent_{}'.format(i),
            # prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i}]))
        
        list_description.append(Node(
            package='ros_disropt_examples', node_executable='ros_disropt_controller', output='screen',
            node_namespace='agent_{}'.format(i),
            # prefix=['xterm -hold -e']
            parameters=[{'agent_id': i}]))
        
    return LaunchDescription(list_description)
