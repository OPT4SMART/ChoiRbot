from launch import LaunchDescription
from launch_ros.actions import Node
from disropt.utils.graph_constructor import ring_graph, binomial_random_graph, path_graph
import numpy as np
import sys
import argparse

def generate_launch_description():

    # parse command line arguments
    # we must avoid the ROS arguments: -r, -p, -e
    ap = argparse.ArgumentParser(prog='ros2 launch ros_disropt system.launch.py')
    ap.add_argument("-n", "--nodes", help="number of nodes", default=3, type=int)
    ap.add_argument("-g", "--graph", help="type of graph", default='path', choices=['ring', 'binomial', 'complete','path'])
    ap.add_argument("-d", "--directed", help="whether the generated graph is directed", default=True, type=bool)
    ap.add_argument("-i", "--iterations", help="maximum number of iterations", default=10, type=int)
    # reqArgs = ap.add_argument_group('required arguments')
    # reqArgs.add_argument("-ne", "--node_exec", help="executable to give to nodes", required=True)

    try:
        args = vars(ap.parse_args(sys.argv[4:])) # skip "ros2 launch ros_disropt main.launch.py"
    except:
        return None
    
    #######################

    N = args['nodes']
    link_type = 'directed' if args['directed'] else 'undirected'
    max_iterations = args['iterations']

    if args['graph'] == 'ring':
        Adj = ring_graph(N, link_type=link_type)
    elif args['graph'] == 'binomial':
        Adj = binomial_random_graph(N, 0.2, seed=1, link_type=link_type)
    elif args['graph'] == 'path':
        Adj = path_graph(N)
    else:
        Adj = (np.ones((N, N)) - np.eye(N)).astype(int)
    
    #######################

    list_description = []

    list_description.append(Node(
            package='ros_disropt', node_executable='ros_disropt_table', output='screen',
            prefix=['xterm -hold -e'], parameters=[{'N': N}]))

    for i in range(N):
        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()

        list_description.append(Node(
            package='ros_disropt', node_executable='ros_disropt_guidance_i', output='screen',
            prefix=['xterm -hold -e'], parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors,
            'out_neigh': out_neighbors, 'max_iterations': max_iterations}]))
        
        list_description.append(Node(
            package='ros_disropt', node_executable='ros_disropt_planner', output='screen',
            prefix=['xterm -hold -e'], parameters=[{'agent_id': i}]))

    return LaunchDescription(list_description)
