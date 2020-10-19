from launch import LaunchDescription
from launch_ros.actions import Node
from disropt.utils.graph_constructor import path_graph
import numpy as np
import sys
import argparse
import os

def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch choirbot planner.launch.py')
    ap.add_argument("-n", "--nodes", help="number of nodes", default=3, type=int)

    try:
        args = vars(ap.parse_args(sys.argv[4:])) # skip "ros2 launch choirbot main.launch.py"
    except:
        return None
    
    #######################
    N = args['nodes']
    Adj = np.array([[0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0],
                    [0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0],
                    [0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0]])

    # Weight matrix to control inter-agent distances
    D= 4.0
    a = D/4.0
    b = np.sqrt(3)*a
    L = np.sqrt(a**2 * b**2)
    D1 = np.sqrt((2*a)**2 + (2*b)**2)

    W = np.array([
        [0,         L,      0,     D1,    0,     2.0*a],
        [L,         0,      L,     0,     4.0*a, 0],
        [0,         L,      0,     2.0*a, 0,     D1],
        [D1,        0,      2.0*a, 0,     L,     0],
        [0,         4.0*a,  0,     L,     0,     L],
        [2.0*a,     0,      D1,    0,     L,     0]
    ])

    P = np.array([
        [-b, a , 0],
        [0, 2.0*a, 0],      
        [b, a, 0],
        [b, -a, 0],
        [0, -2.0*a, 0],
        [-b, -a, 0]
    ])
    
    P += np.random.rand(6,3)

    #######################

    list_description = []

    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        weights = W[i,:].tolist()
        initial_pos = P[i, :].tolist()

        list_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_singleintegrator', output='screen',
            node_namespace='agent_{}'.format(i),
            #prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i, 'init_pos': initial_pos}]))

        list_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_formationcontrol', output='screen',
            node_namespace='agent_{}'.format(i),
            #prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors, 'weights': weights}]))

        list_description.append(Node(
            package='choirbot_examples', node_executable='choirbot_rviz', output='screen',
            node_namespace='agent_{}'.format(i),
            #prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i}]))

    return LaunchDescription(list_description)
