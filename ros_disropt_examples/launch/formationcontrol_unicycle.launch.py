from launch import LaunchDescription
from launch_ros.actions import Node
from disropt.utils.graph_constructor import path_graph
import numpy as np
import sys
import argparse
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    ap = argparse.ArgumentParser(prog='ros2 launch ros_disropt planner.launch.py')
    ap.add_argument("-n", "--nodes", help="number of nodes", default=3, type=int)

    try:
        args = vars(ap.parse_args(sys.argv[4:])) # skip "ros2 launch ros_disropt main.launch.py"
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
        # initial_pos = P[i, :].tolist()

        # list_description.append(Node(
        #     package='ros_disropt_examples', node_executable='ros_disropt_unicycleintegrator', output='screen',
        #     node_namespace='agent_{}'.format(i),
        #     prefix=['xterm -hold -e'],
        #     parameters=[{'agent_id': i, 'init_pos': initial_pos}]))

        list_description.append(Node(
            package='ros_disropt_examples', node_executable='ros_disropt_unicycle_vel', output='screen',
            node_namespace='agent_{}'.format(i),
            #prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i}]))

        list_description.append(Node(
            package='ros_disropt_examples', node_executable='ros_disropt_formationcontrol', output='screen',
            node_namespace='agent_{}'.format(i),
            #prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors, 'weights': weights}]))

    #####################

    launch_file_dir = os.path.join(get_package_share_directory('robot_spawner'), 'launch')

    list_description.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/formation_spawner.launch.py'])
        ))
    

    return LaunchDescription(list_description)
