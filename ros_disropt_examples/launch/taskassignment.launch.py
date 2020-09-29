from launch import LaunchDescription
from launch_ros.actions import Node
from disropt.utils.graph_constructor import path_graph
import numpy as np
import sys
import argparse
import os

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
    Adj = path_graph(N)
    
    #######################

    list_description = []

    list_description.append(Node(
            package='ros_disropt', node_executable='ros_disropt_table', output='screen',
            prefix=['xterm -hold -e'],
            # prefix=['run_on_terminal'],
            parameters=[{'N': N}]))

    # for i in range(N):

    #     in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
    #     out_neighbors = np.nonzero(Adj[i, :])[0].tolist()

    #     list_description.append(Node(
    #         package='ros_disropt', node_executable='ros_disropt_guidance_i', output='screen',
    #         prefix=['xterm -hold -e'],
    #         # prefix=['run_on_terminal'],
    #         parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors,
    #         'out_neigh': out_neighbors}]))

    #     list_description.append(Node(
    #         package='ros_disropt', node_executable='ros_disropt_planner', output='screen',
    #         # prefix=['xterm -hold -e'],
    #         parameters=[{'agent_id': i}]))
        
    #     list_description.append(Node(
    #         package='ros_disropt', node_executable='ros_disropt_controller', output='screen',
    #         # prefix=['xterm -hold -e']
    #         parameters=[{'agent_id': i}]))

        # square_area = 3.0 # agents are spawned randomly in a square with this side length
        # random_pose = (square_area*(np.random.rand(2,1)).flatten()).tolist()
        # random_pose.append(0.0) # z = 0

        # list_description.append(Node(
        #     package='robot_spawner_pkg', node_executable='spawn_turtlebot', output='screen',
        #     parameters=[{'robot_namespace': 'agent_{}'.format(i), 'robot_pose': random_pose}]),)

    ######################

    # launch_file_dir = os.path.join(get_package_share_directory('robot_spawner_pkg'), 'launch')
    # list_description.append(IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([launch_file_dir, '/launch_turtle_spawner.launch.py']),
    #         launch_arguments={'agent_number': N}.items()
    # ))
    
    # use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # list_description.append(ExecuteProcess(
    #         cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
    #         output='screen'))

    # list_description.append(ExecuteProcess(
    #         cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
    #         output='screen'))

    # VICON

    # list_description.append(Node(
    #         package='vicon_receiver', node_executable='vicon_client', output='screen',
    #         prefix=['xterm -hold -e'], parameters=[{'hostname': '192.168.10.1',
    #         'buffer_size': 200, 'namespace': 'vicon', 'frequency': 100}]))

    return LaunchDescription(list_description)
