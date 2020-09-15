import launch
from launch_ros.actions import Node
from disropt.utils.graph_constructor import binomial_random_graph
import numpy as np
import sys
import argparse

def generate_launch_description():
    # we must avoid the ROS arguments: -r, -p, -e
    ap = argparse.ArgumentParser(prog='ros2 launch ros_disropt pickup_delivery.launch.py')
    ap.add_argument("-tg", "--tasks-per-group", help="number of tasks per group (even number)", default=6, type=int)
    ap.add_argument("-g", "--groups", help="number of groups", default=3, type=int)
    ap.add_argument("-n", "--nodes", help="number of agents (>= groups)", default=3, type=int)
    ap.add_argument("-ep", "--edge-probability", help="edge probability", default=0.2, type=float)
    ap.add_argument("-gs", "--graph-seed", help="seed for random graph generator", default=10, type=int)
    ap.add_argument("-ts", "--task-seed", help="seed for random task generator", default=10, type=int)
    ap.add_argument("-lp", "--log-prefix", help="path prefix for log files", default='', type=str)

    try:
        args = vars(ap.parse_args(sys.argv[4:])) # skip "ros2 launch ros_disropt main.launch.py"
    except:
        return
    
    #######################

    n_agents = args['nodes']
    tasks_per_group = int(args['tasks_per_group']/2)
    n_groups = args['groups']
    p = args['edge_probability']
    graph_seed = args['graph_seed']
    task_seed = args['task_seed']
    log_prefix = args['log_prefix']

    # check inputs
    if tasks_per_group < 2:
        raise ValueError('There must be at least 4 tasks per group')
    if n_groups > n_agents:
        raise ValueError('The number of agents must be at least equal to the number of groups')

    # generate adjacency matrix
    Adj = binomial_random_graph(n_agents, p, graph_seed)
    
    #######################
    table = Node(
            package='ros_disropt', node_executable='ros_disropt_table', output='screen',
            # prefix=['xterm -e'],
            # prefix=['run_on_terminal'],
            parameters=[{'N': n_agents, 'n_groups': n_groups, 'tasks_per_group': tasks_per_group,
            'seed': task_seed, 'log_prefix': log_prefix}],
            on_exit=launch.actions.Shutdown())

    list_description = []

    list_description.append(table)

    for i in range(n_agents):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()

        list_description.append(Node(
            package='ros_disropt', node_executable='ros_disropt_guidance_i', output='screen',
            # prefix=['xterm -e'],
            # prefix=['run_on_terminal'],
            parameters=[{'agent_id': i, 'N': n_agents, 'in_neigh': in_neighbors,
            'out_neigh': out_neighbors, 'log_prefix': log_prefix}]))

        list_description.append(Node(
            package='ros_disropt', node_executable='ros_disropt_planner', output='screen',
            # prefix=['xterm -hold -e'],
            parameters=[{'agent_id': i}]))
        
        # list_description.append(Node(
        #     package='ros_disropt', node_executable='ros_disropt_controller', output='screen',
        #     # prefix=['xterm -hold -e']
        #     parameters=[{'agent_id': i}]))

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

    return launch.LaunchDescription(list_description)
