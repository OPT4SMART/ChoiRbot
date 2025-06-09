from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
import pathlib

def get_webots_driver(agent_id):
    package_dir_driver = get_package_share_directory('choirbot_examples')
    robot_description = pathlib.Path(os.path.join(package_dir_driver, 'turtlebot_unicycle.urdf')).read_text()

    turtlebot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        namespace=f'agent_{agent_id}',
        output='screen',
        additional_env={
            'WEBOTS_ROBOT_NAME':f'agent_{agent_id}',
            },
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return turtlebot_driver


def generate_webots_world_file(robots, source_filename, target_filename):
    with open(source_filename, 'r') as source_file:
        contents = source_file.read()

    with open(target_filename, 'w') as target_file:
        target_file.write(contents)

        for robot in robots:
            template_filename = os.path.join(os.path.dirname(source_filename), f'obj_{robot["type"]}.wbt')
            with open(template_filename, 'r') as template_file:
                template = template_file.read()
                template = template.replace('$NAME', robot["name"])
                template = template.replace('$X', str(robot["position"][0]))
                template = template.replace('$Y', str(robot["position"][1]))
                template = template.replace('$Z', str(robot["position"][2]))
                target_file.write(template)

def generate_launch_description():

    # communication matrix
    N = 6

    np.random.seed(42)  # for reproducibility

    P = np.zeros((N, 3))
    P[:,:2] = np.random.rand(N, 2)*N
    robots = [{
            'name': f'agent_{i}',
            'type': 'turtlebot', 
            'position': P[i].tolist(), 
        } for i in range(N) ]

    # generate matrix of desired inter-robot distances
    # adjacent robots have distance L
    # opposite robots have distance 2L
    
    L = 3
    D = 2*L
    H = np.sqrt(3)*L

    W = np.array([
        [0,   L,   0,   D,   H,   L],
        [L,   0,   L,   0,   D,   0],
        [0,   L,   0,   L,   0,   D],
        [D,   0,   L,   0,   L,   0],
        [H,   D,   0,   L,   0,   L],
        [L,   0,   D,   0,   L,   0]
    ])

    Adj = W > 0

    # initialize launch description
    launch_description = [] # launched immediately

    # Generate Webots world file
    world_package_dir = get_package_share_directory('choirbot_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'choirbot_examples.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=os.path.join(world_package_dir, 'worlds', 'choirbot_examples.wbt'))
    
    # initialize launch description
    launch_description = []
    launch_description.append(webots)


    # distance-based neighbors getter
    launch_description.append(Node(
        package='choirbot_examples', executable='choirbot_formationcontrol_closestrobotgetter', output='screen',
        parameters=[{'first_id': 0, 'last_id':N, 'freq': 100.0}]))
    
   
    # add executables for each robot
    for i in range(N):

        in_neighbors  = np.nonzero(Adj[:, i])[0].tolist()
        out_neighbors = np.nonzero(Adj[i, :])[0].tolist()
        weights = W[i,:].tolist()

        # webots exec
        launch_description.append(get_webots_driver(i))
        launch_description.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            additional_env={'WEBOTS_ROBOT_NAME':f'agent_{i}'},
            namespace=f'agent_{i}',
            output='screen',
            parameters=[{
                'robot_description': '<robot name=""><link name=""/></robot>',
                }]))

        # guidance
        launch_description.append(Node(
            package='choirbot_examples', executable='choirbot_formationcontrol_guidance', output='screen',
            namespace=f'agent_{i}',
            parameters=[{'agent_id': i, 'N': N, 'in_neigh': in_neighbors, 'out_neigh': out_neighbors, 'weights': weights}]))
        
        # controller
        launch_description.append(Node(
            package='choirbot_examples', executable='choirbot_formationcontrol_controller', output='screen',
            namespace=f'agent_{i}',
            remappings=[('cmd_vel', 'cmd_vel_des')],
            parameters=[{'agent_id': i}]))
        
        # collision avoidance
        launch_description.append(Node(
            package='choirbot_examples', executable='choirbot_formationcontrol_collision', output='screen', 
            namespace=f'agent_{i}',
            parameters=[{'agent_id': i, 'N':N}]))

    return LaunchDescription(launch_description)
