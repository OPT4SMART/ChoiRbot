from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import os
import numpy as np

def get_webots_driver(agent_id):
    package_dir_driver = get_package_share_directory('choirbot_examples')
    robot_description_path = os.path.join(package_dir_driver, 'turtlebot_unicycle.urdf')
    
    turtlebot_driver = WebotsController(
        robot_name=f'agent_{agent_id}',
        namespace=f'agent_{agent_id}',
        parameters=[
            {'robot_description': robot_description_path},
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
                template = template.replace('$Z', str(0.0))
                target_file.write(template)
                
def generate_launch_description():

    N = 1
    robots = []

    # Turltebots init positions
    P = np.zeros((N, 3))
    P[:,0] = np.linspace(-2.0, 2.0, N)

    robots += [{
        'name': f'agent_{i}',
        'type': 'turtlebot', 
        'position': P[i].tolist(), 
    } for i in range(N) ]

    # Targets list
    targets = [[0.0,1.0,0.0], [1.0,1.0,0.0]]#, [1.0,0.0,0.0], [0.0,0.0,0.0]]

    robots +=[{
        'name': f'target_{i}',
        'type': 'target', 
        'position': targets[i], 
    } for i in range(len(targets))]

    # Generate Webots world file
    world_package_dir = get_package_share_directory('choirbot_examples')
    source_filename = os.path.join(world_package_dir, 'worlds', 'empty_world.wbt')
    target_filename = os.path.join(world_package_dir, 'worlds', 'choirbot_examples.wbt')
    generate_webots_world_file(robots, source_filename, target_filename)            
    webots = WebotsLauncher(world=os.path.join(world_package_dir, 'worlds', 'choirbot_examples.wbt'))
    
    # initialize launch description
    launch_description = []
    launch_description.append(webots)
    launch_description.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        )
    )
    
    # add executables for each robot
    for i in range(N):
        initial_position = P[i, :].tolist()
        targets = np.array(targets).flatten().tolist()

        # webots exec
        launch_description.append(get_webots_driver(i))
            
        launch_description.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                additional_env={'WEBOTS_ROBOT_NAME':f'agent_{i}'},
                namespace=f'agent_{i}',
                output='screen',
                parameters=[{
                    'robot_description': '<robot name=""><link name=""/></robot>',
                }]
            )
        )

        launch_description.append(
            Node(
                package='choirbot_examples',
                executable='choirbot_webots_guidance', 
                output='screen',
                prefix=f'xterm  -title "guidance_{i}" -hold -e ',
                namespace=f'agent_{i}',
                parameters=[{
                    'agent_id': i, 
                    'N': N, 
                    'in_neigh': [False] * N,   # No in-neighbors for this example
                    'out_neigh': [False] * N,  # No in-neighbors for this example, 
                    'init_pos': initial_position,
                    'targets': targets,
                }]
            )
        )
            
        # controller
        launch_description.append(
            Node(
                package='choirbot_examples', 
                executable='choirbot_webots_controller', 
                output='screen',
                namespace=f'agent_{i}',
                parameters=[{'agent_id': i }]
            )
        )
        
    return LaunchDescription(launch_description)
