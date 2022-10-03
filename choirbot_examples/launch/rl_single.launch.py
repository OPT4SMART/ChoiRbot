from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np
import os


def generate_launch_description():
    np.random.seed(0) # seed for initial position

    # generate initial position in [-3, 3] with z = 0
    P = np.zeros(3)
    P[0:2] = np.random.randint(-3, 3)

    # initialize launch description with rviz executable
    rviz_config_dir = get_package_share_directory('choirbot_examples')
    rviz_config_file = os.path.join(rviz_config_dir, 'rvizconf.rviz')

    launch_description = [Node(package='rviz2', node_executable='rviz2', output='screen',
        arguments=['-d', rviz_config_file])]
    
    # add executables
    initial_pos = P.tolist()

    # guidance
    launch_description.append(Node(
        package='choirbot_examples', node_executable='choirbot_rl_guidance_single', output='screen',
        node_namespace='agent_0',
        parameters=[{'agent_id': 0, 'N': 0}]))

    # integrator
    launch_description.append(Node(
        package='choirbot_examples', node_executable='choirbot_rl_integrator', output='screen',
        node_namespace='agent_0',
        parameters=[{'agent_id': 0, 'init_pos': initial_pos}]))

    # rviz
    launch_description.append(Node(
        package='choirbot_examples', node_executable='choirbot_rl_rviz', output='screen',
        node_namespace='agent_0',
        parameters=[{'agent_id': 0}]))

    return LaunchDescription(launch_description)
