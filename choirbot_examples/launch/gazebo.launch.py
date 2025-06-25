from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ros_gz_sim = FindPackageShare('ros_gz_sim')

    world = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    ])

    gzserver_cmd = IncludeLaunchDescription(
        PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world], 
            'on_exit_shutdown': 'true'
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': '-g -v2 '
        }.items()
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        PathJoinSubstitution([
            FindPackageShare('turtlebot3_gazebo'),
            'models'
        ])
    )
    
    # ROS gazebo global topic bridge
    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_world',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(bridge_cmd)

    return ld