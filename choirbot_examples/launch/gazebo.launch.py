from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    # launch gazebo
    list_description = [ExecuteProcess(cmd=['gazebo', '-s', 'libgazebo_ros_factory.so'], output='screen')]

    # set use_sim_time = true after 5 seconds
    action = ExecuteProcess(cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', 'true'], output='screen')
    list_description.append(TimerAction(period=5.0, actions=[action]))

    return LaunchDescription(list_description)
