from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import argparse

def generate_launch_description():

    # parse command line arguments
    # we must avoid the ROS arguments: -r, -p, -e
    ap = argparse.ArgumentParser(prog='ros2 launch ros_disropt planner.launch.py')
    ap.add_argument("-n", "--nodes", help="number of nodes", default=1, type=int)

    try:
        args = vars(ap.parse_args(sys.argv[4:])) # skip "ros2 launch ros_disropt main.launch.py"
    except:
        return None
    
    #######################

    N = args['nodes']
    
    #######################

    list_description = []

    for i in range(N):

        list_description.append(Node(
            package='ros_disropt', node_executable='ros_disropt_planner', output='screen',
            prefix=['xterm -hold -e'], parameters=[{'agent_id': i}]))

    return LaunchDescription(list_description)
