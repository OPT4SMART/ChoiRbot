"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985
"""
import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from rclpy.node import Node

def main():
    """ Main for spawning turtlebot node """

    tb3_model = os.environ['TURTLEBOT3_MODEL']

    # initialize ros
    rclpy.init()

    node = Node('entity_spawner', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    robot_namespace = node.get_parameter('robot_namespace').value
    robot_pose = node.get_parameter('robot_pose').value

    # connect to service
    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    client.wait_for_service()
    node.get_logger().info("...connected!")

    # Get path to the turtlebot3 model
    sdf_file_path = os.path.join(
        get_package_share_directory("robot_spawner"), "models",
        "model_{}.sdf".format(tb3_model))

    # Set data for request
    request = SpawnEntity.Request()
    request.name = robot_namespace
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = robot_namespace
    request.initial_pose.position.x = robot_pose[0]
    request.initial_pose.position.y = robot_pose[1]
    request.initial_pose.position.z = robot_pose[2]

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
