"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985
"""
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import os


def main():
    rclpy.init()

    # get robot parameters
    node = Node('entity_spawner', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    namespace = node.get_parameter('namespace').value
    position = node.get_parameter('position').value

    # connect to service
    node.get_logger().info('Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    client.wait_for_service()
    node.get_logger().info("...connected!")

    # Get path to the turtlebot3 model
    sdf_file_path = os.path.join(
        get_package_share_directory("choirbot_examples"), "model_burger.sdf")

    # Set data for request
    request = SpawnEntity.Request()
    request.name = namespace
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = namespace
    request.initial_pose.position.x = position[0]
    request.initial_pose.position.y = position[1]
    request.initial_pose.position.z = position[2]

    # send request
    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # print result
    if future.result() is not None:
        node.get_logger().info('response: %r' % future.result())
    else:
        raise RuntimeError('exception while calling service: %r' % future.exception())

    # shut down this node
    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()
