import rclpy
from rclpy.node import Node
from .turtlebot3_position_control_feedback import Turtlebot3Feedback


def main():
    rclpy.init()

    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value
    
    turtlebot3_position_control = Turtlebot3Feedback(agent_id, 'pubsub', 'odom')
    rclpy.spin(turtlebot3_position_control)

    turtlebot3_position_control.destroy_node()
    rclpy.shutdown()
