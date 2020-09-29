import rclpy
from rclpy.node import Node
from ros_disropt.planner import SingleIntPlanner
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init(args=None)
    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value

    planner = SingleIntPlanner(agent_id)
    rclpy.spin(planner)
