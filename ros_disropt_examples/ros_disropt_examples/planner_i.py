import rclpy
from rclpy.node import Node
from ros_disropt.planner import PointToPointPlanner
from ros_disropt.planner.mock_planner import MockPointToPointPlanner
from rclpy.executors import MultiThreadedExecutor


def main_planner():
    rclpy.init(args=None)
    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value

    pos_handler = 'pubsub'
    pos_topic = '/agent_{}/odom'.format(agent_id)
    # pos_handler = 'vicon'
    # pos_topic = '/vicon/T{}/T{}'.format(agent_id, agent_id)

    #planner = PointToPointPlanner(agent_id, pos_handler, pos_topic)
    planner = MockPointToPointPlanner(agent_id)
    executor = MultiThreadedExecutor()
    rclpy.spin(planner, executor)

    # rclpy.shutdown()
