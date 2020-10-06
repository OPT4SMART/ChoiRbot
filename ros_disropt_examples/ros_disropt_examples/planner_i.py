import rclpy
from rclpy.node import Node
from ros_disropt.planner import PointToPointPlanner
from rclpy.executors import MultiThreadedExecutor


def main_planner():
    rclpy.init(args=None)

    pos_handler = 'pubsub'
    pos_topic = 'odom'
    # pos_handler = 'vicon'
    # pos_topic = '/vicon/T{}/T{}'.format(agent_id, agent_id)

    planner = PointToPointPlanner(pos_handler, pos_topic)
    executor = MultiThreadedExecutor()
    rclpy.spin(planner, executor)

    rclpy.shutdown()
