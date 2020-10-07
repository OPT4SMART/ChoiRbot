import rclpy
from ros_disropt.planner import TwoDimPointToPointPlanner
from rclpy.executors import MultiThreadedExecutor


def main_planner():
    rclpy.init(args=None)

    goal_tolerance = 0.05
    pos_handler = 'pubsub'
    pos_topic = 'odom'
    # pos_handler = 'vicon'
    # pos_topic = '/vicon/T{}/T{}'.format(agent_id, agent_id)

    planner = TwoDimPointToPointPlanner(goal_tolerance, pos_handler, pos_topic)
    executor = MultiThreadedExecutor()
    rclpy.spin(planner, executor)

    rclpy.shutdown()
