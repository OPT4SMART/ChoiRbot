import rclpy
from choirbot.planner import TwoDimPointToPointPlanner
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init()

    goal_tolerance = 0.05

    planner = TwoDimPointToPointPlanner(goal_tolerance, 'pubsub', 'odom')

    executor = MultiThreadedExecutor()
    rclpy.spin(planner, executor)
    rclpy.shutdown()
