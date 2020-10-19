from rclpy.node import Node
from rclpy.action import ActionServer
from choirbot_interfaces.action import PositionAction
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
import time
import numpy as np


class MockPlanner(Node):

    def __init__(self):
        super().__init__('planner', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        self.agent_id = self.get_parameter('agent_id').value
        self.get_logger().info('Planner {} started'.format(self.agent_id))

class MockPointToPointPlanner(MockPlanner):

    def __init__(self):
        super().__init__()
        np.random.seed(self.agent_id)
        self._action_server = ActionServer(
            self,
            PositionAction,
            'positionaction',
            self.execute_callback)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.odom_timer = self.create_timer(1/100, self.odom_publish)
        self.current_pos = 3*np.random.rand(2)

    def execute_callback(self, goal_handle):
        positiontask_goal = goal_handle.request
        target_position = np.array(positiontask_goal.goal_position)
        self.get_logger().info('Moving robot to position {}'.format(target_position))

        speed = 0.1
        old_pos = self.current_pos
        new_pos = np.array(target_position)[0:2]
        distance = np.linalg.norm(old_pos - new_pos)
        wait_time = distance / speed

        self.get_logger().info('Time needed: {} seconds'.format(wait_time))
        time.sleep(wait_time)

        # send feedback
        # feedback_msg = PositionAction.Feedback()
        # feedback_msg.distance = 0.0
        # goal_handle.publish_feedback(feedback_msg)

        # succeeded
        goal_handle.succeed()
        final_position = np.copy(target_position)
        self.current_pos = np.copy(target_position)
        self.get_logger().info('Robot moved at position {}'.format(final_position))

        result = PositionAction.Result()
        result.final_position = list(final_position)
        return result

    def odom_publish(self):
        point = Point(x=self.current_pos[0], y=self.current_pos[1], z=0.0)
        pose = Pose(position=point)
        posewc = PoseWithCovariance(pose=pose)
        msg = Odometry(pose=posewc)
        self.odom_publisher.publish(msg)