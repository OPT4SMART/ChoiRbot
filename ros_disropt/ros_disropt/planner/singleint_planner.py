from rclpy.node import Node
from rclpy.action import ActionServer
from ros_disropt_interfaces.action import VelocityAction
from .mock_planner import MockPlanner
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
import time
import numpy as np



class SingleIntPlanner(MockPlanner):

    def __init__(self, agent_id: int):
        super().__init__(agent_id)
        np.random.seed(agent_id)
        self._action_server = ActionServer(
            self,
            VelocityAction,
            'velocitytask_{}'.format(agent_id),
            self.execute_callback)
        self.odom_publisher = self.create_publisher(Odometry, '/agent_{}/odom'.format(agent_id), 10)
        self.odom_timer = self.create_timer(1/100, self.odom_publish)
        self.current_pos = 3*np.random.rand(2)
        self.u = np.zeros(2)
        self.steptime = 0.01

    def execute_callback(self, goal_handle):
        velocitytask_goal = goal_handle.request
        self.u = np.array(velocitytask_goal.goal_velocity)
        self.get_logger().info('Robot new input is {}'.format(self.u))

        # succeeded
        goal_handle.succeed()
        final_velocity = np.copy(self.u)

        result = VelocityAction.Result()
        result.final_velocity = list(final_velocity)
        return result

    def odom_publish(self):
        self.current_pos += self.steptime * self.u
        point = Point(x=self.current_pos[0], y=self.current_pos[1], z=0.0)
        pose = Pose(position=point)
        posewc = PoseWithCovariance(pose=pose)
        msg = Odometry(pose=posewc)
        self.odom_publisher.publish(msg)