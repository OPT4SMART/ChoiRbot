from rclpy.node import Node
from .. import Pose
from .controller import Controller
from geometry_msgs.msg import Vector3, Twist
import numpy as np


class UnicycleVelocityController(Controller):

    def __init__(self, pose_handler: str=None, pose_topic: str=None):
        super().__init__(pose_handler, pose_topic)
        self.subscription = self.create_subscription(Vector3, 'velocity', self.control_callback, 1)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.yaw = 0.0
        self.yaw_old = 0.0
        self.yaw_old_old = 0.0

    def control_callback(self, msg):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return

        u_ = np.array([msg.x, msg.y, msg.z])     
        self.get_yaw()
        a = np.cos(self.yaw)
        b = np.sin(self.yaw)
        u = np.zeros(2)
        u[0] = 0.5*(a*u_[0] + b*u_[1])
        u[1] = (np.pi/8.0)*np.arctan2(-b*u_[0] + a*u_[1], u_[0])/(np.pi/2)
        self.send_input(u)

    def send_input(self, u):
        msg = Twist()
        msg.linear.x = u[0]
        msg.angular.z = u[1]
        self.publisher_.publish(msg)

    def get_yaw(self):
        quat = np.copy(self.current_pose.orientation)
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw_new = np.arctan2(siny_cosp, cosy_cosp)
        yaw_array = np.array([self.yaw_old_old, self.yaw_old, yaw_new])
        yaw_array_new = np.unwrap(yaw_array)

        # shift value of variables
        self.yaw, self.yaw_old, self.yaw_old_old = yaw_array_new[2], self.yaw, self.yaw_old
