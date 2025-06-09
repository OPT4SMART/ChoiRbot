import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from .motor_ctrl import MotorCtrl

import numpy as np


class MotorCtrlUnicycle(MotorCtrl):

    def init(self, webots_node, properties):
        super().init(webots_node, properties)

        print('MotorCrtl Started!')

        # Declare Subscriptions
        msg_type = Twist()
        self.target_cmd_vel = msg_type
        self.cf_driver.create_subscription(msg_type, '/{}/cmd_vel'.format(self.namespace), self.setpoint_callback, 1)
        self.odom_publisher = self.cf_driver.create_publisher(Odometry, '/{}/odom'.format(self.namespace), 10)

        # Get the physical params
        self.wheel_distance = 0.16
        self.wheel_radius = 0.033
        self.wheel_max_speed = min(self.left_motor.getMaxVelocity(), self.right_motor.getMaxVelocity())

    def setpoint_callback(self, twist):
        self.target_cmd_vel = twist

    def step(self):

        rclpy.spin_once(self.cf_driver, timeout_sec=0)
        self.time = self.robot.getTime()

        self.update_current_pose()
        self.publish_odometry()

        forward_speed = self.target_cmd_vel.linear.x
        angular_speed = self.target_cmd_vel.angular.z

        command_motor_left = (forward_speed - angular_speed * self.wheel_distance / 2) / self.wheel_radius
        command_motor_left = np.clip(command_motor_left,-self.wheel_max_speed, self.wheel_max_speed)

        command_motor_right = (forward_speed + angular_speed * self.wheel_distance / 2) / self.wheel_radius
        command_motor_right = np.clip(command_motor_right,-self.wheel_max_speed, self.wheel_max_speed)


        self.left_motor.setVelocity(command_motor_left)
        self.right_motor.setVelocity(command_motor_right)

        self.past_time = self.robot.getTime()
        self.past_position = self.current_pose.position