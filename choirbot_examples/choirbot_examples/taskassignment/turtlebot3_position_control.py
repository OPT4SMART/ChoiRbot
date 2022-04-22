#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert

import math
import numpy

from geometry_msgs.msg import Twist, Point
from rclpy.node import Node
from rclpy.qos import QoSProfile
from choirbot.utils.position_getter import pose_subscribe
from choirbot import Pose


class Turtlebot3Path():

    def turn(angle, angular_velocity, step):
        twist = Twist()

        if math.fabs(angle) > 0.01:  # 0.01 is small enough value
            if angle >= math.pi:
                twist.angular.z = -angular_velocity
            elif math.pi > angle and angle >= 0:
                twist.angular.z = angular_velocity
            elif 0 > angle and angle >= -math.pi:
                twist.angular.z = -angular_velocity
            elif angle > -math.pi:
                twist.angular.z = angular_velocity
        else:
            step += 1

        return twist, step

    def go_straight(distance, linear_velocity, step):
        twist = Twist()

        if distance > 0.01:  # 0.01 is small enough value
            twist.linear.x = linear_velocity
        else:
            step += 1

        return twist, step


class Turtlebot3PositionControl(Node):

    def __init__(self, robot_id, pos_handler: str=None, pos_topic: str=None):
        super().__init__('agent_{}_turtlebot3_position_control'.format(robot_id))

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.current_pose = Pose(None, None, None, None)
        self.goal_pose_x = None
        self.goal_pose_y = None
        self.goal_pose_theta = None
        self.step = 1
        self.init_odom_state = False  # To get the initial pose at the beginning
        self.robot_id = robot_id
        self.goal_point = None
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/agent_{}/cmd_vel'.format(robot_id), qos)

        # Initialise subscribers
        self.position_sub = pose_subscribe(pos_handler, pos_topic, self, self.current_pose, self.pose_callback)

        self.goal_sub = self.create_subscription(
            Point,
            '/agent_{}/goal'.format(self.robot_id),
            self.goal_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(0.010, self.update_callback)  # unit: s

        self.get_logger().info("Turtlebot3 position control node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def pose_callback(self):
        self.last_pose_x = self.current_pose.position[0]
        self.last_pose_y = self.current_pose.position[1]
        _, _, self.last_pose_theta = self.euler_from_quaternion(self.current_pose.orientation)

        self.init_odom_state = True

    def update_callback(self):
        if self.init_odom_state is True:
            self.generate_path()

    def generate_path(self):
        twist = Twist()

        if self.goal_pose_x is not None:
            # Step 1: Turn
            if self.step == 1:
                path_theta = math.atan2(
                    self.goal_pose_y-self.last_pose_y,
                    self.goal_pose_x-self.last_pose_x)
                angle = path_theta - self.last_pose_theta
                angular_velocity = 0.1  # unit: rad/s

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            # Step 2: Go Straight
            elif self.step == 2:
                distance = math.sqrt(
                    (self.goal_pose_x-self.last_pose_x)**2
                    + (self.goal_pose_y-self.last_pose_y)**2)
                linear_velocity = 0.1  # unit: m/s

                twist, self.step = Turtlebot3Path.go_straight(distance, linear_velocity, self.step)

            # Step 3: Turn
            elif self.step == 3:
                angle = self.goal_pose_theta - self.last_pose_theta
                angular_velocity = 0.1  # unit: rad/s

                twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

            # Reset
            elif self.step == 4:
                self.step = 1
                self.goal_pose_x = False
                self.goal_pose_y = False
                self.goal_pose_theta = False

            self.cmd_vel_pub.publish(twist)

    def goal_callback(self, msg):
        # Print terminal message and get inputs
        self.goal_pose_x = msg.x
        self.goal_pose_y = msg.y
        self.goal_pose_theta = 0.0

    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
