from rclpy.node import Node
from typing import Callable

from .. import Pose
from ..utils.position_getter import pose_subscribe

import dill

import numpy as np

from std_msgs.msg import ByteMultiArray
from geometry_msgs.msg import Twist

from .safety_filters import SafetyFilterStrategy

from time import time
import os
from datetime import datetime

class CollisionAvoidance(Node):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None, 
                 node_frequency: float = 100.0, topic_msg = Twist, input_topic_name: str = 'cmd_vel', safety_filter: SafetyFilterStrategy = None):
        super().__init__('collision_avoidance', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        

        # get class attributes
        self.safety_filter = safety_filter
        self.input_topic_name = input_topic_name

        # get parameters
        self.agent_id = self.get_parameter('agent_id').value
        self.node_frequency = node_frequency

        # initialize pose subscription
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose, pose_callback)

        # subscribe to closest robots location
        self.closest_robots_poses = []
        self.closest_robots_subscription = self.create_subscription(ByteMultiArray, f'closest_robots', self.closest_robots_callback, 10)

        # subscribe to desired input
        self.desired_input = None
        self.subscriber_desired_input = self.create_subscription(topic_msg, f'{self.input_topic_name}_des', self.get_desired_input, 1)

        # compute safe input
        self.timer_compute_safe_input = self.create_timer(1.0/self.node_frequency, self.compute_safe_input)

        # publish safe input
        self.publisher_safe_input = self.create_publisher(topic_msg, self.input_topic_name, 1)


    def get_state(self):
        return self.current_pose.tolist()
    
    def get_obstacles(self):
        return self.closest_robots_poses.tolist()
    
    def get_desired_input(self, msg):
        # This function needs to be implemented in the derived class
        raise NotImplementedError
    
    def compute_safe_input(self):
        # compute safe input

        if self.current_pose.position is None:
            return

        if self.desired_input is None:
            return
    
        safe_input = self.safety_filter.filter_desired_input(
            state = self.get_state(),
            desired_input = self.desired_input, 
            obstacles = self.get_obstacles()
        )

        if safe_input is None:
            print("safe input is None")
            self.send_safe_input(self.desired_input)

        # check if each element of the input is a float
        if not all(isinstance(u_i, (int, float)) for u_i in safe_input):
            raise ValueError(f"safe input {safe_input} must be a list of floats")

        self.send_safe_input(safe_input)

    
    def send_safe_input(self, safe_input):
        # This function needs to be implemented in the derived class
        raise NotImplementedError
    
    def closest_robots_callback(self, msg):
        # build up full byte string
        data = bytes(map(lambda x: x[0], msg.data))

        # decode message
        received_closest_robots_data = dill.loads(data)

        # get closest robots poses from received data
        self.closest_robots_poses = received_closest_robots_data.values()
