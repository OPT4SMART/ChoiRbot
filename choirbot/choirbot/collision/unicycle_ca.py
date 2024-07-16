from typing import Callable
from geometry_msgs.msg import Twist

from .collision_avoidance import CollisionAvoidance

from .safety_filters import SafetyFilterStrategy

from scipy.spatial.transform import Rotation as R
import numpy as np

import dill
import os

class UnicycleCollisionAvoidance(CollisionAvoidance):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None, 
                 node_frequency: float = 10.0, topic_msg = Twist, topic_name: str = 'cmd_vel', safety_filter: SafetyFilterStrategy = None):
        super().__init__(pose_handler, pose_topic, pose_callback, node_frequency, topic_msg, topic_name, safety_filter)
        '''
        Mapping Single Integrator Dynamics to Unicycle Control Commands

        # Unicycle 
        - state
        x = [px, py, theta].T
        xp = [px, py].T

        - dynamics
                [cos(theta), 0 ]
        x_dot = |sin(theta), 0 | * [v, omega].T
                [   0        1 ]

        # Single Integrator Mapping
        - state
        s(x) = [px, py].T + l * [cos(theta), sin(theta)].T
        with l>0.

        - dynamics
        s_dot(x) = R(l,theta) * [v, omega].T

        with R(l,theta) = [ cos(theta), -l*sin(theta) ]
                          [ sin(theta),  l*cos(theta) ]
           
        # Input Mapping
        Considering the S.I. dynamic s_dot(x) = u

        - Unicycle
        [v, omega].T = R(l,theta)^-1 * u
        with R(l,theta)^-1 = [ cos(theta), sin(theta) ]
                             [ -sin(theta), cos(theta) ] / l

        Namely,

        v = cos(theta) * u[0] + sin(theta) * u[1]
        omega = (-sin(theta) * u[0] + cos(theta) * u[1])/ l                          

        '''
        self.l = 0.1
    
    def get_state(self):

        # Unicycle
        px, py = self.current_pose.position[:2]
        _, _, theta = R.from_quat(self.current_pose.orientation).as_euler('xyz')

        # Single Integrator Mapping
        s_x = px + self.l * np.cos(theta)
        s_y = py + self.l * np.sin(theta)
        return [s_x, s_y]
    
    def get_obstacles(self):
        return [pose.position[:2] for pose in self.closest_robots_poses]

    def get_desired_input(self, msg):

        if self.current_pose.position is None:
            return
        
        # Unicycle Desierd input
        v, omega = msg.linear.x, msg.angular.z

        # Single Integrator Mapping
        _, _, theta = R.from_quat(self.current_pose.orientation).as_euler('xyz')

        u_des_0 = np.cos(theta) * v - self.l * np.sin(theta) * omega
        u_des_1 = np.sin(theta) * v + self.l * np.cos(theta) * omega

        self.desired_input = [u_des_0, u_des_1]
        
    def send_safe_input(self, safe_input):

        # Single Integrator input
        u = safe_input

        # Unicycle Input Mapping
        _, _, theta = R.from_quat(self.current_pose.orientation).as_euler('xyz')

        v = np.cos(theta) * u[0] + np.sin(theta) * u[1]
        omega = (-np.sin(theta) * u[0] + np.cos(theta) * u[1])/ self.l

        safe_input_msg = Twist()
        safe_input_msg.linear.x = v
        safe_input_msg.angular.z = omega
        self.publisher_safe_input.publish(safe_input_msg)

    