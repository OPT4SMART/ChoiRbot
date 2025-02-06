import numpy as np
from numpy.linalg import norm
from ..guidance import Guidance
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
import os
import dill
from typing import Callable

class SimpleGuidance(Guidance):
    """
    This Guidance implement a simple Guidance layer to steer robots through a list of targets (positions).
    """

    def __init__(self, update_frequency: float=20, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None, input_topic = 'position'):
        super().__init__(pose_handler, pose_topic, pose_callback)
        self.publisher_ = self.create_publisher(Vector3, input_topic, 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/self.update_frequency, self.control)
        self.get_logger().info('Guidance {} started'.format(self.agent_id))

        self.targets = self.get_parameter('targets').value
        self.targets = np.array(self.targets).reshape(-1, 3)
        self.id_target = 0
        self.goal_tolerance = 0.05 # [m]
        self.target_list_completed = False

    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # compute input
        u = self.evaluate_input()

        # send input to planner/controller
        if u is not None:
            self.send_input(u)
        

    def send_input(self, u):
        msg = Vector3()

        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]

        self.publisher_.publish(msg)

    def evaluate_input(self):

        if not self.target_list_completed:

            if np.linalg.norm(self.current_pose.position[:2] - self.targets[self.id_target][:2]) <= self.goal_tolerance:
                self.get_logger().info(f'[Agent{self.agent_id}] new targets: {self.targets[self.id_target]}')
                self.id_target += 1
            
            
            if self.id_target >= len(self.targets):
                self.get_logger().info(f'[Agent{self.agent_id}] Target list completed')
                self.id_target -= 1
                self.target_list_completed = True
        
        u = self.targets[self.id_target]

        return u

