from geometry_msgs.msg import Vector3
from ..guidance import Guidance
import numpy as np


class DistributedControlGuidance(Guidance):

    def __init__(self, update_frequency: float, pos_handler: str=None, pos_topic: str=None):
        super().__init__(pos_handler, pos_topic)
        self.publisher_ = self.create_publisher(Vector3, 'velocity', 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/update_frequency, self.control)

    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # exchange current position with neighbors
        data = self.communicator.neighbors_exchange(self.current_pose.position, self.in_neighbors, self.out_neighbors, False)

        # compute input
        u = self.evaluate_velocity(data)

        # send input to planner/controller
        self.send_input(u)

    def send_input(self, u):
        msg = Vector3()

        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]

        self.publisher_.publish(msg)

    def evaluate_velocity(self, neigh_data):
        raise NotImplementedError

