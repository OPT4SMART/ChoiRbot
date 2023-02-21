from geometry_msgs.msg import Vector3
from ..guidance import Guidance
import numpy as np
from time import sleep


class DistributedControlGuidance(Guidance):

    def __init__(self, update_frequency: float, pose_handler: str=None, pose_topic: str=None, input_topic: str = 'velocity'):
        super().__init__(pose_handler, pose_topic)
        self.publisher_ = self.create_publisher(Vector3, input_topic, 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/self.update_frequency, self.control)
        self.get_logger().info('Guidance {} started'.format(self.agent_id))

    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # exchange current position with neighbors
        data = self.communicator.neighbors_exchange(self.current_pose, self.in_neighbors, self.out_neighbors, False)

        # compute input
        u = self.evaluate_input(data)

        # send input to planner/controller
        self.send_input(u)

    def send_input(self, u):
        msg = Vector3()

        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]

        self.publisher_.publish(msg)

    def evaluate_input(self, neigh_data):
        raise NotImplementedError

