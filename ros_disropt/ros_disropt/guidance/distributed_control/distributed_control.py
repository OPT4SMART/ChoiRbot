from geometry_msgs.msg import Vector3
from ..guidance import Guidance
import numpy as np


class DistributedControlGuidance(Guidance):

    def __init__(self, update_frequency: float, pos_handler: str=None, pos_topic: str=None, is_unicycle: bool=False):
        super().__init__(pos_handler, pos_topic)
        self.publisher_ = self.create_publisher(Vector3, 'velocity', 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/update_frequency, self.control)
        self.is_unicycle = is_unicycle

    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # exchange current position with neighbors
        data = self.communicator.neighbors_exchange(self.current_pose.position, self.in_neighbors, self.out_neighbors, False)

        # compute input
        u = self.evaluate_velocity(data)

        if self.is_unicycle:
            u = self.to_unicycle(u)

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

    def to_unicycle(self, u_):

        quat = np.copy(self.current_pose.orientation)

        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])

        u = np.zeros(2)
        u[0] = 1.0*(a*u_[0] + b*u_[1])
        u[1] = (np.pi)*np.arctan2(-b*u_[0] + a*u_[1], dxu[0])/(np.pi/2)

        return u
