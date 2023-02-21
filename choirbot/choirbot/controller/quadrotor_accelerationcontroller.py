import numpy as np
from geometry_msgs.msg import Vector3
from scipy.constants import g
from .quadrotor_controller import QuadrotorController

class QuadrotorAccelerationController(QuadrotorController):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, update_frequency = 100.0, mass: float=0.03):
        super().__init__(pose_handler, pose_topic, update_frequency, mass)

        self.subscriber = self.create_subscription(Vector3, 'acceleration', self.set_des_acceleration, 1)

    def thrust_dir(self):
        # compute desired thrust vector
        e_3 = np.array([0.0, 0.0, 1.0])
        F_des = self.mass*g*e_3 + self.mass*self.desired_acc
        return F_des

    def set_des_acceleration(self, msg):
        self.desired_acc =  np.array([msg.x, msg.y, msg.z])