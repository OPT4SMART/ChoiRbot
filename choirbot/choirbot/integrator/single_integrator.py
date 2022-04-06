from .integrator import Integrator
from geometry_msgs.msg import Vector3
import numpy as np


class SingleIntegrator(Integrator):

    def __init__(self, integration_freq: float, odom_freq: float=None):
        super().__init__(integration_freq, odom_freq)

        # create input subscription
        self.u = np.zeros(3)
        self.subscription = self.create_subscription(Vector3, 'velocity', self.input_callback, 1)
        
        self.get_logger().info('Integrator {} started'.format(self.agent_id))
    
    def input_callback(self, msg):
        # save new input
        self.u = np.array([msg.x, msg.y, msg.z])

    def integrate(self):
        self.current_pos += self.samp_time * self.u