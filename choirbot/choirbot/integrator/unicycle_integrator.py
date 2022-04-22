from .integrator import Integrator
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R


class UnicycleIntegrator(Integrator):

    def __init__(self, integration_freq: float, odom_freq: float=None):
        super().__init__(integration_freq, odom_freq)

        # create input subscription
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.input_callback, 1)
        
        self.get_logger().info('Integrator {} started'.format(self.agent_id))
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
    
    def input_callback(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def integrate(self):
        self.current_pos[0] += self.samp_time * self.v*np.cos(self.theta)
        self.current_pos[1] += self.samp_time * self.v*np.sin(self.theta)
        self.theta += self.samp_time * self.omega
        self.current_or = R.from_euler('z', self.theta).as_quat()

        