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
        # save new input
        self.v = msg.linear.x
        self.omega = msg.angular.z
       #  self.get_logger().info('New robot input is {} {}'.format(self.v, self.omega))

    def integrate(self):
        print(self.current_pos[0])
        self.current_pos[0] += self.samp_time * self.v*np.cos(self.theta)
        print(self.current_pos[0])
        self.current_pos[1] += self.samp_time * self.v*np.sin(self.theta)
        self.theta += self.samp_time * self.omega

    def send_odom(self):
        point = Point(x=self.current_pos[0], y=self.current_pos[1], z=self.current_pos[2])
        quat_ = R.from_euler('z', self.theta).as_quat()
        quat = Quaternion(x=quat_[0], y=quat_[1], z=quat_[2], w=quat_[3])
        pose = Pose(position=point, orientation=quat)
        posewc = PoseWithCovariance(pose=pose)
        msg = Odometry(pose=posewc)
        self.odom_publisher.publish(msg)

        