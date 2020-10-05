from .integrator import Integrator
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R


class SingleIntegrator(Integrator):

    def __init__(self, integration_freq: float, odom_freq: float=None, initial_position: np.ndarray=None, initial_attitude: float=0.0):
        super().__init__(integration_freq, odom_freq, initial_position)

        # create input subscription
        self.u = np.zeros(3)
        self.subscription = self.create_subscription(Vector3, 'unicycle_velocity', self.input_callback, 1)
        
        self.get_logger().info('Integrator {} started'.format(self.agent_id))
        self.theta = initial_attitude
    
    def input_callback(self, msg):
        # save new input
        self.v = msg.linear.x
        self.omega = msg.angular.z
        self.get_logger().info('New robot input is {}'.format(self.u))

    def integrate(self):
        self.current_pos[0] += self.samp_time * self.v*cos(self.theta)
        self.current_pos[1] += self.samp_time * self.v*sin(self.theta)
        self.theta += self.samp_time * self.omega

    def send_odom(self):
        point = Point(x=self.current_pos[0], y=self.current_pos[1], z=self.current_pos[2])
        quat_ = R.from_euler('z', self.theta).as_quat()
        quat = Quaternion(x=quat_[0], y=quat_[1], z=quat_[2], w=quat_[3])
        pose = Pose(position=point, orientation=quat)
        posewc = PoseWithCovariance(pose=pose)
        msg = Odometry(pose=posewc)
        self.odom_publisher.publish(msg)

        