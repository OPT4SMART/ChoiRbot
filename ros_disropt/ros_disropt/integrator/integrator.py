from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
import numpy as np

class Integrator(Node):

    def __init__(self, integration_freq: float, odom_freq: float=None, initial_position: np.ndarray=None):
        super().__init__('integrator', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # get agent id
        self.agent_id = self.get_parameter('agent_id').value

        # create odom publisher
        self.current_pos = np.zeros(3)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # create integration/odometry timer(s)
        self.samp_time = 1.0/integration_freq

        if odom_freq is None or integration_freq == odom_freq:
            self.int_timer = self.create_timer(self.samp_time, self.integrate_and_send_odom)
        else:
            self.odom_samp_time = 1.0/odom_freq
            self.int_timer = self.create_timer(self.samp_time, self.integrate)
            self.odom_timer = self.create_timer(self.odom_samp_time, self.send_odom)

        # get initial position or generate a new one with x,y in [0,3] and z=0
        if initial_position is None:
            np.random.seed(self.agent_id)
            self.current_pos = 3*np.random.rand(3)
            self.current_pos[2] = 0.0
        else:
            self.current_pos = initial_position.copy()
    
    def integrate_and_send_odom(self):
        self.integrate()
        self.send_odom()

    def integrate(self):
        raise NotImplementedError

    def send_odom(self):
        point = Point(x=self.current_pos[0], y=self.current_pos[1], z=self.current_pos[2])
        pose = Pose(position=point)
        posewc = PoseWithCovariance(pose=pose)
        msg = Odometry(pose=posewc)
        self.odom_publisher.publish(msg)