from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, TwistWithCovariance, Twist, Quaternion, Vector3
import numpy as np

class Integrator(Node):

    def __init__(self, integration_freq: float, odom_freq: float=None):
        super().__init__('integrator', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # get agent id
        self.agent_id = self.get_parameter('agent_id').value
        init_pos = self.get_parameter('init_pos').value

        # create odom publisher
        self.current_pos = np.zeros(3)
        self.current_or = np.zeros(4)
        self.current_or[3] = 1
        self.current_vel = np.zeros(3)
        self.current_ang_vel = np.zeros(3)
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
        if init_pos is None:
            np.random.seed(self.agent_id)
            self.current_pos = 3*np.random.rand(3)
            self.current_pos[2] = 0.0
        else:
            self.current_pos = np.array(init_pos)
    
    def integrate_and_send_odom(self):
        self.integrate()
        self.send_odom()

    def integrate(self):
        raise NotImplementedError

    def send_odom(self):
        point = Point(x=self.current_pos[0], y=self.current_pos[1], z=self.current_pos[2])
        quat = Quaternion(x=self.current_or[0], y=self.current_or[1], z=self.current_or[2], w=self.current_or[3])
        pose = Pose(position=point, orientation=quat)
        posewc = PoseWithCovariance(pose=pose)

        vel = Vector3(x=self.current_vel[0], y=self.current_vel[1], z=self.current_vel[2])
        ang_vel = Vector3(x=self.current_ang_vel[0], y=self.current_ang_vel[1], z=self.current_ang_vel[2]) #float(self.agent_id)
        twist = Twist(linear=vel, angular=ang_vel)
        twistwc = TwistWithCovariance(twist=twist)

        msg = Odometry(pose=posewc, twist=twistwc, child_frame_id=str(self.agent_id))
        self.odom_publisher.publish(msg)