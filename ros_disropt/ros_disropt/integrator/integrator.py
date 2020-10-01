from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
import numpy as np

class Integrator(Node):

    def __init__(self, steptime: float):
        super().__init__('integrator', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        self.agent_id = self.get_parameter('agent_id').value
        self.steptime = steptime
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.odom_timer = self.create_timer(steptime, self.integrate)
        self.get_logger().info('Integrator {} started'.format(self.agent_id))

    def execute_callback(self, msg):
        self.u = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info('Robot new input is {}'.format(self.u))

    def integrate(self):
        pass

    def send_odom(self):
        point = Point(x=self.current_pos[0], y=self.current_pos[1], z=self.current_pos[2])
        pose = Pose(position=point)
        posewc = PoseWithCovariance(pose=pose)
        msg = Odometry(pose=posewc)
        self.odom_publisher.publish(msg)