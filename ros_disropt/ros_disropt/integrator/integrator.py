from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
import numpy as np

class Integrator(Node):

    def __init__(self, agent_id: int, steptime: float):
        super().__init__('agent_{}_planner'.format(agent_id))
        self.agent_id = agent_id
        self.get_logger().info('Planner {} started'.format(agent_id))
        self.steptime = steptime
        self.subscription = self.create_subscription(Point, 'agent_{}_velocity'.format(agent_id), self.execute_callback, 1)
        self.odom_publisher = self.create_publisher(Odometry, '/agent_{}/odom'.format(agent_id), 10)
        self.odom_timer = self.create_timer(steptime, self.integrate)

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