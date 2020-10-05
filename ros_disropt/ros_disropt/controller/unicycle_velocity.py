from rclpy.node import Node
from .. import Pose
from .position_getter import pose_subscribe
from geometry_msgs.msg import Vector3, Twist


class UnicycleVelocityController(Controller):

    def __init__(self, pos_handler: str=None, pos_topic: str=None):
        super().__init__(pos_handler, pos_topic)
        self.subscription = self.create_subscription(Vector3, 'velocity', self.control_callback, 1)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

    def control_callback(self, msg):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return

        u_ = np.array([msg.x, msg.y, msg.z])     
        yaw = self.get_yaw()
        a = np.cos(yaw)
        b = np.sin(yaw)
        u = np.zeros(2)
        u[0] = 1.0*(a*u_[0] + b*u_[1])
        u[1] = (np.pi)*np.arctan2(-b*u_[0] + a*u_[1], dxu[0])/(np.pi/2)
        self.send_input(u)

    def send_input(self, u):
        msg = Twist()
        msg.linear.x = u[0]
        msg.angular.z = u[1]
        self.publisher_.publish(msg)

    def get_yaw(self):
        quat = np.copy(self.current_pose.orientation)
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]
        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
