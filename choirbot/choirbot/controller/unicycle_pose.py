from rclpy.node import Node
from .. import Pose
from .controller import Controller
from geometry_msgs.msg import Vector3, Twist
import numpy as np
from std_msgs.msg import Empty
import dill
from scipy.spatial.transform import Rotation as R
from typing import Callable


class UnicyclePoseController(Controller):
    """
    This class implement a pose-following kinematic control law applicable to unicycle-type robots.
    The Lyapunov-based feedback control law is based on the following paper:
    @inproceedings{park2011smooth,
        title={A smooth control law for graceful motion of differential wheeled mobile robots in 2D environment},
        author={Park, Jong Jin and Kuipers, Benjamin},
        booktitle={2011 IEEE International Conference on Robotics and Automation},
        pages={4896--4902},
        year={2011},
        organization={IEEE}
    }

    Note: The notation used in this implementation is consistent with the one used in the paper.
    """

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable=None):
        super().__init__(pose_handler, pose_topic, pose_callback)

        self.subscription = self.create_subscription(Vector3, 'position', self.control_callback, 1)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        self.yaw_history = np.zeros(3)
        self.delta_history = np.zeros(3)

        # Control gains
        self.k1 = 1         # Gain for angular velocity 
        self.k2 = 10        # Gain for angular velocity
        self.k3_far = 0.2   # Gain for linear velocity
        self.k3_near = 0.1  # Gain for linear velocity

        # Velocity limits based on Turtlebot 3 Burger
        self.min_velocity = 0.02
        self.max_velocity = 0.1

        # tolerance for reaching the target [m]
        self.target_tollerance = 0.02 

        self.get_logger().info('UnicyclePoseController started')

    def control_callback(self, msg):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return

        target_pose = np.array([msg.x, msg.y]) 
        theta = 0.0 # desired orientation with respect to the line of sight from the robot to the target

        r = target_pose - self.current_pose.position[:2]

        r_norm = np.linalg.norm(r)

        if r_norm < 0.5:
            k3 = self.k3_far
        else:
            k3 = self.k3_near

        v = np.clip(r_norm*k3, self.min_velocity, self.max_velocity)

        _, _, yaw = R.from_quat(self.current_pose.orientation).as_euler('xyz')
        yaw = self.unwrap_angle(yaw, self.yaw_history)

        delta = - np.arctan2(r[1],r[0]) + yaw
        delta = self.unwrap_angle(delta, self.delta_history)

        omega = -(v/r_norm)*(self.k2*(delta-np.arctan(-self.k1*theta)) + (1+ self.k1/(1 + (self.k1*theta)*(self.k1*theta)))*np.sin(delta))
        
        if r_norm < self.target_tollerance:
            # print(f'Reached goal at [{self.current_pose.position[0]:.3f},{self.current_pose.position[1]:.3f}] with tollerance {self.target_tollerance:.2f} m. Stopping the robot.')
            self.send_input([0.0,0.0])
        else:
            # print(f'p=[{self.current_pose.position[0]:.3f},{self.current_pose.position[1]:.3f}]\tg=[{target_pose[0]:.2f},{target_pose[1]:.2f}]\tdelta={delta:.2f}\tyaw={yaw:.2f}\tyaw_g={theta:.2f}\tv={v:.2f}\tomega={omega:.2f}')
            self.send_input([v,omega])

    def send_input(self, u):
        msg = Twist()
        msg.linear.x = u[0]
        msg.angular.z = u[1]
        self.publisher_.publish(msg)

    def unwrap_angle(self, angle: float, angle_history: np.array):
        angle_history = np.append(angle_history[1:], angle)
        unwrapped_angle_history = np.unwrap(angle_history)
        angle_history[-1] = np.copy(unwrapped_angle_history[-1])
        return unwrapped_angle_history[-1]
