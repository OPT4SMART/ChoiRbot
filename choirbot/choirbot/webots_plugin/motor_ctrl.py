import rclpy
from rclpy.time import Time
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R
import numpy as np

from .. import Pose

class MotorCtrl:
    def init(self, webots_node, properties):
        
        # Declare the robot name and fix the timestep
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.robot_name = self.robot.getName()

        # Initialize webots driver node
        rclpy.init(args=None)
        self.namespace = str(self.robot_name)
        self.cf_driver = rclpy.create_node(
                            'cf_driver',
                            namespace=self.namespace,
                            allow_undeclared_parameters=True,
                            automatically_declare_parameters_from_overrides=True)
        
        ## Initialize motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')

        self.left_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)

        self.right_motor.setPosition(float('inf'))
        self.right_motor.setVelocity(0)

        ## Initialize Sensors
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        ## Intialize Variables
        self.current_pose = Pose(None, None, None, None)
        self.past_position = np.zeros(3)
        self.past_time = self.robot.getTime()

    def step(self):
        raise NotImplementedError

    def update_current_pose(self):
        ## Get measurements
        self.current_rpy = np.array(self.imu.getRollPitchYaw())
        self.current_pose.orientation = np.array(R.from_euler('xyz', self.current_rpy).as_quat())
        self.current_pose.angular = np.array(self.gyro.getValues())
        self.current_pose.position = np.array(self.gps.getValues())

        # TODO use velocity_low_pass
        dt = self.robot.getTime() - self.past_time
        self.current_pose.velocity = (self.current_pose.position - self.past_position)/dt

    def publish_odometry(self):
        odom = Odometry()
        odom.header.stamp = Time(seconds=self.robot.getTime()).to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.current_pose.position[0]
        odom.pose.pose.position.y = self.current_pose.position[1]
        odom.pose.pose.position.z = self.current_pose.position[2]

        odom.twist.twist.linear.x = self.current_pose.velocity[0]
        odom.twist.twist.linear.y = self.current_pose.velocity[1]
        odom.twist.twist.linear.z = self.current_pose.velocity[2]

        odom.pose.pose.orientation.x = self.current_pose.orientation[0]
        odom.pose.pose.orientation.y = self.current_pose.orientation[1]
        odom.pose.pose.orientation.z = self.current_pose.orientation[2]
        odom.pose.pose.orientation.w = self.current_pose.orientation[3]

        odom.twist.twist.angular.x = self.current_pose.angular[0]
        odom.twist.twist.angular.y = self.current_pose.angular[1]
        odom.twist.twist.angular.z = self.current_pose.angular[2]
        self.odom_publisher.publish(odom)
