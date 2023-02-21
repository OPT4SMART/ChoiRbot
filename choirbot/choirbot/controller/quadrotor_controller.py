import numpy as np
from numpy import cos, sin
from numpy.linalg import norm
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as R
from scipy.constants import g
from .controller import Controller

class QuadrotorController(Controller):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, update_frequency: float = 100.0, mass: float=0.03):
        super().__init__(pose_handler, pose_topic)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.K_p = np.array([0.2, 0.2, 1.0])
        self.K_v = np.array([0.3, 0.3, 0.8])
        K_r_z = 10**(-4)
        K_w_z = 10**(-4)
        K_r_x = 80*10**(-4)
        K_w_x = 8*10**(-4)
        self.K_r = np.array([K_r_x, K_r_x, K_r_z])
        self.K_w = np.array([K_w_x, K_w_x, K_w_z])
        
        self.desired_pos = np.zeros(3)
        self.desired_or =  np.zeros(3)
        self.desired_vel =  np.zeros(3)
        self.desired_acc =  np.zeros(3)

        self.mass = 0.03
        init_pos = self.get_parameter('init_pos').value
        if init_pos is None:
            self.desired_pos = np.zeros(3)
        else:
            self.desired_pos = np.array(init_pos)

        self.update_frequency = update_frequency
        self.ctrl_timer = self.create_timer(1.0/self.update_frequency, self.controller)

    def controller(self):
        if self.current_pose.position is not None:
            # implement flatness based control scheme
            u = np.zeros(4)
            # compute desired thrust vector
            F_des = self.thrust_dir()
            # compute thrust
            RR = R.from_quat(self.current_pose.orientation).as_matrix()
            e_3 = np.array([0,0,1])
            u[0] = np.dot(np.dot(RR,F_des),e_3)
            # compute desired attitude via geometric control
            z_b_des = F_des/norm(F_des)
            x_c_des = np.array([cos(self.desired_or[2]),sin(self.desired_or[2]),0]).transpose()
            y_b_des = np.cross(z_b_des, x_c_des)/norm(np.cross(z_b_des, x_c_des))
            x_b_des = np.cross(y_b_des,z_b_des)
            R_des = np.array([x_b_des, y_b_des, z_b_des]).transpose()
            # compute input angular velocity
            # error on rotation matrix
            error = 1/2*(np.dot(R_des.transpose(),RR) - (np.dot(RR.transpose(),R_des)))
            e_r = np.array([error[2,1], error[0,2], error[1,0]])
            # dumping angular velocity
            e_w = self.current_pose.angular
            # angular velocity inputs
            u[1:4] = -self.K_r*e_r -self.K_w*e_w

            self.send_input(u)

    def send_input(self, u):
        msg = Twist()
        msg.linear.z  = u[0]
        msg.angular.x = u[1]
        msg.angular.y = u[2]
        msg.angular.z = u[3]
        self.publisher_.publish(msg)

    def thrust_dir(self):
        # compute desired thrust vector
        e_3 = np.array([0,0,1])
        e_p =   self.current_pose.position - self.desired_pos
        e_v =   self.current_pose.velocity - self.desired_vel
        F_des = -self.K_p*e_p - self.K_v*e_v + self.mass*g*e_3 + self.mass*self.desired_acc
