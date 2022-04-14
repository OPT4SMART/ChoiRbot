from .integrator import Integrator
from geometry_msgs.msg import Twist 
from scipy.spatial.transform import Rotation as R
from scipy.constants import g, pi
import numpy as np
from numpy import cos, sin


class QuadrotorIntegrator(Integrator):

    def __init__(self, integration_freq: float, odom_freq: float=None, mass: float=0.03):
        super().__init__(integration_freq, odom_freq)

        # create input subscription
        self.u = np.zeros(4)
        self.current_vel = np.zeros(3)
        self.current_ang_vel = np.zeros(3)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.input_callback, 1)

        self.mass = mass

        self.state = np.zeros(9)
        self.state[0:3] = np.copy(self.current_pos)
        self.state[3:6] = R.from_quat(self.current_or).as_euler('xyz')

        self.get_logger().info('Integrator {} started'.format(self.agent_id))
    
    def input_callback(self, msg):
        # save received input
        self.u = np.array([msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z])

    def integrate(self):

        sol = solve_ivp(lambda t, v: self.state_dot(), [0, dt], self.state)
        self.state =  sol.y[:,-1]
        self.state[3:6] = self.wrap_angle(self.state[3:6])

        self.current_pos = np.copy(self.state[0:3])
        self.current_or = R.from_euler('xyz',self.state[3:6]).as_quat()
        self.current_vel = np.copy(self.state[6:9])
        self.current_ang_vel = np.copy(self.u[1:4])

    def state_dot(self):
        # State space representation: [x y z phi theta psi x_dot y_dot z_dot phi_dot theta_dot psi_dot]
        RR = R.from_euler('xyz', self.state[3:6]).as_matrix()
        state_dot = np.zeros(9)
        state_dot[0:3] = self.state[6:9]
        state_dot[3:6] = np.dot(self.change_matrix(),self.u[1:4])
        state_dot[6:9] = np.array([0,0,-g]) + np.dot(RR, np.array([0,0,self.u[0]]))/self.mass
        return state_dot

    def change_matrix(self):
        # convert angular velocities to euler rates
        angles = self.state[3:6]
        cphi = cos(angles[0])
        ctheta = cos(angles[1])
        sphi = sin(angles[0])
        stheta = sin(angles[1])
        ttheta = stheta/ctheta
        R = np.array([[1,sphi*ttheta,cphi*ttheta],[0,cphi,-stheta],[0,sphi/ctheta,cphi/ctheta]])
        return R

    def wrap_angle(self, val):
        return( ( val + pi) % (2 * pi ) - pi )
