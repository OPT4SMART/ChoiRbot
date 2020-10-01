from .integrator import Integrator
import numpy as np


class SingleIntegrator(Integrator):

    def __init__(self, steptime: float, initial_position: np.ndarray=None):
        super().__init__(steptime)
        self.subscription = self.create_subscription(Point, 'velocity', self.execute_callback, 1)

        if initial_position is None:
            np.random.seed(self.agent_id)
            self.current_pos = 3*np.random.rand(3)
            self.current_pos[2] = 0.0
        else:
            self.current_pos = initial_position
        self.u = np.zeros(3)

    def integrate(self):
        self.current_pos += self.steptime * self.u
        self.send_odom()