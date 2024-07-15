import numpy as np

class SafetyFilterStrategy:

    def __init__(self):
        pass
 
    def filter_desired_input(self, state: list, desired_input: list, obstacles: list):

        self.state_dim = len(state)
        self.state = np.array(state).reshape((self.state_dim, 1))

        self.input_dim = len(desired_input)
        self.desired_input = np.array(desired_input).reshape((self.input_dim, 1))

        self.obstacles = [np.array(obstacle).reshape((self.state_dim, 1)) for obstacle in obstacles]

        safe_input = self.compute_safe_input()

        return safe_input


    def compute_safe_input(self):
        '''
        This function computes the safe input for a robot given its state, desired input, obstacles and safe distance

        Args:
        - state (list): state of the robot
        - desired_input (list): desired input for the robot
        - obstacles (list): list of obstacles locations, usually closest robots
        '''
        raise NotImplementedError
    