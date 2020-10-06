import numpy as np
from typing import Dict
from .optimizer import Optimizer


class MPCOptimizer(Optimizer):

    def __init__(self):
        super().__init__(None)
    
    def initialize_scenario(self, prediction_horizon: int, system_matrices: dict, cost_matrices: dict,
        local_constraints: dict=None, coupling_constraints: dict=None):
        raise NotImplementedError
    
    def create_opt_control_problem(self, initial_condition: np.ndarray, output_trajectories: Dict[int, np.ndarray]):
        raise NotImplementedError

    def optimize(self):
        raise NotImplementedError

    def get_result(self):
        raise NotImplementedError
    
    def get_cost(self):
        raise NotImplementedError