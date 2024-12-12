import numpy as np
from disropt.functions import Variable, QuadraticForm, AffineForm
from disropt.agents import Agent
from threading import Event

from .optimizer import Optimizer
from .aggregative_utils import AggregativeProblem, AggregativeGradientTracking  


class AggregativeOptimizer(Optimizer):

    def __init__(self, settings: dict=None):
        super().__init__(settings)

        self.algorithm = None
        self.agent = None
        self.n_agents = None
        self._read_settings(**settings)

    
    def _read_settings(self, 
                       agent_dim: int=None, 
                       stepsize: float=0.1,
                       gamma_intruder: float=0.1,
                       gamma_target: float=0.1,
                       gamma_shape: float=0.1,
                       max_iterations: int=1000,
                       enable_log: bool=False,
                       **kwargs):
        
        self.agent_dim = agent_dim
        self.stepsize = stepsize
        self.gamma_intruder = gamma_intruder
        self.gamma_target = gamma_target
        self.gamma_shape = gamma_shape
        self.max_iterations = max_iterations
        self.enable_log = enable_log
    
    def initialize(self, guidance: 'Guidance', halt_event: Event=None):
        super().initialize(guidance, halt_event)

        # create agent
        self.agent = Agent(in_neighbors=self.guidance.in_neighbors, out_neighbors=self.guidance.out_neighbors,
                      communicator=self.guidance.communicator, in_weights=self.guidance.weights)

    def create_problem(self):

        self.n_agents = self.guidance.n_agents
        initial_condition = self.guidance.get_initial_condition()
        intruder = self.guidance.get_intruder()
        target = self.guidance.get_target()

        self.x_sigma = Variable(2*self.agent_dim)
        self.x = Variable(self.agent_dim)

        obj = self._generate_cost(intruder, target, self.gamma_intruder, self.gamma_target, self.gamma_shape) 
        agg_fn = self._generate_aggregative_fn()

        problem = AggregativeProblem(objective_function=obj, aggregative_function=agg_fn)
        self.agent.problem = problem
        
        # create algorithm object
        if self.algorithm is None:
            self.algorithm = AggregativeGradientTracking(agent = self.agent, 
                                                        initial_condition = initial_condition,
                                                        enable_log=self.enable_log)
    
    def _generate_cost(self, intruder, target, gamma_intruder, gamma_target, gamma_shape):
        mask_x = np.hstack((np.eye(self.agent_dim),np.zeros((self.agent_dim,self.agent_dim)))).T
        mask_sigma = np.hstack((np.zeros((self.agent_dim,self.agent_dim)),np.eye(self.agent_dim))).T

        # define objective function
        fun1 = 0.0
        if isinstance(intruder, np.ndarray) and gamma_intruder:
            fun1 = QuadraticForm(mask_x @ self.x_sigma - intruder, np.eye(self.agent_dim))
        
        fun2 = 0.0
        if isinstance(target, np.ndarray) and gamma_target:
            fun2 = QuadraticForm(mask_sigma @ self.x_sigma - target, np.eye(self.agent_dim))
        
        fun3 = 0.0
        if gamma_shape:
            fun3 = QuadraticForm(mask_x @ self.x_sigma - mask_sigma @ self.x_sigma, np.eye(self.agent_dim))

        cost = gamma_intruder*fun1 + gamma_target*fun2 + gamma_shape*fun3

        # Check if cost function is defined
        if isinstance(cost, float):
            raise ValueError('Cost function is not defined')

        return cost

    def _generate_aggregative_fn(self):
        return AffineForm(self.x,np.eye(self.agent_dim))

    def optimize(self):
        # run the algorithm
        self.algorithm.run(iterations=self.max_iterations, 
                           stepsize=self.stepsize, 
                           verbose=False,
                           callback_iter=self.update_problem)

    def get_result(self):
        return self.algorithm.get_result()

    def update_problem(self):
        self.create_problem()

    def get_cost(self):
        return self.algorithm.get_cost_list()
    
    def get_subgradient(self):
        return self.algorithm.get_subgradient_list()
    
    def get_aggregative_function(self):
        return self.algorithm.get_aggregative_function_list()

    def get_trackers(self):
        return self.algorithm.get_trackers_list()

    def get_positions(self):
        return self.algorithm.get_position_list()
