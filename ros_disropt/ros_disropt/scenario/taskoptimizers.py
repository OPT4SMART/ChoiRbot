from .scenarios import Optimizer
import numpy as np
from disropt.algorithms import DistributedSimplex
from disropt.functions import Variable
from disropt.constraints import Box
from disropt.problems import LinearProblem
from disropt.agents import Agent
from threading import Event
from scipy.spatial import distance_matrix

# class PDVRPOptimizer(Optimizer):
#     def __init__(self, robot_id: int, network_size: int, task_pos: np.ndarray, task_indices: list, task_demand: np.ndarray,
#                  n_tot_tasks: int, service_times: np.ndarray, starting_point: np.ndarray, vehicle_speed: float, capacity: float,
#                  epsilon: float=0.9, use_runavg: bool=True, initial_load: float=0.0,
#                  pending_tasks: list = [], pickup_delivery_pairs: dict = {},
#                  in_neighbors: list=None, out_neighbors: list=None,
#                  resolution_strategy: str=None, cost_function: str='euclidean', event: Event=None, **kwargs):

# def retrieve_cost(self):
#         solution, _, _ = self.algorithm.get_result()
#         return (self.cost_vector.T @ solution).flatten()

class TaskOptimizer(Optimizer):
    def __init__(self, robot_id: int, network_size: int, task_list: np.ndarray, task_indices: list, starting_point: np.ndarray, in_neighbors: list=None, out_neighbors: list=None, 
                 resolution_strategy: str=None, cost_function: str='euclidean', stop_iterations=None, event: Event=None, **kwargs):
        super(TaskOptimizer, self).__init__(robot_id, network_size, in_neighbors, out_neighbors, resolution_strategy, event)
        if cost_function is not None:
            if not isinstance(cost_function,str):
                raise TypeError("cost_function must be a string ")
        self.cost_function = cost_function
        if not isinstance(starting_point, np.ndarray):
            raise TypeError("starting_point must be a numpy array")
        self.starting_point = starting_point
        self.set_taskList(task_list, task_indices)
        self.halt_optimization = False
        self.stop_iterations = stop_iterations

    def set_taskList(self, task_list: np.ndarray, task_indices: list):
        """tasklist setter
        Args:
            task_list (numpy.ndarray): :math:`M \\times d` matrix of task positions, M is the number of tasks
        """
        if not isinstance(task_list, np.ndarray):
            raise TypeError("task_list must be a numpy array")
        if not isinstance(task_indices, list):
            raise TypeError("task_indices must be a list")

        self.task_list = task_list
        self.task_indices = task_indices
        self.task_number = self.network_size

    def costFunction(self):
        if self.starting_point.shape[0] is not self.task_list.shape[1]:
            raise TypeError("starting_point and task sizes must coincide")

        if self.cost_function == 'euclidean':
            cost_vector = np.empty((len(self.task_indices), 1))
            for idx, row in enumerate(self.task_list[self.task_indices, :]):
                cost_vector[idx, :] = np.linalg.norm(row-self.starting_point)

        return cost_vector

    def problemConstraints(self):
        A = np.zeros((2*self.network_size, len(self.task_indices)))
        b = np.ones((2*self.network_size, 1))

        for idx, t in enumerate(self.task_indices):
            A[self.robot_id, idx] = 1
            A[self.task_number + t, idx] = 1

        return A, b

    def instantiate_algorithm(self):
        c = self.costFunction()
        A, b = self.problemConstraints()
        # define the local problem data
        x = Variable(len(self.task_indices))

        obj = c @ x
        constr = A.transpose() @ x == b
        problem = LinearProblem(objective_function=obj, constraints=constr)

        # create agent
        agent = Agent(in_neighbors=self.in_neighbors, out_neighbors=self.out_neighbors,
                      communicator=self.communicator)
        agent.problem = problem

        # instantiate the algorithm
        self.algorithm = DistributedSimplex(agent, stop_iterations=self.stop_iterations)

    def optimize(self, iterations):
        self.algorithm.run(iterations, event=self.event)

    def retrieve_results(self):
        x_basic = self.algorithm.x_basic
        basis = self.algorithm.B
        nonzero_basic = np.nonzero(x_basic)[0]
        for idx in nonzero_basic:
            col = basis[1:, idx]
            if np.equal(col[self.robot_id], [1]):
                nonzero_col = np.nonzero(col)[0]
                return self.task_list[nonzero_col[-1]-self.task_number, :], nonzero_col[-1]-self.task_number
