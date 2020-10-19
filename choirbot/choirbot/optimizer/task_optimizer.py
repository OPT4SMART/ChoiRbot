import numpy as np
from disropt.algorithms import DistributedSimplex
from disropt.functions import Variable
from disropt.problems import LinearProblem
from disropt.agents import Agent
from threading import Event

from .optimizer import Optimizer


class TaskOptimizer(Optimizer):

    def __init__(self, resolution_strategy: str='simplex', cost_function: str='euclidean', settings: dict=None):
        super().__init__(settings)

        if resolution_strategy is not None and not isinstance(resolution_strategy, str): # FIXME: put a list of possible choices
            raise TypeError("resolution_strategy must be a string")

        if cost_function is not None and not isinstance(cost_function, str): # FIXME: idem (cannot be None)
            raise TypeError("cost_function must be a string")
        
        self.resolution_strategy = resolution_strategy
        self.cost_function = cost_function
        self.algorithm = None
        self.halt_optimization = False
        self.task_list = None
        self.n_tasks = None
        self.agent = None
        self._read_settings(**settings)
    
    def initialize(self, guidance: 'Guidance', halt_event: Event=None):
        super().initialize(guidance, halt_event)

        # create agent
        self.agent = Agent(in_neighbors=self.guidance.in_neighbors, out_neighbors=self.guidance.out_neighbors,
                      communicator=self.guidance.communicator)
    
    def _read_settings(self, stop_iterations: int=None, max_iterations: int=1000, **kwargs):
        self.stop_iterations = stop_iterations
        self.max_iterations = max_iterations
    
    def create_problem(self, task_list):
        # prepare problem data (TODO spostare calcolo di task_positions in classe funzione di costo)
        self.task_list = task_list
        self.n_tasks = self.guidance.n_agents
        task_positions = np.array([np.array(t.coordinates) for t in task_list.tasks])
        task_indices = [t.id for t in task_list.tasks]
        starting_position = self.guidance.current_pose.position[:-1]

        # create problem matrices
        c = self._generate_cost(task_positions, starting_position)
        A, b = self._generate_constraints(task_indices)

        # create problem object
        x = Variable(len(self.task_list.tasks))
        obj = c @ x
        constr = A.transpose() @ x == b
        problem = LinearProblem(objective_function=obj, constraints=constr)
        self.agent.problem = problem
        
        # set communicator label
        self.guidance.communicator.current_label = int(task_list.label)

        # create algorithm object
        self.algorithm = DistributedSimplex(self.agent, stop_iterations=self.stop_iterations)

    def _generate_cost(self, task_positions, starting_position):
        if self.cost_function == 'euclidean':
            cost_vector = np.empty((len(self.task_list.tasks), 1))
            for idx, row in enumerate(task_positions):
                cost_vector[idx, :] = np.linalg.norm(row - starting_position)

        return cost_vector

    def _generate_constraints(self, task_indices):
        # TODO compute A as [A_1; A_2], where A_1 has a row of ones and A_2 is the identity (check column order)
        N = self.guidance.n_agents
        A = np.zeros((2*N, len(self.task_list.tasks)))
        b = np.ones((2*N, 1))

        for idx, t in enumerate(task_indices):
            A[self.guidance.agent_id, idx] = 1
            A[self.n_tasks + t, idx] = 1

        return A, b

    def optimize(self):
        self.algorithm.run(self.max_iterations, event=self._halt_event)

    def get_result(self):
        x_basic = self.algorithm.x_basic
        basis = self.algorithm.B

        # cycle over basis columns where x_basic is nonzero
        nonzero_basic = np.nonzero(x_basic)[0]
        for idx in nonzero_basic:

            # extract column
            col = basis[1:, idx]

            # check if this is the agent's assignment column
            if col[self.guidance.agent_id]:

                # get task id
                task_id = np.nonzero(col)[0][-1] - self.n_tasks 

                # return list with the task (there is only one because we assume N=M)
                return [next(t for t in self.task_list.tasks if t.id == task_id)]

    def get_cost(self):
        return self.algorithm.J