import numpy as np
from disropt.algorithms.constraintexchange import DistributedSimplex
from disropt.functions import Variable
from disropt.problems import LinearProblem
from disropt.agents import Agent
from mpi4py import MPI
from disropt.utils.graph_constructor import ring_graph
import time


class MPITaskOptimizer():
    def __init__(self, robot_id: int, network_size: int, task_list: np.ndarray, task_indices: list, starting_point: np.ndarray, in_neighbors: list=None, out_neighbors: list=None, 
                 resolution_strategy: str=None, cost_function: str='euclidean', stop_iterations=None,  **kwargs):
        self.robot_id = robot_id
        self.network_size = network_size
        self.in_neighbors = in_neighbors
        self.out_neighbors = out_neighbors
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

        if self.cost_function is 'euclidean':
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
        agent = Agent(in_neighbors=self.in_neighbors, out_neighbors=self.out_neighbors)
        agent.problem = problem

        # instantiate the algorithm
        self.algorithm = DistributedSimplex(agent, stop_iterations=self.stop_iterations)

    def optimize(self, iterations):
        self.algorithm.run(iterations)

    def retrieve_results(self):
        x_basic = self.algorithm.x_basic
        basis = self.algorithm.B
        nonzero_basic = np.nonzero(x_basic)[0]
        for idx in nonzero_basic:
            col = basis[1:, idx]
            if np.equal(col[self.robot_id], [1]):
                nonzero_col = np.nonzero(col)[0]
                return self.task_list[nonzero_col[-1]-self.task_number, :], nonzero_col[-1]-self.task_number


if __name__ == "__main__":
    # get MPI info
    comm = MPI.COMM_WORLD
    N = comm.Get_size()
    id = comm.Get_rank()

    # Generate a ring graph (for which the diameter is nproc-1)
    Adj = ring_graph(N)
    neigh = np.nonzero(Adj[id, :])[0].tolist()

    task_pos = np.array([[1.5, 2.1],[3.4, 4], [5.2, 6]])
    starting = np.array([[0, 0],[1, 1], [2, 2]])
    tasklist = [[0,1,2],[0,1],[2]]

    task_opt = MPITaskOptimizer(id, N, task_pos, tasklist[id], starting[id,:], in_neighbors=neigh)
    ################
    task_opt.instantiate_algorithm()
    time.sleep(1)

    print("Agent {} A,b are {}".format(id, task_opt.problemConstraints()))
    print('Agent {} starting'.format(id))

    task_opt.optimize(iterations=100)
    print('Agente {} has basis {}'.format(id, task_opt.algorithm.B))
    print('Agent {} will do task {}'.format(id, task_opt.retrieve_results()))