from .scenarios import Optimizer
import numpy as np
from threading import Event
import random
import time


class MockPDVRPOptimizer(Optimizer):
    def __init__(self, robot_id: int, network_size: int, task_pos: np.ndarray, task_indices: list, task_demand: np.ndarray,
                 n_tot_tasks: int, service_times: np.ndarray, starting_point: np.ndarray, vehicle_speed: float, capacity: float,
                 epsilon: float=0.9, use_runavg: bool=True, initial_load: float=0.0,
                 pending_tasks: list = [], pickup_delivery_pairs: dict = {},
                 in_neighbors: list=None, out_neighbors: list=None,
                 resolution_strategy: str=None, cost_function: str='euclidean', event: Event=None, **kwargs):
        super(MockPDVRPOptimizer, self).__init__(robot_id, network_size, in_neighbors, out_neighbors, resolution_strategy, event)

        # problem data
        if cost_function is not None:
            if not isinstance(cost_function, str):
                raise TypeError("cost_function must be a string ")
        self.cost_function = cost_function
        if not isinstance(starting_point, np.ndarray):
            raise TypeError("starting_point must be a numpy array")
        if not isinstance(n_tot_tasks, int):
            raise TypeError("n_tot_tasks must be integer")
        
        self.set_taskList(task_pos, task_indices, task_demand, capacity, pickup_delivery_pairs)
        self.shuffled_idx = None

    def set_taskList(self, task_pos: np.ndarray, task_indices: list, task_demand, capacity, pd_pairs):
        """tasklist setter
        Args:
            task_pos (numpy.ndarray): :math:`M \\times d` matrix of task positions, M is the number of tasks
        """

        # filter out tasks with unaffordable demand
        doable_tasks = [j for j in range(len(task_pos)) if abs(task_demand[j]) <= capacity]

        # prepare inverse indices
        inv_ind = {task_indices[doable_tasks[j]]:j for j in range(len(doable_tasks))}

        # save tasks
        self.task_pos = task_pos[doable_tasks]
        self.task_indices = [task_indices[j] for j in doable_tasks]
        self.loc_task_count = len(doable_tasks)
        self.task_demand = task_demand[doable_tasks]

        # save pickup/delivery pairs (with doable_tasks indexing)
        self.pd_pairs = {inv_ind[k]:inv_ind[v] for (k,v) in pd_pairs.items() if k in inv_ind and v in inv_ind}

    def instantiate_algorithm(self):
        pass

    def optimize(self, iterations=None):
        
        # Mock solution is a shuffled list of all tasks
        random.seed(self.robot_id)
        
        shuffled_idx = list(range(self.loc_task_count))
        random.shuffle(shuffled_idx)

        # prepare task order map
        task_order = {shuffled_idx[j]:j for j in range(len(shuffled_idx))}

        # adjust list so as to have pickup tasks first
        for i in shuffled_idx:

            # ensure this is a pickup task
            if self.task_demand[i] > 0:
                # get id of pickup and delivery
                P = i
                D = self.pd_pairs[i]

                # get order of pickup and delivery
                P_ord = task_order[P]
                D_ord = task_order[D]

                # if delivery comes first, swap tasks
                if D_ord < P_ord:
                    shuffled_idx[P_ord], shuffled_idx[D_ord] = shuffled_idx[D_ord], shuffled_idx[P_ord]
                    # we don't need to adjust task_order since each P is paired only with one D
        
        # save result
        self.shuffled_idx = shuffled_idx

        # simulate 5-second computation
        time.sleep(5)

    def retrieve_results(self):

        # return ordered list of tuples with (position, id)
        return [(self.task_pos[i], self.task_indices[i]) for i in self.shuffled_idx]

    def retrieve_cost(self):
        return 0