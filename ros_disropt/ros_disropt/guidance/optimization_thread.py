from rclpy.task import Future
from typing import Type, List
from threading import Thread, Event

import numpy as np

from ros_disropt_interfaces.msg import PositionTask
from .guidance import Guidance
from .task import TaskGuidance
from ..scenario import Optimizer, TaskOptimizer, PDVRPOptimizer
from . import OptimizationSettings


class OptimizationThread(Thread):

    def __init__(self, old_thread: 'OptimizationThread', guidance: Guidance, optimizer_cls: Type[Optimizer], settings: OptimizationSettings):
        super().__init__()
        self._abort_event = Event()
        self.old_thread = old_thread
        self.guidance = guidance
        self.optimizer_cls = optimizer_cls
        self.settings = settings
        self.thread_num = old_thread.thread_num + 1 if old_thread is not None else 0
    
    def run(self):
        # quit old thread if still alive
        if self.old_thread is not None and self.old_thread.is_alive():
            self.old_thread.abort()
            self.old_thread.join() # wait for previous thread to exit

        # perform optimization
        self.do_optimize()

        # trigger guard condition when optimization has finished
        self.guidance.optimization_gc.trigger()
    
    def do_optimize(self):
        raise NotImplementedError

    def abort(self):
        self.guidance.get_logger().info('Aborting previous optimization')
        self._abort_event.set()

    def get_result(self):
        raise NotImplementedError

class TaskOptimizationThread(OptimizationThread):

    def __init__(self, future: Future, old_thread: 'TaskOptimizationThread', guidance: TaskGuidance, optimizer_cls: Type[TaskOptimizer], settings: OptimizationSettings):
        super().__init__(old_thread, guidance, optimizer_cls, settings)
        self.future = future
        self.result = None
        self.fetch_data_event = None
    
    def run(self):
        # prepare handling of asynchronous request
        self.fetch_data_event = Event()

        def unblock(_):
            self.fetch_data_event.set()
        
        self.future.add_done_callback(unblock)

        # call method of parent class
        super().run()

    def do_optimize(self):
        # wait for problem data to be ready
        self.fetch_data_event.wait()
        received_task_list = self.future.result().tasks
        
        # prepare problem data
        tasks = np.array([np.array(t.coordinates) for t in received_task_list.tasks])
        task_indices = [t.id for t in received_task_list.tasks]
        agent_id = self.guidance.agent_id
        N = self.guidance.agent_count
        starting_pos = self.guidance.current_position[:-1]
        in_neigh = self.guidance.in_neighbors
        out_neigh = self.guidance.out_neighbors
        iters = self.settings.max_iterations

        # initialize and start optimization
        self.guidance.get_logger().info('Starting optimization')
        optimizer = self.optimizer_cls(agent_id, N, tasks, task_indices, starting_pos, in_neighbors=in_neigh, out_neighbors=out_neigh, event=self._abort_event)
        optimizer.instantiate_algorithm()
        optimizer.optimize(iterations=iters)

        # fetch results
        results = optimizer.retrieve_results()

        # save result as list of PositionTask
        task_index = task_indices.index(results[1]) # results[1] is the task ID
        self.result = [received_task_list.tasks[task_index]] # (THERE IS ONLY ONE TASK BECAUSE WE ASSUME N = M)
    
    def get_result(self) -> List[PositionTask]:
        return self.result

# TODO TEMPORARY CLASS: adapt TaskOptimizer class so that previous class suits all situations
class PDVRPOptimizationThread(OptimizationThread):

    def __init__(self, future: Future, old_thread: 'PDVRPOptimizationThread', guidance: TaskGuidance, optimizer_cls: Type[PDVRPOptimizer], settings: OptimizationSettings):
        super().__init__(old_thread, guidance, optimizer_cls, settings)
        self.future = future
        self.result = None
        self.fetch_data_event = None
        self.final_cost = 0.0
    
    def run(self):
        # prepare handling of asynchronous request
        self.fetch_data_event = Event()

        def unblock(_):
            self.fetch_data_event.set()
        
        self.future.add_done_callback(unblock)

        # call method of parent class
        super().run()

    def do_optimize(self):
        # wait for problem data to be ready
        self.fetch_data_event.wait()
        received_task_list = self.future.result().tasks
        
        # prepare problem data
        tasks = np.array([np.array(t.coordinates) for t in received_task_list.tasks])
        task_indices = [t.id for t in received_task_list.tasks]
        tasks_print = [(t.seq_num, "P" if t.load > 0 else "D") for t in received_task_list.tasks]
        self.guidance.get_logger().info('Received tasks: {}'.format(tasks_print))
        self.guidance.get_logger().info('Indices of received tasks: {}'.format(task_indices))

        demand = np.array([t.load for t in received_task_list.tasks])
        service_times = np.array([t.service_time for t in received_task_list.tasks])
        task_count = received_task_list.all_tasks_count
        speed = self.guidance.data.speed
        capacity = self.guidance.data.capacity
        epsilon = self.settings.epsilon
        use_runavg = self.settings.runavg
        current_load = self.guidance.data.current_load
        agent_id = self.guidance.agent_id
        N = self.guidance.agent_count
        starting_pos = self.guidance.current_position[:-1]
        in_neigh = self.guidance.in_neighbors
        out_neigh = self.guidance.out_neighbors
        iters = self.settings.max_iterations

        # create map of seq_num => id
        id_map = {t.seq_num:t.id for t in received_task_list.tasks}

        # list of pending deliveries that must be performed by this agent (task.id)
        pending_tasks = [id_map[i] for i in self.guidance.pending_tasks]

        # make dictionary with pickup/delivery pairs (key = pickup task.id, value = delivery task.id)
        pd_pairs = {t.id:id_map[t.corresponding_delivery] for t in received_task_list.tasks if t.load > 0}

        # initialize and start optimization
        self.guidance.get_logger().info('Starting optimization')
        optimizer = self.optimizer_cls(agent_id,N,tasks,task_indices,demand,task_count,service_times,starting_pos,
                speed,capacity,epsilon,use_runavg,current_load,pending_tasks=pending_tasks,pickup_delivery_pairs=pd_pairs,
                in_neighbors=in_neigh,out_neighbors=out_neigh,event=self._abort_event)
        optimizer.instantiate_algorithm()
        optimizer.optimize(iterations=iters)

        # fetch results
        result = optimizer.retrieve_results()
        self.final_cost = optimizer.retrieve_cost()

        # save result
        result_idx = [task_indices.index(t[1]) for t in result] # ordered indices of tasks in "result" (t[1] is the ID of a task)
        self.result = [received_task_list.tasks[j] for j in result_idx] # save as list of PositionTask objects
    
    def get_result(self) -> List[PositionTask]:
        return self.result
    
    def get_cost(self):
        return self.final_cost