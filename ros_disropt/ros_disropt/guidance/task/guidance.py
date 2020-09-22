from rclpy.task import Future
from std_msgs.msg import Empty
from typing import Type, List

from ros_disropt_interfaces.srv import TaskCompletionService, TaskPermissionService
from ...scenario import TaskOptimizer
from .. import OptimizationSettings, RobotData
from .dynamic import TaskManager
from .executor import TaskExecutor
from ..guidance import Guidance


class TaskGuidance(Guidance):
    # classe per livello di guida per scenari robotici task-like
    # questa classe si occupa di eseguire i task che trova in coda
    # nel frattempo sta in ascolto per eventuali optimization trigger,
    # che vengono gestiti in base alla strategia dinamica/statica scelta

    def __init__(self, agent_id: int, N: int, in_neigh: List[int], out_neigh: List[int], optimizer_cls: Type[TaskOptimizer],
            manager: TaskManager, task_executor_cls: Type[TaskExecutor], data: RobotData, opt_settings: OptimizationSettings,
            pos_handler: str=None, pos_topic: str=None):
        super().__init__(agent_id, N, in_neigh, out_neigh, data, pos_handler, pos_topic)
        self.optimizer_cls = optimizer_cls
        self.task_manager = manager
        self.current_task = None
        self.opt_settings = opt_settings
        self.pending_tasks = []

        # triggering mechanism to start optimization
        self.opt_trigger_subscription = self.create_subscription(
                Empty, 'optimization_trigger', self.start_optimization, 10)
        self.task_list_client = self.create_client(task_executor_cls.service, 'task_list')
        self.task_completion_client = self.create_client(TaskCompletionService, 'task_completion')
        self.task_permission_client = self.create_client(TaskPermissionService, 'task_permission')
        self.optimization_thread = None
        self.optimization_gc = self.create_guard_condition(self.optimization_ended)
        self._permission_future = None

        # initialize task executor
        self.task_executor = task_executor_cls(agent_id, self, self.data)

        # guard condition to start a new task
        self.task_gc = self.create_guard_condition(self.start_new_task)

        # wait for services
        self.task_list_client.wait_for_service()
        self.task_completion_client.wait_for_service()
        self.task_permission_client.wait_for_service()
        self.task_executor.wait_for_services()

        self.get_logger().info('Guidance {} started'.format(agent_id))
    
    def start_optimization(self, _):
        self.get_logger().info('Optimization triggered: requesting task list')

        # remove all enqueued tasks
        self.task_manager.empty_tasks()

        # request updated task list
        request = self.task_executor.service.Request(agent_id=self.agent_id)
        future = self.task_list_client.call_async(request)

        # launch task optimization on a new thread
        from ..optimization_thread import TaskOptimizationThread
        thread = TaskOptimizationThread(future, self.optimization_thread, self, self.optimizer_cls, self.opt_settings)
        thread.start()
        self.optimization_thread = thread
    
    def optimization_ended(self):
        self.get_logger().info('Optimization ended')

        # collect results from external thread
        result = self.optimization_thread.get_result()
        self.get_logger().info('Assigned tasks {}'.format([task.seq_num for task in result]))
        self.task_manager.update_tasks(result)
        self.optimization_thread = None

        # start new task
        self.task_gc.trigger()
    
    def start_new_task(self):
        # stop if a task is already in execution
        if self.current_task is not None:
            return

        # stop if there are no new tasks
        if not self.task_manager.has_tasks():
            return
        
        # get task
        task = self.task_manager.get_task()

        # TODO maybe we don't want to always ask permission to table

        # ask table permission to perform task
        self.get_logger().info('Got new task (seq_num {}) - asking permission'.format(task.seq_num))
        request = TaskPermissionService.Request()
        request.agent_id = self.agent_id
        request.task_seq_num = task.seq_num
        self.current_task = task

        # sending request
        future = self.task_permission_client.call_async(request)
        future.add_done_callback(self._permission_callback)
    
    def _permission_callback(self, future: Future):
        # extract result
        response = future.result()

        if response.permission_granted:
            # permission granted - execute task
            self.get_logger().info('Permission granted for task (seq_num {}) - starting execution'.format(self.current_task.seq_num))
            self.task_executor.execute_async(self.current_task, self.task_ended)

            # TODO mark some of future tasks as pending
        else:
            # permission denied - clean and start new task
            self.get_logger().info('Permission denied for task (seq_num {})'.format(self.current_task.seq_num))
            self.current_task = None
            self.task_gc.trigger()
    
    def task_ended(self):
        # log to console
        self.get_logger().info('Task completed (seq_num {})'.format(self.current_task.seq_num))

        # notify table that task execution has completed
        request = TaskCompletionService.Request()
        request.agent_id = self.agent_id
        request.task_seq_num = self.current_task.seq_num
        self.task_completion_client.call_async(request)

        # start a new task
        self.current_task = None
        self.task_gc.trigger()