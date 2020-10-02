from std_msgs.msg import Empty
from typing import Type

from ros_disropt_interfaces.srv import TaskCompletionService
from ...scenario import TaskOptimizer
from .. import OptimizationSettings, RobotData
from .executor import TaskExecutor
from ..guidance import Guidance


class TaskGuidance(Guidance):
    # classe per livello di guida per scenari robotici task-like
    # questa classe si occupa di eseguire i task che trova in coda
    # nel frattempo sta in ascolto per eventuali optimization trigger,
    # che vengono gestiti in base alla strategia dinamica/statica scelta

    def __init__(self, optimizer_cls: Type[TaskOptimizer],
            task_executor_cls: Type[TaskExecutor], data: RobotData, opt_settings: OptimizationSettings,
            pos_handler: str=None, pos_topic: str=None):
        super().__init__(pos_handler, pos_topic)
        self.data = data
        self.optimizer_cls = optimizer_cls
        self.opt_settings = opt_settings
        self.current_task = None
        self.pending_tasks = []
        self.task_queue = []

        # triggering mechanism to start optimization
        self.opt_trigger_subscription = self.create_subscription(
                Empty, '/optimization_trigger', self.start_optimization, 10)
        self.task_list_client = self.create_client(task_executor_cls.service, '/task_list')
        self.task_completion_client = self.create_client(TaskCompletionService, '/task_completion')
        self.optimization_thread = None
        self.optimization_gc = self.create_guard_condition(self.optimization_ended)

        # initialize task executor
        self.task_executor = task_executor_cls(self.agent_id, self, self.data)

        # guard condition to start a new task
        self.task_gc = self.create_guard_condition(self.start_new_task)

        # wait for services
        self.task_list_client.wait_for_service()
        self.task_completion_client.wait_for_service()
        self.task_executor.wait_for_services()

        self.get_logger().info('Guidance {} started'.format(self.agent_id))
    
    def start_optimization(self, _):
        self.get_logger().info('Optimization triggered: requesting task list')

        # remove all enqueued tasks
        self.task_queue = []

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
        self.task_queue = result
        self.optimization_thread = None

        # start new task
        self.task_gc.trigger()
    
    def start_new_task(self):
        # stop if a task is already in execution or if there are no new tasks
        if self.current_task is not None or not self.task_queue:
            return
        
        # get task
        task = self.task_queue.pop(0)

        # ask table permission to perform task
        self.get_logger().info('Got new task (seq_num {}) - starting execution'.format(task.seq_num))
        self.current_task = task
        self.task_executor.execute_async(self.current_task, self.task_ended)
    
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