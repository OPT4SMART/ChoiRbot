from rclpy.task import Future
from std_msgs.msg import Empty
from threading import Event

from ros_disropt_interfaces.srv import TaskCompletionService
from ros_disropt_interfaces.msg import PositionTask
from ...optimizer import TaskOptimizer
from .. import RobotData
from .executor import TaskExecutor
from ..guidance import Guidance
from ..optimization_thread import OptimizationThread


class TaskGuidance(Guidance):
    # classe per livello di guida per scenari robotici task-like
    # questa classe si occupa di eseguire i task che trova in coda
    # nel frattempo sta in ascolto per eventuali optimization trigger,
    # che vengono gestiti in base alla strategia dinamica/statica scelta

    def __init__(self, optimizer: TaskOptimizer, executor: TaskExecutor,
            data: RobotData, pos_handler: str=None, pos_topic: str=None):
        super().__init__(pos_handler, pos_topic)
        self.data = data
        self.optimizer = optimizer
        self.task_executor = executor
        self.current_task = None
        self.pending_tasks = []
        self.task_queue = []

        # triggering mechanism to start optimization
        self.opt_trigger_subscription = self.create_subscription(
                Empty, '/optimization_trigger', self.start_optimization, 10)
        self.task_list_client = self.create_client(executor.service, '/task_list')
        self.task_completion_client = self.create_client(TaskCompletionService, '/task_completion')
        self.optimization_thread = None
        self.optimization_gc = self.create_guard_condition(self.optimization_ended)

        # guard condition to start a new task
        self.task_gc = self.create_guard_condition(self.start_new_task)

        # initialize task executor
        self.task_executor.initialize(self)

        # wait for services
        self.task_list_client.wait_for_service()
        self.task_completion_client.wait_for_service()

        self.get_logger().info('Guidance {} started'.format(self.agent_id))
    
    def start_optimization(self, _):
        self.get_logger().info('Optimization triggered: requesting task list')

        # remove all enqueued tasks
        self.task_queue = []

        # request updated task list
        request = self.task_executor.service.Request(agent_id=self.agent_id)
        future = self.task_list_client.call_async(request)

        # launch task optimization on a new thread
        thread = TaskOptimizationThread(future, self.optimization_thread, self, self.optimizer)
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


class TaskOptimizationThread(OptimizationThread):

    def __init__(self, future: Future, old_thread: 'TaskOptimizationThread', guidance: TaskGuidance, optimizer: TaskOptimizer):
        super().__init__(old_thread, guidance, optimizer)
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

        # initialize and start optimization
        # TODO forse conviene chiamare un metodo della classe guida per il logging, in questo modo si customizza maggiormente
        self.guidance.get_logger().info('Starting optimization')
        self.optimizer.create_problem(received_task_list)
        self.optimizer.optimize()
    
    def get_result(self):
        return self.optimizer.get_result()
    
    def get_cost(self):
        return self.optimizer.get_cost()
