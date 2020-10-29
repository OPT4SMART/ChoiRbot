from rclpy.node import Node
from choirbot_interfaces.msg import PositionTask, PositionTaskArray
from choirbot_interfaces.srv import PositionTaskService, TaskCompletionService
from std_msgs.msg import Empty
import numpy as np

np.random.seed(4)

class TaskTable(Node):

    def __init__(self, N, service_type):
        super().__init__('task_table')
        self.N = N
        self.trigger_publisher = self.create_publisher(Empty, '/optimization_trigger', 10)
        self.gc = self.create_guard_condition(self.send_new_tasks)
        self.task_list_srv = self.create_service(service_type, '/task_list', self.task_list_service)
        self.task_completion_srv = self.create_service(TaskCompletionService, '/task_completion', self.task_completion_service)
        self.task_list = []
        self.task_list_comm = [] # task list to be communicated to agents
        self.bipartite_graph = {} # keys correspond to seq_num, values are lists of agents allowed to perform that task
        self.largest_seq_num = 0
        self.label = 0

        self.get_logger().info('Task table started')
    
    def task_list_service(self, request, response):
        agent = request.agent_id

        # filter tasks based on bipartite graph
        filtered_tasks = [t for t in self.task_list_comm if agent in self.bipartite_graph[t.seq_num]]

        # return response
        response.tasks = self.make_task_array(filtered_tasks)
        response.tasks.all_tasks_count = len(self.task_list_comm)
        response.tasks.label = self.label

        task_list_print = [t.seq_num for t in filtered_tasks]
        self.get_logger().info('Sending task list to agent {}: {}'.format(agent, task_list_print))

        return response
    
    def task_completion_service(self, request, response):
        agent = request.agent_id
        task_seq_num = request.task_seq_num

        self.get_logger().info('Agent {} has completed task {}'.format(agent, task_seq_num))

        # get task
        index = next(k for k, t in enumerate(self.task_list) if t.seq_num == task_seq_num)

        # mark task as complete
        del self.task_list[index]
        del self.bipartite_graph[task_seq_num]

        # trigger generation of new tasks
        self.gc.trigger()
        
        return response

    def gen_task_id(self):
        # IDs are recycled
        ids = [t.id for t in self.task_list]
        for i in range(len(ids)+1):
            if i not in ids:
                return i
    
    def gen_task_seq_num(self):
        # sequence numbers are sequential (i.e. not recycled)
        seq_num = self.largest_seq_num
        self.largest_seq_num += 1
        return seq_num
    
    def send_new_tasks(self):
        if not self.can_generate_tasks():
            return # no need to generate new tasks
        
        self.get_logger().info('Generating new tasks and triggering optimization')
        self.generate_tasks()
        msg = Empty()
        self.trigger_publisher.publish(msg)
    
    def make_task_array(self, task_list):
        raise NotImplementedError
    
    def generate_tasks(self):
        raise NotImplementedError

    def can_generate_tasks(self):
        raise NotImplementedError

class PositionTaskTable(TaskTable):

    def __init__(self, N):
        super(PositionTaskTable, self).__init__(N, PositionTaskService)
        self.times_tasks_generated = 0
    
    def make_task_array(self, task_list):
        return PositionTaskArray(tasks=task_list)
    
    def generate_tasks(self):
        n_new_tasks = self.N - len(self.task_list) # in total we must always have N tasks
        prob = 0.5

        for _ in range(n_new_tasks):
            x_lim = y_lim = [-3, 3]
            position_x = np.random.uniform(x_lim[0], x_lim[1])
            position_y = np.random.uniform(y_lim[0], y_lim[1])
            position = [position_x, position_y]
            task_id = self.gen_task_id()
            task_seq_num = self.gen_task_seq_num()
            task = PositionTask(coordinates=position, id=task_id, seq_num=task_seq_num)
            self.task_list.append(task) # must do this before calling self.gen_task_id() again

            # each agent can perform this task with a given probability
            agents_can_perform = [i for i in range(self.N) if np.random.rand() < prob]
            self.bipartite_graph[task_seq_num] = agents_can_perform

        self.task_list_comm = self.task_list.copy()
        self.times_tasks_generated += 1
        self.label += 1

    def can_generate_tasks(self):
        return len(self.task_list) < self.N and self.times_tasks_generated < 8
