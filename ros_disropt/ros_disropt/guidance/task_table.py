import rclpy
from rclpy.node import Node
from ros_disropt_interfaces.msg import PositionTask, PositionTaskArray
from ros_disropt_interfaces.srv import PositionTaskService, TaskCompletionService, TaskPermissionService
from std_msgs.msg import Empty
import numpy as np
from threading import Thread

np.random.seed(4)

class TaskTable(Node):

    def __init__(self, N, service_type):
        super().__init__('task_table')
        self.N = N
        self.publisher = self.create_publisher(Empty, 'optimization_trigger', 10)
        self.gc = self.create_guard_condition(self.send_new_tasks)
        self.timer = self.create_timer(5, self.timer_callback)
        self.task_list_srv = self.create_service(service_type, 'task_list', self.task_list_service)
        self.task_completion_srv = self.create_service(TaskCompletionService, 'task_completion', self.task_completion_service)
        self.task_list = []
        self.task_list_comm = [] # task list to be communicated to agents
        self.agents_requested_task_list = [] # agents having already requested task list
        self.completed_tasks = []
        self.bipartite_graph = {} # each key corresponds to a task
        self.largest_seq_num = 0

        self.get_logger().info('Task table started')
    
    def task_list_service(self, request, response):
        agent = request.agent_id

        # filter tasks based on bipartite graph
        filtered_tasks = [t for t in self.task_list_comm if agent in self.bipartite_graph[t.seq_num]]

        # return response
        response.tasks = self.make_task_array(filtered_tasks)
        response.tasks.all_tasks_count = len(self.task_list_comm)

        task_list_print = [(t.seq_num, "P" if t.load > 0 else "D") for t in self.task_list_comm]
        self.get_logger().info('Sending task list to agent {}: {}'.format(agent, task_list_print))
        self.agents_requested_task_list.append(agent)

        return response
    
    def make_task_array(self, task_list):
        raise NotImplementedError
    
    def timer_callback(self):
        self.gc.trigger()
    
    def task_completion_service(self, request, response):
        agent = request.agent_id
        task_seq_num = request.task_seq_num

        self.get_logger().info('Agent {} has completed task {}'.format(agent, task_seq_num))

        # get task
        index = next(k for k, t in enumerate(self.task_list) if t.seq_num == task_seq_num)
        task = self.task_list[index]

        # mark task as complete
        del self.task_list[index]
        del self.bipartite_graph[task_seq_num]
        self.completed_tasks.append(task)

        # trigger generation of new tasks
        self.gc.trigger()
        
        return response

    def gen_task_id(self):
        ids = [t.id for t in self.task_list]
        for i in range(len(ids)+1):
            if i not in ids:
                return i
    
    def gen_task_seq_num(self):
        seq_num = self.largest_seq_num
        self.largest_seq_num += 1
        return seq_num

    def generate_tasks(self):
        raise NotImplementedError
    
    def send_new_tasks(self):
        if not self.can_generate_tasks():
            return # no need to generate new tasks
        
        self.get_logger().info('Generating new tasks and triggering optimization')
        self.generate_tasks()
        msg = Empty()
        self.publisher.publish(msg)

        self.agents_requested_task_list = [] # reset list of agents having already requested task list

    def can_generate_tasks(self):
        raise NotImplementedError

class PositionTaskTable(TaskTable):

    def __init__(self, N):
        super(PositionTaskTable, self).__init__(N, PositionTaskService)
        self.times_tasks_generated = 0
    
    def make_task_array(self, task_list):
        return PositionTaskArray(tasks=task_list)
    
    def generate_tasks(self):
        n_new_tasks = self.N
        list_new_tasks = []

        # space_dim = 2
        for _ in range(n_new_tasks):
            # x_lim = [-1.2, 1.5]
            # y_lim = [-1.8, 1.5]
            x_lim = y_lim = [-3, 3]
            position_x = np.random.uniform(x_lim[0], x_lim[1])
            position_y = np.random.uniform(y_lim[0], y_lim[1])
            position = [position_x, position_y]
            task_id = self.gen_task_id()
            task_seq_num = self.gen_task_seq_num()
            task = PositionTask(coordinates=position, id=task_id, seq_num=task_seq_num)
            list_new_tasks.append(task)
            self.task_list.append(task) # must do this before calling self.gen_task_id() again
            self.bipartite_graph[task_seq_num] = list(np.arange(self.N)) # all agents can do this task

        self.task_list_comm = self.task_list.copy()
        self.times_tasks_generated += 1

    def can_generate_tasks(self):
        return len(self.task_list) == 0 #and self.times_tasks_generated < 2

class PDPositionTaskTable(TaskTable):

    def __init__(self, N, n_groups, tasks_per_group, seed, log_prefix):
        super(PDPositionTaskTable, self).__init__(N, PositionTaskService)
        self.task_permission_srv = self.create_service(TaskPermissionService, 'task_permission', self.task_permission_service)
        self.task_permissions = {} # each key corresponds to a task
        self.times_tasks_generated = 0
        self.n_groups = n_groups
        self.tasks_per_group = tasks_per_group
        self.log_prefix = log_prefix
        np.random.seed(seed)
        self.filename = "{}table.pkl".format(self.log_prefix)
        import os
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        self.bipartite_graph2 = []

        # create groups
        self.groups = {j:[] for j in range(n_groups)} # initially they are empty
        for i in range(N):
            group = i % n_groups
            self.groups[group].append(i)
        
        self.get_logger().info('Groups: {}'.format(self.groups))
    
    def make_task_array(self, task_list):
        return PositionTaskArray(tasks=task_list)
    
    def gen_task_id(self):
        # in this pickup/delivery table, IDs of authorized tasks (and not yet completed) can be used
        ids = [t.id for t in self.task_list if t.seq_num not in self.task_permissions]
        for i in range(len(ids)+1):
            if i not in ids:
                return i
    
    def generate_tasks(self):
        
        list_new_tasks = []
        max_P = 10 # max demand
        max_t = 5  # max service time

        # size of the area
        # x_lim = [-1.2, 1.5]
        # y_lim = [-1.8, 1.5]
        x_lim = y_lim = [0, 3]

        # number of new pickup tasks (then we assume P = D)
        n_new_P_tasks = (self.tasks_per_group-1) * self.n_groups

        # generate demands
        demand = np.random.uniform(1, max_P, n_new_P_tasks)

        # generate service times
        serv_time = np.random.uniform(0.1, max_t, 2*n_new_P_tasks)

        # generate random task positions
        position_x = np.random.uniform(x_lim[0], x_lim[1], 2*n_new_P_tasks)
        position_y = np.random.uniform(y_lim[0], y_lim[1], 2*n_new_P_tasks)
        task_pos = np.column_stack((position_x, position_y))

        # generate task objects
        for j in range(n_new_P_tasks):
            # generate pickup/delivery sequence numbers
            task_seq_num_P = self.gen_task_seq_num()
            task_seq_num_D = self.gen_task_seq_num()

            # determine to which agents this task must be assigned
            group = j//(self.tasks_per_group-1)
            agents = self.groups[group].copy() # agents of group "group"
            
            # check if task is shared with another group
            if j % (self.tasks_per_group - 1) == 0:
                # add agents of secondary group
                group_2 = (group-1) % self.n_groups
                agents.extend(self.groups[group_2])
                self.get_logger().info('Task {} assigned to groups {} and {}'.format(j, group, group_2))
            else:
                self.get_logger().info('Task {} assigned to group {}'.format(j, group))

            # generate pickup task
            position_P = task_pos[j, :].tolist()
            task_id_P = self.gen_task_id()
            load_P = demand[j]
            service_time_P = serv_time[j]
            task_P = PositionTask(coordinates=position_P, id=task_id_P, seq_num=task_seq_num_P, load=load_P,
                service_time=service_time_P, corresponding_delivery=task_seq_num_D)
            
            # add pickup task
            list_new_tasks.append(task_P)
            self.task_list.append(task_P) # must do this before calling self.gen_task_id() again
            self.bipartite_graph[task_seq_num_P] = agents

            # generate delivery task            
            position_D = task_pos[j+n_new_P_tasks, :].tolist()
            task_id_D = self.gen_task_id()
            load_D = -demand[j] # we assume q_delivery = -q_pickup
            service_time_D = serv_time[j+n_new_P_tasks]
            task_D = PositionTask(coordinates=position_D, id=task_id_D, seq_num=task_seq_num_D, load=load_D,
                service_time=service_time_D, corresponding_pickup=task_seq_num_P)
            
            # add delivery task
            list_new_tasks.append(task_D)
            self.task_list.append(task_D)
            self.bipartite_graph[task_seq_num_D] = agents
        
        self.times_tasks_generated += 1
        self.bipartite_graph2 = self.bipartite_graph.copy()

        task_list_print = [(t.seq_num, "P" if t.load > 0 else "D") for t in list_new_tasks]
        self.get_logger().info('New tasks: {}'.format(task_list_print))

        # generate list to communicate to agents
        # discard tasks for which authorization has been already requested
        self.task_list_comm = [t for t in self.task_list if t.seq_num not in self.task_permissions]

    def can_generate_tasks(self):
        return self.times_tasks_generated == 0
        # return len(self.task_list) < 2*self.N-3 and self.times_tasks_generated < 2
    
    def task_permission_service(self, request, response):
        agent = request.agent_id
        task_seq_num = request.task_seq_num

        # conditions so that task can be done
        task_already_assigned = task_seq_num in self.task_permissions
        task_already_done = task_seq_num in [t.seq_num for t in self.completed_tasks]
        task_list_not_requested = agent not in self.agents_requested_task_list # dynamic case: avoid requests of old task queues
        
        # check for associated pickup/delivery tasks
        corresponding_pickup_not_authorized = False

        if not task_already_done:
            # pick task from task list
            task = next(filter(lambda t: t.seq_num == task_seq_num, self.task_list))

            # check if task is delivery, then if pickup task has been authorized to the same agent
            pickup_seqnum = task.corresponding_pickup
            if task.load < 0 and (pickup_seqnum not in self.task_permissions or self.task_permissions[pickup_seqnum] != agent):
                corresponding_pickup_not_authorized = True

        # check if task can be done
        if task_already_assigned or task_already_done or task_list_not_requested or corresponding_pickup_not_authorized:
            if task_already_assigned:
                reason = 'task already assigned'
            elif task_already_done:
                reason = 'task already done'
            elif task_list_not_requested:
                reason = 'task list of agent is expired'
            else:
                reason = 'corresponding pickup not authorized'
            self.get_logger().info('Denying permission to agent {} for task {}: {}'.format(agent, task_seq_num, reason))
            response.permission_granted = False
        else:
            self.get_logger().info('Granting permission to agent {} for task {}'.format(agent, task_seq_num))
            response.permission_granted = True
            self.task_permissions[task_seq_num] = agent

        return response
    
    def task_completion_service(self, request, response):
        response = super().task_completion_service(request, response)

        # pick task
        task = next(filter(lambda t: t.seq_num == request.task_seq_num, self.completed_tasks))

        # if task is delivery, we can delete both P and D tasks from permission dictionary
        if task.load < 0:
            del self.task_permissions[task.corresponding_pickup] # delete pickup
            del self.task_permissions[task.seq_num] # delete delivery
        
        # if there are no more tasks, quit
        self.get_logger().info('There are {} tasks left'.format(len(self.task_list)))
        if len(self.task_list) == 0:
            self.save_tasks()
            Thread(target=rclpy.get_default_context().shutdown()).start()

        return response
    
    def save_tasks(self):

        import dill
        tasks = []
        seq_num_to_id = {t.seq_num:t.id for t in self.completed_tasks}

        for t in self.completed_tasks:
            corresponding_task = t.corresponding_delivery if t.load > 0 else t.corresponding_pickup
            task = {
                    'id': t.id,
                    'pos': t.coordinates,
                    'corresponding_task': seq_num_to_id[corresponding_task],
                    'load': t.load,
                    'service_time': t.service_time,
                    'doable_by': self.bipartite_graph2[t.seq_num]
                }
            tasks.append(task)
        
        table = {'N': self.N, 'tasks': tasks}

        with open(self.filename, 'wb') as out_file:
            dill.dump(table, out_file, dill.HIGHEST_PROTOCOL)