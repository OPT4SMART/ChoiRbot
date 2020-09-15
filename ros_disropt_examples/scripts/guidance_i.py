import rclpy
from rclpy.node import Node
from ros_disropt.guidance import TaskGuidance, PDPositionTaskTable, PositionTaskTable, DynamicTaskManager, RobotData, OptimizationSettings, PositionTaskExecutor
from ros_disropt.scenario import PDVRPOptimizer, TaskOptimizer, MockPDVRPOptimizer
from ros_disropt_interfaces.msg import PositionTask, PositionTaskArray
import time

def main_guidance():
    rclpy.init(args=None)

    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value
    N = node.get_parameter('N').value
    in_neigh = node.get_parameter('in_neigh').value
    out_neigh = node.get_parameter('out_neigh').value
    log_prefix = node.get_parameter('log_prefix').value

    L_cap = [10, 12, 10, 11, 12, 11, 10, 12, 9, 12]
    capacity = L_cap[agent_id]
    speed = 0.1 # m/s
    current_load = 0.0

    iterations = 80
    epsilon = 0.9
    use_runavg = True

    manager = DynamicTaskManager[PositionTask, PositionTaskArray]()
    data = RobotData(capacity, speed, current_load)
    opt_settings = OptimizationSettings(iterations, epsilon, use_runavg)

    pos_handler = 'pubsub'
    pos_topic = '/agent_{}/odom'.format(agent_id)
    # pos_handler = 'vicon'
    # pos_topic = '/vicon/T{}/T{}'.format(agent_id, agent_id)
    
    guidance = TaskGuidance(agent_id, N, in_neigh, out_neigh, MockPDVRPOptimizer, manager, PositionTaskExecutor, data, opt_settings, pos_handler, pos_topic, True, log_prefix)
    # guidance = TaskGuidance(agent_id, N, in_neigh, out_neigh, TaskOptimizer, manager, PositionTaskExecutor, data, opt_settings, pos_handler, pos_topic, False)

    rclpy.spin(guidance)

    # rclpy.shutdown()

def main_table():
    rclpy.init(args=None)

    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    N = node.get_parameter('N').value
    n_groups = node.get_parameter('n_groups').value
    tasks_per_group = node.get_parameter('tasks_per_group').value
    log_prefix = node.get_parameter('log_prefix').value
    seed = node.get_parameter('seed').value

    table = PDPositionTaskTable(N, n_groups, tasks_per_group, seed, log_prefix)
    # table = PositionTaskTable(N)

    node.get_logger().info('Waiting for 3 seconds')
    time.sleep(3)

    rclpy.spin(table)

    # rclpy.shutdown()