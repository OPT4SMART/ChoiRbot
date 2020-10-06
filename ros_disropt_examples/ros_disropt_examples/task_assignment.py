import rclpy
from rclpy.node import Node
from ros_disropt.guidance.task import TaskGuidance, PositionTaskTable, PositionTaskExecutor
from ros_disropt.optimizer import TaskOptimizer
import time

def main_guidance():
    rclpy.init(args=None)

    opt_settings = {'max_iterations': 20}
    executor = PositionTaskExecutor()
    optimizer = TaskOptimizer(settings=opt_settings)

    pos_handler = 'pubsub'
    pos_topic = 'odom'
    # pos_handler = 'vicon'
    # pos_topic = '/vicon/T{}/T{}'.format(guidance.agent_id, guidance.agent_id)
    
    guidance = TaskGuidance(optimizer, executor, None, pos_handler, pos_topic)

    rclpy.spin(guidance)

    rclpy.shutdown()

def main_table():
    rclpy.init(args=None)

    node = Node('table_parameters', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    N = node.get_parameter('N').value

    table = PositionTaskTable(N)
    table.gc.trigger()

    node.get_logger().info('Waiting for 5 seconds')
    time.sleep(5)

    rclpy.spin(table)

    rclpy.shutdown()