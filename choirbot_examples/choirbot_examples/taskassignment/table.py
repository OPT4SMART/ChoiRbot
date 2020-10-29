import rclpy
from rclpy.node import Node
from choirbot.guidance.task import PositionTaskTable
import time


def main():
    rclpy.init()

    node = Node('table_parameters', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    N = node.get_parameter('N').value

    table = PositionTaskTable(N)
    table.gc.trigger()

    table.get_logger().info('Waiting for 10 seconds to let all nodes be ready')
    time.sleep(10)

    rclpy.spin(table)
    rclpy.shutdown()
