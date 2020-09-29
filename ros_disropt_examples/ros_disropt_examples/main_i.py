import rclpy
from rclpy.node import Node
from .ros_consensus import ros_consensus
from .test_synch import test_basic, test_future, test_cbgroups
from .test_asynch import test_asynch_basic, test_asynch_multiple_msg
from .task_assignment import task_assignment
import numpy as np


def main():
    rclpy.init(args=None)

    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

    id = node.get_parameter('agent_id').value
    N = node.get_parameter('N').value
    neigh = node.get_parameter('neigh').value
    exec = node.get_parameter('exec').value

    ########################

    if exec == 'consensus':
        ros_consensus(id, N, neigh)
    elif exec == 'test_basic':
        test_basic(id, N, neigh)
    elif exec == 'test_future':
        test_future(id, N, neigh)
    elif exec == 'test_cbgroups':
        test_cbgroups(id, N, neigh)
    elif exec == 'test_asynch_basic':
        test_asynch_basic(id, N, neigh)
    elif exec == 'test_asynch_multiple_msg':
        test_asynch_multiple_msg(id, N, neigh)
    elif exec == 'task_assignment':
        task_pos = np.array([[1.5, 2.1],[3.4, 4], [5.2, 6]])
        starting = np.array([[0, 0],[1, 1], [2, 2]])
        tasklist = [[0,1,2],[0,1],[2]]
        task_assignment(id, N, neigh, task_pos, tasklist[id], starting[id,:])
    rclpy.shutdown()
