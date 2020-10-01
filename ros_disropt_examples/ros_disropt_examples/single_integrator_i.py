import rclpy
from rclpy.node import Node
from ros_disropt.integrator import SingleIntegrator
from rclpy.executors import MultiThreadedExecutor


def main():
    rclpy.init(args=None)
    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value

    P = np.array([
        [2, 2, 0,
        [2, 0, 0,     
        [0, 2, 0,
        [0, 1.5, 0,
        [1.5, 0, 0,
         1.5, 3 ,0])

    initial_pos = P[agent_id, :]
    steptime = 0.01

    integrator = SingleIntegrator(agent_id, steptime, initial_pos)
    rclpy.spin(integrator)
