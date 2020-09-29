import rclpy
from rclpy.node import Node
from ros_disropt.guidance.formation_control import FormationControlGuidance
import numpy as np

def main():
    rclpy.init(args=None)
    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value

    Adj = np.array([[0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0],
                    [0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0],
                    [0, 1, 0, 1, 0, 1],
                    [1, 0, 1, 0, 1, 0]])

    in_neigh  = np.nonzero(Adj[:, agent_id])[0].tolist()
    out_neigh = np.nonzero(Adj[agent_id, :])[0].tolist()
    N = Adj.shape[0]
    update_frequency = 100

    d = 0.3
    ddiag = np.sqrt(5)*d

# Weight matrix to control inter-agent distances
    W = np.array([
        [0, d, 0, d, 0, ddiag],
        [d, 0, d, 0, d, 0],
        [0, d, 0, ddiag, 0, d],
        [d, 0, ddiag, 0, d, 0],
        [0, d, 0, d, 0, d],
        [ddiag, 0, d, 0, d, 0]
    ])

    pos_handler = 'pubsub'
    pos_topic = '/agent_{}/odom'.format(agent_id)

    weights = W[agent_id, :]
    guidance = FormationControlGuidance(agent_id, N, in_neigh, out_neigh, weights, 
                                        update_frequency, pos_handler, pos_topic)

    rclpy.spin(guidance)
