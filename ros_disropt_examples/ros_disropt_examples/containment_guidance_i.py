import rclpy
from rclpy.node import Node
from ros_disropt.guidance.distributed_control import FormationControlGuidance
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
    
    if agent_id < 3:
         is_leader = True
    else:
        is_leader = False


    pos_handler = 'pubsub'
    pos_topic = '/agent_{}/odom'.format(agent_id)

    
    guidance = FormationControlGuidance(agent_id, N, in_neigh, out_neigh, 
                                        update_frequency, is_leader, pos_handler, pos_topic)

    rclpy.spin(guidance)
