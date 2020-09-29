import numpy as np
import rclpy
from geometry_msgs.msg import Point
from .. import RobotData
from ..guidance import Guidance


class FormationControlGuidance(Guidance):
    # classe per livello di guida per scenari robotici task-like
    # questa classe si occupa di eseguire i task che trova in coda
    # nel frattempo sta in ascolto per eventuali optimization trigger,
    # che vengono gestiti in base alla strategia dinamica/statica scelta

    def __init__(self, agent_id: int, N: int, in_neigh: List[int], out_neigh: List[int], weights:np.ndarray, update_frequency: float, data: RobotData, pos_handler: str=None, pos_topic: str=None):
        super().__init__(agent_id, N, in_neigh, out_neigh, data, pos_handler, pos_topic)
        self.publisher_ = self.create_publisher(Point, 'agent_{}_velocity'.format(agent_id), 1)
        self.update_frequency = update_frequency 
        self.timer = self.create_timer(1.0/update_frequency, self.velocity_evaluation)

    def velocity_evaluation(self):
        msg = Point()
        self.u = 0.0
        current_pos = np.copy(self.current_pose.position)
        data = self.agent.neighbors_exchange(current_pos) 
        for ii in data:
            self.x_neigh[ii] = data[ii]
            error = self.x_neigh[ii] - current_pos
            u += formation_control_gain*(np.power(np.linalg.norm(error), 2)- np.power(weights[ii], 2)) * error
        
        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]

        self.publisher_.publish(msg)
    
    


    