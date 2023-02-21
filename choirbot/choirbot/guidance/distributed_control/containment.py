import numpy as np
import random
from .distributed_control import DistributedControlGuidance
from ...communicator import TimeVaryingCommunicator


class ContainmentGuidance(DistributedControlGuidance):

    def __init__(self, update_frequency: float, gain: float=0.1, pose_handler: str=None, pose_topic: str=None, input_topic = 'velocity'):
        super().__init__(update_frequency, pose_handler, pose_topic, input_topic)
        self.containment_gain = gain
        self.is_leader = self.get_parameter('is_leader').value

    def evaluate_input(self, neigh_data):
        u = np.zeros(3)
        if not self.is_leader:
            for pos_ii in neigh_data.values():
                u += self.containment_gain*(pos_ii.position - self.current_pose.position)
        return u

class TimeVaryingContainmentGuidance(ContainmentGuidance):

    def __init__(self, update_frequency: float, gain: float=0.1, edge_prob = 0.8, pose_handler: str=None, pose_topic: str=None):
        super().__init__(update_frequency, gain, pose_handler, pose_topic)
        self.edge_prob = edge_prob
    
    def _instantiate_communicator(self):
        # communicator must have differentiated topics
        return TimeVaryingCommunicator(self.agent_id, self.n_agents, self.in_neighbors,
            out_neighbors=self.out_neighbors, differentiated_topics=True)
    
    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # decide neighbors and prepare messages
        # messages are tuples of the type (position, bool) with bool = True if
        # the neighbor is active from the perspective of the current robot
        neigh = random.sample(self.in_neighbors, np.random.binomial(len(self.in_neighbors), self.edge_prob))
        msg = {j:(self.current_pose,j in neigh) for j in self.in_neighbors}
        
        # exchange current position with neighbors
        data = self.communicator.neighbors_exchange(msg, self.in_neighbors, self.out_neighbors, True)

        # discard messages from inactive neighbors (either from my or their perspective)
        neighbor_positions = {j:value[0] for j, value in data.items() if j in neigh and value[1]}

        # compute input
        u = self.evaluate_input(neighbor_positions)

        # send input to planner/controller
        self.send_input(u)
