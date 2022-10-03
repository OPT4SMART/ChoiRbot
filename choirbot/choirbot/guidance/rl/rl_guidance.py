from rclpy.node import Node
from ..guidance import Guidance


class RLController:

    def initialize(self, node: Node, input_topic: str):
        self.node = node
        self.publisher_ = self._initialize_publisher(input_topic)
    
    def _initialize_publisher(self, input_topic):
        raise NotImplementedError
    
    def evaluate_input(self, data):
        raise NotImplementedError

    def send_input(self, u):
        raise NotImplementedError

class DistributedRLGuidance(Guidance):

    def __init__(self, update_frequency: float, controller: RLController, pos_handler: str=None, pos_topic: str=None):
        super().__init__(pos_handler, pos_topic)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/self.update_frequency, self.control)
        self.controller = controller
        controller.initialize(self)
        self.get_logger().info('Guidance {} started'.format(self.agent_id))

    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # exchange current position with neighbors
        data = self.communicator.neighbors_exchange(self.current_pose, self.in_neighbors, self.out_neighbors, False)

        # compute input
        u = self.controller.evaluate_input(data)

        # send input to planner/controller
        self.controller.send_input(u)

class SingleAgentRLPolicyGuidance(DistributedRLGuidance):

    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return

        # compute input
        u = self.controller.evaluate_input(self.current_pose)

        # send input to planner/controller
        self.controller.send_input(u)
