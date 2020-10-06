from rclpy.node import Node
from typing import Type

from .optimization_thread import OptimizationThread
from .. import Pose
from ..utils.position_getter import pose_subscribe
from ..communicator import Communicator
from ..optimizer import Optimizer


class Guidance(Node):

    def __init__(self, pose_handler: str=None, pose_topic: str=None):
        super().__init__('guidance', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # get parameters
        self.agent_id = self.get_parameter('agent_id').value
        self.n_agents = self.get_parameter('N').value
        self.in_neighbors = self.get_parameter('in_neigh').value
        self.out_neighbors = self.get_parameter('out_neigh').value

        # initialize pose subscription
        self.current_pose = Pose(None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose)

        # initialize communicator
        self.communicator = self.instantiate_communicator()

    def instantiate_communicator(self):
        return Communicator(self.agent_id, self.n_agents, self.in_neighbors)


class OptimizationGuidance(Guidance):

    def __init__(self, optimizer: Optimizer, thread_t: Type[OptimizationThread],
            pos_handler: str=None, pos_topic: str=None):
        super().__init__(pos_handler, pos_topic)
        
        # save optimizer
        self.optimizer = optimizer

        # create guard condition to be triggered at end of optimization
        self.optimization_ended_gc = self.create_guard_condition(self.optimization_ended)

        # create and start optimization thread
        self.optimization_thread = thread_t(self, self.optimizer, self.optimization_ended_gc)
        self.optimization_thread.start()
    
    def optimization_ended(self):
        raise NotImplementedError