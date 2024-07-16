from rclpy.node import Node
from typing import Type, Callable

from .optimization_thread import OptimizationThread
from .. import Pose
from ..utils.position_getter import pose_subscribe
from ..communicator import TimeVaryingCommunicator
from ..optimizer import Optimizer


class Guidance(Node):
    """
    Bases: :class:`rclpy.node.Node`

    Base ROS node for guidance level

    This is the base class for the guidance level in a multi-robot scenario.
    This is a basic class and should be extended before instantiation.

    The following ROS parameters must be set in order to instantiate the class:

        - ``N`` (int): total number of agents in the network
        - ``agent_id`` (int): ID of current agent (ranging from 0 to N-1)
        - ``in_neigh`` (list): list of in-neighbors
        - ``out_neigh`` (list): list of out-neighbors

    Attributes:
        agent_id: ID of agent
        n_agents: Total number of agents
        in_neighbors: List of in-neighbors
        out_neighbors: List of in-neighbors
        current_pose: Current robot pose
        communicator: Neighboring communication facilities
    """

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None):
        """
        Args:
            pose_handler (str, optional): Pose handler (see
                :func:`~choirbot.utils.position_getter.pose_subscribe`). Defaults to None.
            pose_topic (str, optional): Topic where pose is published. Defaults to None.
        """
        super().__init__('guidance', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # get parameters
        self.agent_id = self.get_parameter('agent_id').value
        self.n_agents = self.get_parameter('N').value
        self.in_neighbors = self.get_parameter('in_neigh').value
        self.out_neighbors = self.get_parameter('out_neigh').value
        self.weights = self.get_parameter_or('weights').value

        # initialize pose subscription
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose, pose_callback)

        # initialize communicator
        self.communicator = self._instantiate_communicator()

    def _instantiate_communicator(self):
        return TimeVaryingCommunicator(self.agent_id, self.n_agents, self.in_neighbors,
            out_neighbors=self.out_neighbors)


class OptimizationGuidance(Guidance):
    """
    Base ROS node for guidance level with optimization features

    This Guidance class provides optimization-related features.
    This is an abstract class and is intended to be extended before instantiation.
    See :class:`Guidance` for information on the required ROS parameters.

    Attributes:
        optimizer: Optimizer class
        optimization_thread: Separate thread performing optimization
    """

    def __init__(self, optimizer: Optimizer, thread_t: Type[OptimizationThread],
            pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None):
        """
        Args:
            optimizer (:class:`~choirbot.optimizer.Optimizer`): optimizer to be run by thread
            thread_t (Type[:class:`~choirbot.guidance.optimization_thread.OptimizationThread`]):
                Type of optimization thread to be instantiated
            pose_handler (str, optional): Pose handler (see
                :func:`~choirbot.utils.position_getter.pose_subscribe`). Defaults to None.
            pose_topic (str, optional): Topic where pose is published. Defaults to None.
        """
        super().__init__(pose_handler, pose_topic, pose_callback)
        
        # save optimizer
        self.optimizer = optimizer

        # create guard condition to be triggered at end of optimization
        self._optimization_ended_gc = self.create_guard_condition(self._optimization_ended)

        # create and start optimization thread
        self.optimization_thread = thread_t(self, self.optimizer, self._optimization_ended_gc)
        self.optimization_thread.start()
    
    def _optimization_ended(self):
        raise NotImplementedError