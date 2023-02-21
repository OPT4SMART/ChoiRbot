from rclpy.node import Node
from typing import Callable
from .. import Pose

from ..utils.position_getter import pose_subscribe


class Controller(Node):

    def __init__(self, pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None):
        super().__init__('controller', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # get parameters
        self.agent_id = self.get_parameter('agent_id').value

        # initialize pose subscription
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose, pose_callback)
