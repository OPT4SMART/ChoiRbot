from rclpy.node import Node
from .. import Pose
from .position_getter import pose_subscribe
from typing import Callable

class Visualizer(Node):

    def __init__(self, update_frequency: int= 25,
            pose_handler: str=None, pose_topic: str=None, pose_callback: Callable = None):
        super().__init__('visualizer', allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        self.agent_id = self.get_parameter('agent_id').value
        self.current_pose = Pose(None, None, None, None)
        self.subscription = pose_subscribe(pose_handler, pose_topic, self, self.current_pose, pose_callback)
        self.timer = self.create_timer(1.0/update_frequency, self.publish_data)

    def publish_data(self):
        raise NotImplementedError