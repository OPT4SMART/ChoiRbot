import rclpy
from choirbot.guidance.task import TaskGuidance, PositionTaskExecutor
from choirbot.optimizer import TaskOptimizer

def main():
    rclpy.init()

    # initialize task guidance
    opt_settings = {'max_iterations': 20}
    executor = PositionTaskExecutor()
    optimizer = TaskOptimizer(settings=opt_settings)
    
    guidance = TaskGuidance(optimizer, executor, None, 'pubsub', 'odom')

    rclpy.spin(guidance)
    rclpy.shutdown()
