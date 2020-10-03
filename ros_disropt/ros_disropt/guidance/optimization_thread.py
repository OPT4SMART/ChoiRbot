from threading import Thread, Event

from .guidance import Guidance
from ..optimizer import Optimizer


class OptimizationThread(Thread):

    def __init__(self, old_thread: 'OptimizationThread', guidance: Guidance, optimizer: Optimizer):
        super().__init__()
        self._halt_event = Event()
        self.old_thread = old_thread
        self.guidance = guidance
        self.optimizer = optimizer
        self.thread_num = old_thread.thread_num + 1 if old_thread is not None else 0
        self.optimizer.initialize(guidance, self._halt_event)
    
    def run(self):
        # quit old thread if still alive
        if self.old_thread is not None and self.old_thread.is_alive():
            self.old_thread.halt()
            self.old_thread.join() # wait for previous thread to exit

        # perform optimization
        self.do_optimize()

        # trigger guard condition when optimization has finished
        self.guidance.optimization_gc.trigger() # TODO create attribute in Guidance (or create OptimizationGuidance class)
    
    def do_optimize(self):
        raise NotImplementedError

    def halt(self):
        self.guidance.get_logger().info('Halting previous optimization')
        self._halt_event.set()

    def get_result(self):
        raise NotImplementedError
