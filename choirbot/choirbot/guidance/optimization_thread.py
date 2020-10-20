from threading import Thread, Event, Lock
from rclpy.guard_condition import GuardCondition

from . import guidance as guidance_module
from ..optimizer import Optimizer
from ..utils import OrEvent


class OptimizationThread(Thread):
    """
    Base class representing separate thread in which optimization tasks are performed.

    This is an abstract class and is intended to be extended before instantiation.

    Attributes:
        optimizer: Optimizer class
    """

    def __init__(self, guidance: 'guidance_module.Guidance', optimizer: Optimizer, gc_end: GuardCondition):
        """
        Args:
            guidance (:class:`~choirbot.guidance.Guidance`): Guidance object
            optimizer (:class:`~choirbot.optimizer.Optimizer`): Optimizer object
            gc_end (:class:`rclpy.guard_condition.GuardCondition`): Guard Condition triggered at end of each optimization
        """
        super().__init__()
        self._halt_event = Event()  # event to trigger optimization stop
        self._begin_event = Event() # event to trigger optimization start
        self._quit_event = Event()  # event to trigger thread quitting
        self._lock = Lock()
        self._is_optimizing = False # True if there is an optimization in progress
        self._gc_end = gc_end       # guard condition to be triggered at end of optimization
        self.guidance = guidance
        self.optimizer = optimizer
        self.optimizer.initialize(guidance, self._halt_event)
    
    def run(self):
        # event used to trigger either optimization stop or thread quitting
        begin_or_quit = OrEvent(self._begin_event, self._quit_event)

        # loop until quit event is set
        while not self._quit_event.is_set():
            begin_or_quit.wait()

            # check if optimization if event has been set
            if self._begin_event.is_set():

                # clear begin event and mark optimization as in progress
                self._begin_event.clear()
                with self._lock:
                    self._is_optimizing = True
                
                # start optimization
                self.do_optimize()

                # mark optimization as ended and (possibly) clear halt event
                with self._lock:
                    self._is_optimizing = False
                
                # clear halt event if previously set
                if self._halt_event.is_set():
                    self._halt_event.clear()
                else:
                    # otherwise trigger guard condition to notify end of optimization
                    self._gc_end.trigger()
        
        # clear quit event before actually quitting
        self._quit_event.clear()
    
    def do_optimize(self):
        """
        TODO
        """
        raise NotImplementedError

    def halt(self):
        """
        TODO
        """
        self.guidance.get_logger().info('Halting optimization')
        self._halt_event.set()
    
    def _halt_if_optimizing(self):
        with self._lock:
            if self._is_optimizing:
                self.guidance.get_logger().info('Halting previous optimization')
                self._halt_event.set()
    
    def optimize(self):
        """
        TODO
        """
        self._halt_if_optimizing()
        self.guidance.get_logger().info('Beginning optimization')
        self._begin_event.set()
    
    def quit(self):
        """
        TODO
        """
        self._halt_if_optimizing()
        self.guidance.get_logger().info('Quitting optimization thread')
        self._quit_event.set()
