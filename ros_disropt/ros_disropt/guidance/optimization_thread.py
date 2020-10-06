from threading import Thread, Event, Lock
from rclpy.guard_condition import GuardCondition

from ..optimizer import Optimizer


class OptimizationThread(Thread):

    def __init__(self, guidance: 'Guidance', optimizer: Optimizer, gc_end: GuardCondition):
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
        raise NotImplementedError

    def halt(self):
        self.guidance.get_logger().info('Halting optimization')
        self._halt_event.set()
    
    def _halt_if_optimizing(self):
        with self._lock:
            if self._is_optimizing:
                self.guidance.get_logger().info('Halting previous optimization')
                self._halt_event.set()
    
    def optimize(self):
        self._halt_if_optimizing()
        self.guidance.get_logger().info('Beginning optimization')
        self._begin_event.set()
    
    def quit(self):
        self._halt_if_optimizing()
        self.guidance.get_logger().info('Quitting optimization thread')
        self._quit_event.set()

############################
# OrEvent - by https://stackoverflow.com/a/12320352

def or_set(self):
    self._set()
    self.changed()

def or_clear(self):
    self._clear()
    self.changed()

def orify(e, changed_callback):
    e._set = e.set
    e._clear = e.clear
    e.changed = changed_callback
    e.set = lambda: or_set(e)
    e.clear = lambda: or_clear(e)

def OrEvent(*events):
    or_event = Event()
    def changed():
        bools = [e.is_set() for e in events]
        if any(bools):
            or_event.set()
        else:
            or_event.clear()
    for e in events:
        orify(e, changed)
    changed()
    return or_event