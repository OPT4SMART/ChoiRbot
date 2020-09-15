from rclpy import Context
from rclpy.executors import Executor, ShutdownException, TimeoutException

class SpinSomeExecutor(Executor):
    """Runs callbacks and remembers when timeouts are hit"""

    def __init__(self, *, context: Context = None) -> None:
        super().__init__(context=context)
        self.timeout = None

    def spin_once(self, timeout_sec: float = None) -> None:
        self.timeout = False
        try:
            handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
        except ShutdownException:
            pass
        except TimeoutException:
            self.timeout = True
        else:
            handler()
            if handler.exception() is not None:
                raise handler.exception()