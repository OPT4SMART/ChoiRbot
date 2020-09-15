import threading
from rclpy.callback_groups import CallbackGroup

class AuthorizationCallbackGroup(CallbackGroup):
    # """
    # Throttle callbacks using a token bucket.
    # Callback groups are responsible for controlling when callbacks are allowed to be executed.
    # rclpy provides two groups: one which always allows a callback to be executed, and another which
    # allows only one callback to be executed at a time. If neither of these are sufficient then a
    # custom callback group should be used instead.
    # """

    def __init__(self):
        super().__init__()
        self.is_authorized = False
        self.is_permanent = False
        self.lock = threading.Lock()

    def give_authorization(self, permanent:bool=False):
        self.is_authorized = True
        self.is_permanent = permanent
    
    def draw_authorization(self):
        self.is_authorized = False
        self.is_permanent = False

    def can_execute(self, entity):
        # """
        # Ask group if this entity could be executed.
        # :param entity: A timer, subscriber, client, or service instance
        # :rtype bool: true if a callback can be executed
        # """
        return self.is_authorized

    def beginning_execution(self, entity):
        # """
        # Get permission from the group to execute a callback for an entity.
        # :param entity: A timer, subscriber, client, or service instance
        # :rtype bool: true if the executor has permission to execute it
        # """
        with self.lock:
            if self.is_authorized:
                if not self.is_permanent:
                    self.draw_authorization()
                return True
            return False

    def ending_execution(self, entity):
        # """
        # Notify group that a callback finished executing.
        # :param entity: A timer, subscriber, client, or service instance
        # """
        pass
