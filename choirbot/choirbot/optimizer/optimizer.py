from threading import Event


class Optimizer():

    def __init__(self, settings: dict=None):
        if settings is not None and not isinstance(settings, dict):
            raise ValueError("settings must be either a dictionary or None")
        
        self.settings = settings if settings is not None else {}
        self.guidance = None
        self._halt_event = None

    def initialize(self, guidance: 'Guidance', halt_event: Event=None):
        if guidance is None:
            raise ValueError("guidance must be a valid instance of Guidance")

        # store variables
        self.guidance = guidance
        self._halt_event = halt_event
    
    def optimize(self):
        raise NotImplementedError

    def get_result(self):
        raise NotImplementedError
    
    def get_cost(self):
        raise NotImplementedError