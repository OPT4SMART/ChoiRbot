# load default communicator for time-varying graphs
from .communicator import TimeVaryingCommunicator as Communicator

# load static and best-effort communicators
from .communicator import StaticCommunicator
from .communicator import BestEffortCommunicator