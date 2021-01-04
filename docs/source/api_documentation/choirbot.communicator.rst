.. _communicator:

==========================
Graph-based communication
==========================

Here we provide the API documentation for classes related to graph-based communication
capabilities. The main class providing this functionality is :class:`~choirbot.communicator.Communicator`,
but there are also additional internal classes implementing particular features.

Communicators
----------------------------

.. autoclass:: choirbot.communicator.BestEffortCommunicator
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: choirbot.communicator.StaticCommunicator
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: choirbot.communicator.TimeVaryingCommunicator
   :members:
   :undoc-members:
   :show-inheritance:

Additional internal classes
----------------------------

.. autoclass:: choirbot.communicator.callback_group.AuthorizationCallbackGroup
   :members:
   :undoc-members:
   :show-inheritance:

.. autoclass:: choirbot.communicator.executor.SpinSomeExecutor
   :members:
   :undoc-members:
   :show-inheritance: