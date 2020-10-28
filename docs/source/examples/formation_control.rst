.. _examples_formationcontrol:

====================================================
Distributed formation control for unicycle vehicles
====================================================

In this page we show how to implement a distributed formation control algorithm
for a team of unicycle vehicles. The resulting algorithm is simulated with
Gazebo by using Turtlebot3 ground robots.
A reference for this example can be found in [MeEg10]_.


Prerequisites
----------------------------

We assume a working installation of **ChoiRbot** and Gazebo is available
(see the :ref:`installation page <installation>`),
and also that the Turtlebot3 ROS 2 files are installed.
Moreover, we assume the reader to be familiar with the basic concepts
of ROS 2, Python and **ChoiRbot**
(see the :ref:`quick start page <quickstart>`).


Maths of the problem
----------------------------

We consider driving a team of :math:`N` robots to a translationally
independent formation in the :math:`(x,y)` plane that satisfies a given
set of constraints.

System dynamics and communication graph
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
The considered algorithm models each robot as having single-integrator dynamics

.. math::

   \dot{x}_i = u_i,

where for all :math:`i \in \{1, \ldots, N\}`, :math:`x_i \in \mathbb{R}^2` is the
:math:`i`-th system state (i.e., position) and :math:`u_i \in \mathbb{R}^2`
is the :math:`i`-th system input (i.e., velocity).

We assume the robots communicate according to a given
undirected graph :math:`\mathcal{G} = (\mathcal{V}, \mathcal{E})`, where
:math:`\mathcal{V} = \{1, \ldots, N\}` is the set of robots and
:math:`\mathcal{E} \subset \mathcal{V} \times \mathcal{V}` is the set of
edges. If :math:`(i,j) \in \mathcal{E}`, then also :math:`(j,i) \in \mathcal{E}`
and robots :math:`i` and :math:`j` can send information to each other.
The set of neighbors of each agent :math:`i` is denoted by
:math:`\mathcal{N}_i = \{j \in \mathcal{V} \mid (i,j) \in \mathcal{E}\}`.

Formation specification and distributed control law
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The desired formation is assumed to be rigid (see also [MeEg10]_) and is
specified by a set of desired distances :math:`\{d_{ij}\}_{(i,j) \in \mathcal{E}}`
between any two communicating robots :math:`i` and :math:`j`.

The distributed control law applied by each robot :math:`i` is

.. math::
   u_i(t) = \sum_{j \in \mathcal{N}_i} (\|x_i(t) - x_j(t)\|^2 - d_{ij}^2) (x_j(t) - x_i(t)).


Implementation in **ChoiRbot**
--------------------------------

In order to implement the formation control example in **ChoiRbot**,
we consider the following nodes for each robot:

* a Team Guidance node that exchanges the current position with the neighbors
  and computes the input :math:`u_i(t)` with a certain frequency
* a Control node that converts the single-integrator input (vector velocity)
  into the corresponding suitable unicycle input (angular and linear velocity)

To run the simulation, we will also need to interface **ChoiRbot** with
Turtlebot3 robots in Gazebo. Finally, we will also need a launch file
and the executable scripts (as required by the **ChoiRbot** paradigm).

We analyze each of these components separately in the following subsections.

Team Guidance
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Team Guidance layer is responsible for the actual implementation of the
distributed control law outlined above. It is based on the abstract class
:class:`~choirbot.guidance.distributed_control.DistributedControlGuidance`,
which extends the basic :class:`~choirbot.guidance.Guidance` class as follows:

.. code-block:: python

   from geometry_msgs.msg import Vector3
   from choirbot.guidance import Guidance


   class DistributedControlGuidance(Guidance):

      def __init__(self, update_frequency: float, pos_handler: str=None, pos_topic: str=None):
         super().__init__(pos_handler, pos_topic)
         self.publisher_ = self.create_publisher(Vector3, 'velocity', 1)
         self.update_frequency = update_frequency
         self.timer = self.create_timer(1.0/update_frequency, self.control)

When the class is instantiated, the ``__init__`` method creates a publisher
for the velocity inputs and creates a timer that executes the method ``control``
with a user-defined frequency. Note that, since this class extends
:class:`~choirbot.guidance.Guidance`, it inherits several useful attributes:

* ``current_pose``, which always contains the most up-to-date robot pose and is
  periodically updated by the parent class;
* ``communicator``, which is an instance of the class
  :class:`~choirbot.communicator.Communicator` and provides methods
  for graph-based communication;
* ``in_neighbors`` and ``out_neighbors``, which are the lists of the
  robot's in- and out- neighbors (in this example the graph is undirected
  so the two lists are identical and are equal to :math:`\mathcal{N}_i`).

The main body of the class that is repeatedly executed is contained in the
``control`` method. The code is as follows:

.. code-block:: python

   def control(self):
      # exchange current position with neighbors
      data = self.communicator.neighbors_exchange(self.current_pose.position, self.in_neighbors, self.out_neighbors, False)

      # compute input
      u = self.evaluate_velocity(data)

      # send input to planner/controller
      self.send_input(u)

When the method is run, it first exchanges the current position with the neighbors
by calling :func:`~choirbot.communicator.Communicator.neighbors_exchange`,
which returns a dictionary with the positions received from the neighbors.
Then, it calls the method ``evaluate_velocity``, which computes the actual
value of :math:`u_i(t)`, and finally calls the method ``send_input``,
which sends the input to the controller node. The method ``evaluate_velocity``
is left unimplemented in order to allow for arbitrary control laws,
while the body of ``send_input`` is very simple and only publishes the
computed input on the ``velocity`` topic:

.. code-block:: python

   def send_input(self, u):
      msg = Vector3()

      msg.x = u[0]
      msg.y = u[1]
      msg.z = u[2]

      self.publisher_.publish(msg)

In order to implement the formation control law, the class
:class:`~choirbot.guidance.distributed_control.DistributedControlGuidance`
must be extended to override the ``evaluate_velocity`` method.
The child class :class:`~choirbot.guidance.distributed_control.FormationControlGuidance`
is as follows:

.. code-block:: python

   import numpy as np
   from numpy.linalg import norm

   class FormationControlGuidance(DistributedControlGuidance):

      def __init__(self, update_frequency: float, gain: float=0.1, pos_handler: str=None, pos_topic: str=None):
         super().__init__(update_frequency, pos_handler, pos_topic)
         self.formation_control_gain = gain
         self.weights = self.get_parameter('weights').value

      def evaluate_velocity(self, neigh_data):
         u = np.zeros(3)
         for ii, pos_ii in neigh_data.items():
               error = pos_ii - self.current_pose.position
               u += self.formation_control_gain*(norm(error)**2- self.weights[ii]**2) * error
         return u

As it can be seen from the ``__init__`` method, this class requires that
the ROS parameter ``weights`` is set. This parameter represents the desired
inter-robot distances :math:`\{d_{ij}\}_{(i,j) \in \mathcal{E}}` and must be
passed to each robot :math:`i` as a dictionary with each element having key
:math:`j` and value :math:`d_{ij}`.
The ``evaluate_velocity`` method is simply the implementation of
the distributed formation control law and returns :math:`u_i(t)`.

Unicycle control
~~~~~~~~~~~~~~~~~~~~~~~~~~~
The goal of the control node is to translate the vector velocity input
:math:`u_i(t)` into the corresponding unicycle inputs :math:`v_i(t)`
(linear velocity) and :math:`\omega_i(t)` (angular velocity).
This translation is performed according to the approach described in
[WiGl20]_ within the class :class:`~choirbot.controller.UnicycleVelocityController`.
The initialization block of the class is as follows:

.. code-block:: python

   from choirbot.controller import Controller
   from geometry_msgs.msg import Vector3, Twist
   import numpy as np


   class UnicycleVelocityController(Controller):

      def __init__(self, pos_handler: str=None, pos_topic: str=None):
         super().__init__(pos_handler, pos_topic)
         self.subscription = self.create_subscription(Vector3, 'velocity', self.control_callback, 1)
         self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
         self.yaw = 0.0
         self.yaw_old = 0.0
         self.yaw_old_old = 0.0

Since the main job of this class is to translate the vector velocity input
into a unicycle input, in the ``__init__`` method we simply create a publisher
and a subscription for the relative topics and we initialize the quantities
for the control translation scheme. The main job is performed by the subscription
callback method ``control_callback``, which implements the law described in
[WiGl20]_ and publishes the translated input in the ``cmd_vel`` topic.

Interfacing with Gazebo
~~~~~~~~~~~~~~~~~~~~~~~~~~~
In order to run the algorithm within the Gazebo simulation environment,
we first need to create the robots. To this end, we use the
``SpawnEntity`` service provided by the Gazebo ros factory plugin,
as suggested by `this thread <https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985>`_
of the ROS community.
This service requires the Gazebo process to be executed with the following
command, which will be embedded later directly in the launch file:

.. code-block:: bash

   gazebo -s libgazebo_ros_factory.so

(TODO describe script to spawn robot, moreover explain that robots receive commands
in the ``cmd_vel`` topic and that the updated odometry is retrieved by guidance class)

Launch file and executables
~~~~~~~~~~~~~~~~~~~~~~~~~~~

TODO

Running the simulation
-----------------------------

TODO

.. rubric:: References

.. [MeEg10] Mesbahi, M., Egerstedt, M. (2010). Graph theoretic methods in multiagent networks (Vol. 33). Princeton University Press.
.. [WiGl20] Wilson, S., Glotfelter, P., Wang, L., Mayya, S., Notomista, G., Mote, M., Egerstedt, M. (2020). The robotarium: Globally impactful opportunities, challenges, and lessons learned in remote-access, distributed control of multirobot systems. IEEE Control Systems Magazine, 40(1), 26-44.