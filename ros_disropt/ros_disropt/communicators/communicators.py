import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import ByteMultiArray, MultiArrayDimension

from threading import Event
from disropt.communicators import Communicator
import dill

from .callback_groups import AuthorizationCallbackGroup
from .executors import SpinSomeExecutor


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class ROSCommunicator(Communicator, metaclass=Singleton):

    def __init__(self, agent_id, size, neighbors, synchronous_mode = True):
        super(ROSCommunicator, self).__init__()

        # initialize variables
        self.size = size
        self.rank = agent_id
        self.subscriptions = {}
        self.callback_groups = {}
        self.neighbors = None
        self.received_data = None
        self.future = None
        self.synchronous_mode = synchronous_mode
        self.executor = SingleThreadedExecutor() if synchronous_mode else None

        # initialize publisher and subscriptions
        self.qos_profile = self._getQoSProfile()
        self.node = Node('disropt_agent_{}'.format(agent_id))
        self.publisher = self.node.create_publisher(ByteMultiArray, 'disropt{}'.format(agent_id), self.qos_profile) # FIXME naming convention
        self._subscribe(neighbors)
    
    def _getQoSProfile(self):
        profile = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        profile.liveliness = QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
        profile.deadline = Duration()
        profile.lifespan = Duration()
        return profile

    def _subscribe(self, neighbors): # TODO time varying
        for i in neighbors:
            if i not in self.subscriptions:
                # use a specific callback group if in synchronous mode
                if self.synchronous_mode:
                    cbgroup = AuthorizationCallbackGroup()
                    self.callback_groups[i] = cbgroup
                else:
                    cbgroup = None
                
                # create subscription
                self.subscriptions[i] = self.node.create_subscription(
                    ByteMultiArray,
                    'disropt{}'.format(i), # FIXME naming convention
                    lambda msg, node=i: self._subscription_callback(msg, node),
                    self.qos_profile, callback_group=cbgroup)

    def neighbors_send(self, obj, neighbors):
        """Send data to neighbors

        Args:
            obj (Any): object to send
            neighbors (list): list of neighbors
        """
        # convert obj to string
        data = dill.dumps(obj)

        # prepare message
        msg = ByteMultiArray()
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = "label"
        msg.layout.dim[0].size = len(data)
        msg.layout.dim[0].stride = len(data)
        msg.data = [bytes([x]) for x in data]

        # publish message
        self.publisher.publish(msg)

    def neighbors_receive(self, neighbors, event: Event = None):
        """Receive data from neighbors (waits until data are received from all neighbors)

        Args:
            neighbors (list): list of neighbors

        Returns:
            dict: dict containing received data associated to each neighbor in neighbors
        """
        if not self.synchronous_mode:
            raise Exception("Cannot perform synchronous receive because communicator is in asynchronous mode")
        
        # self.subscribe(neighbors) # check subscriptions
        self.neighbors = neighbors
        self.received_data = {} # initialize dictionary of received data
        self.future = Future()
        for k in self.callback_groups:
            self.callback_groups[k].give_authorization()

        # receive a single message from all neighbors (check Event every 'timeout' seconds)
        timeout = 0.5 if event is not None else None
        while not self.future.done():
            rclpy.spin_until_future_complete(self.node, self.future, executor=self.executor, timeout_sec=timeout)

            # check if external event has been set
            if event is not None and event.is_set():
                # remove pending messages from topics
                for k in self.callback_groups:
                    self.callback_groups[k].give_authorization(permanent=True)

                self.synchronous_mode = False
                self.neighbors_receive_asynchronous(neighbors)
                self.synchronous_mode = True

                for k in self.callback_groups:
                    self.callback_groups[k].draw_authorization()
                break

        return self.received_data
    
    def neighbors_receive_asynchronous(self, neighbors):
        """Receive data (if any) from neighbors. Overwrite if multiple messages are received.

        Args:
            neighbors (list): list of neighbors

        Returns:
            dict: dict containing received data
        """
        if self.synchronous_mode:
            raise Exception("Cannot perform asynchronous receive because communicator is in synchronous mode")
        
        # initialize dictionary of received data
        self.received_data = {}

        # initialize specific executor
        executor = SpinSomeExecutor()
        timeout = False

        # perform all pending callbacks without waiting for further work
        while True:
            rclpy.spin_once(self.node, executor=executor, timeout_sec=0)

            # check for two consecutive timeouts
            if executor.timeout and timeout:
                break
            
            timeout = executor.timeout
        
        return self.received_data

    def _subscription_callback(self, msg, node):
        # build up full byte string
        data = bytes(map(lambda x: x[0], msg.data))

        # decode message
        self.received_data[node] = dill.loads(data)

        # notify reception from all neighbors if necessary
        if self.synchronous_mode and len(self.received_data) == len(self.neighbors):
            self.future.set_result(1)

    def neighbors_exchange(self, send_obj, in_neighbors, out_neighbors, dict_neigh, event: Event = None):
        """exchange information (synchronously) with neighbors

        Args:
            send_obj (any): object to send
            in_neighbors (list): list of in-neighbors
            out_neighbors (list): list of out-neighbors
            dict_neigh: True if send_obj contains a dictionary with different objects for each neighbor

        Returns:
            dict: dict containing received data associated to each neighbor in in-neighbors
        """

        if not dict_neigh:
            self.neighbors_send(send_obj, out_neighbors)
        else:
            raise NotImplementedError # TODO differentiated topics for each link
            # for j in out_neighbors:
            #     self.neighbors_send(send_obj[j], [j])
        data = self.neighbors_receive(in_neighbors, event)

        return data
