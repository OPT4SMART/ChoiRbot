import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.duration import Duration
from rclpy.qos import QoSProfile
from rclpy.qos import QoSLivelinessPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import ByteMultiArray, MultiArrayDimension

from threading import Event
import dill

from .callback_group import AuthorizationCallbackGroup
from .executor import SpinSomeExecutor
from disropt.communicators import Communicator as CommInterface


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class BestEffortCommunicator(CommInterface, metaclass=Singleton):
    # Best-effort communicator only supports asynchronous communication

    def __init__(self, agent_id, size, in_neighbors, out_neighbors = None, differentiated_topics = False):
        
        # initialize variables
        self.size = size
        self.rank = agent_id
        self.subscriptions = {}
        self.publishers = {}
        self.neighbors = None
        self.out_neighbors = out_neighbors if out_neighbors is not None else in_neighbors
        self.in_neighbors = in_neighbors
        self.callback_groups = self._get_callback_groups(in_neighbors)
        self.received_data = None
        self.current_label = 0
        self.differentiated_topics = differentiated_topics

        # initialize publisher and subscriptions
        self.qos_profile = self._getQoSProfile()
        self.node = Node('communicator')
        if not self.differentiated_topics:
            self.publisher = self.node.create_publisher(ByteMultiArray, 'communicator', self.qos_profile)
        else:
            for j in out_neighbors:
                self.publishers[j] = self.node.create_publisher(ByteMultiArray, 'communicator_{}'.format(j), self.qos_profile)
        self._subscribe(in_neighbors)
    
    # QoS profile for best-effort communication
    def _getQoSProfile(self):
        profile = QoSProfile(depth=10)
        profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        profile.liveliness = QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
        profile.deadline = Duration()
        profile.lifespan = Duration()
        return profile
    
    def _get_callback_groups(self, in_neighbors):
        return {j:None for j in in_neighbors}

    def _subscribe(self, neighbors):
        for i in neighbors:
            if i not in self.subscriptions:
                # decide topic name
                topic_name = '/agent_{}/communicator'.format(i)
                if self.differentiated_topics:
                    topic_name = '/agent_{}/communicator_{}'.format(i, self.rank)
                
                # create subscription
                self.subscriptions[i] = self.node.create_subscription(
                    ByteMultiArray,
                    topic_name,
                    lambda msg, node=i: self._subscription_callback(msg, node),
                    self.qos_profile, callback_group=self.callback_groups[i])

    def neighbors_send(self, obj, neighbors):
        """Send data to neighbors

        Args:
            obj (Any): object to send
            neighbors (list): list of neighbors
        """
        if self.differentiated_topics:
            for j in neighbors:
                if j not in self.publishers:
                    raise RuntimeError("Cannot send to {} since it is not an out-neighbor".format(j))

        # convert obj to string
        data = dill.dumps(obj)

        # prepare message
        msg = ByteMultiArray()
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = str(self.current_label)
        msg.layout.dim[0].size = len(data)
        msg.layout.dim[0].stride = len(data)
        msg.data = [bytes([x]) for x in data]

        # publish message
        if not self.differentiated_topics:
            self.publisher.publish(msg)
        else:
            for j in neighbors:
                self.publishers[j].publish(msg)
    
    def neighbors_receive_asynchronous(self, neighbors):
        """Receive data (if any) from neighbors. Overwrite if multiple messages are received.

        Args:
            neighbors (list): list of neighbors

        Returns:
            dict: dict containing received data
        """

        for j in neighbors:
            if j not in self.subscriptions:
                raise RuntimeError("Cannot receive from {} since it is not an in-neighbor".format(j))
        
        self.neighbors = neighbors
        
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
        # discard messages for inactive in-neighbors
        if node not in self.neighbors:
            return False

        # discard messages with old label
        if self.current_label > int(msg.layout.dim[0].label):
            return False

        # build up full byte string
        data = bytes(map(lambda x: x[0], msg.data))

        # decode message
        self.received_data[node] = dill.loads(data)

        return True


class TimeVaryingCommunicator(BestEffortCommunicator):

    def __init__(self, agent_id, size, in_neighbors, out_neighbors = None, synchronous_mode = True, differentiated_topics = False):
        
        # initialize additional variables for synchronous communication
        self.future = None
        self.synchronous_mode = synchronous_mode
        self.executor = SingleThreadedExecutor() if synchronous_mode else None

        # continue initialization
        super().__init__(agent_id, size, in_neighbors, out_neighbors, differentiated_topics)
    
    # QoS profile for reliable communication
    def _getQoSProfile(self):
        profile = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL)
        profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        profile.liveliness = QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
        profile.deadline = Duration()
        profile.lifespan = Duration()
        return profile
    
    def _get_callback_groups(self, in_neighbors):
        return {j:AuthorizationCallbackGroup() for j in in_neighbors}

    def neighbors_receive(self, neighbors, stop_event: Event = None):
        """Receive data from neighbors (waits until data are received from all neighbors)

        Args:
            neighbors (list): list of neighbors

        Returns:
            dict: dict containing received data associated to each neighbor in neighbors
        """
        if not self.synchronous_mode:
            raise Exception("Cannot perform synchronous receive because communicator is in asynchronous mode")
        
        for j in neighbors:
            if j not in self.subscriptions:
                raise RuntimeError("Cannot receive from {} since it is not an in-neighbor".format(j))
        
        self.neighbors = neighbors
        self.received_data = {} # initialize dictionary of received data
        self.future = Future()
        for k in self.callback_groups:
            self.callback_groups[k].give_authorization()

        # receive a single message from all neighbors (check Event every 'timeout' seconds)
        timeout = 0.2 if stop_event is not None else None
        while not self.future.done():
            rclpy.spin_until_future_complete(self.node, self.future, executor=self.executor, timeout_sec=timeout)

            # check if external event has been set
            if stop_event is not None and stop_event.is_set():
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
        if self.synchronous_mode:
            raise Exception("Cannot perform asynchronous receive because communicator is in synchronous mode")
        
        return super().neighbors_receive_asynchronous(neighbors)

    def _subscription_callback(self, msg, node):

        # perform actual callback and check status
        if super()._subscription_callback(msg, node):

            # notify reception from all neighbors if necessary
            if self.synchronous_mode and len(self.received_data) == len(self.neighbors):
                self.future.set_result(1)

    def neighbors_exchange(self, send_obj, in_neighbors, out_neighbors, dict_neigh, stop_event: Event = None):
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
            if not self.differentiated_topics:
                raise RuntimeError('Communicator must be initialized with differentiated topics in order to use dict_neigh=True')
            for j in out_neighbors:
                self.neighbors_send(send_obj[j], [j])
        data = self.neighbors_receive(in_neighbors, stop_event)

        return data

class StaticCommunicator(TimeVaryingCommunicator):

    def neighbors_send(self, obj):
        return super().neighbors_send(obj, self.out_neighbors)
    
    def neighbors_receive(self, stop_event: Event = None):
        return super().neighbors_receive(self.in_neighbors, stop_event)
    
    def neighbors_receive_asynchronous(self):
        return super().neighbors_receive_asynchronous(self.in_neighbors)
    
    def neighbors_exchange(self, send_obj, dict_neigh, stop_event: Event = None):
        return super().neighbors_exchange(send_obj, self.in_neighbors, self.out_neighbors, dict_neigh, stop_event)
