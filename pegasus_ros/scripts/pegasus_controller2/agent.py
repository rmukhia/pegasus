import rospy
import socket
import Queue
import threading

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt8

from pegasus_commander.states import StateEnum as CommanderState
from pegasus_controller2.path_helper import PathHelper

import messages.pegasus_messages_pb2 as messages_pb2


class Agent(object):
    def __init__(self, index, controller, namespace, address):
        self.index = index
        self.commander_state = CommanderState.IDLE
        self.address = address
        self.namespace = namespace
        self.socket = None
        self.controller = controller
        self.echo_rx_thread = threading.Thread(target=self.recv_thread)
        self._tx_heartbeat_time = None
        self.send_q = Queue.Queue()
        self.recv_q = Queue.Queue()
        self.publishers = {}
        self.path_helper = PathHelper(self)
        self.current_command_id = 0
        self.current_request = None

    def _tx(self):
        try:
            tx_data = self.send_q.get_nowait()
            self.socket.sendto(tx_data, self.address)
            self.send_q.task_done()
        except Queue.Empty:
            pass

    def _rx(self):
        now = rospy.get_rostime()
        rx_data = self.socket.recv(1024)
        self.recv_q.put((now, rx_data))

    def _rx_dispatcher(self):
        try:
            timestamp, rx_data = self.recv_q.get_nowait()
            self.recv_q.task_done()
        except Queue.Empty:
            return
        reply = messages_pb2.Reply()
        reply.ParseFromString(rx_data)
        if reply.status == messages_pb2.Status.HEART_BEAT_STATUS:
            self._rx_heartbeat(timestamp, reply)

    def _rx_heartbeat(self, timestamp, reply):
        self.commander_state = reply.heartbeat_data.commander_state
        self.commander_state_complete = reply.heartbeat_data.state_complete
        self.controller.state.set_complete(True)
        self.mavros_state = State()
        self.mavros_state.deserialize(reply.heartbeat_data.mavros_state)
        self.local_pose = PoseStamped()
        self.local_pose.deserialize(reply.heartbeat_data.local_pose)
        self.global_position = NavSatFix()
        self.global_position.deserialize(reply.heartbeat_data.gps_nav_sat)
        self.publishers['commander_state'].publish(self.commander_state)
        self.publishers['mavros_state'].publish(self.mavros_state)
        self.publishers['local_pose'].publish(self.local_pose)
        self.publishers['global_position'].publish(self.global_position)
        # Update
        if self.current_request.command_id == reply.command_id:
            self.current_request = None

    def _tx_heartbeat(self, now):
        if self._tx_heartbeat_time is not None and now - self._tx_heartbeat_time < rospy.Duration(2):
            return
        request = messages_pb2.Request()
        request.command = messages_pb2.Command.HEART_BEAT
        tx_data = request.SerializeToString()
        self.send_q.put(tx_data)
        self._tx_heartbeat_time = now

    def _tx_current_request(selfs, now):
        if self.current_request is None:
            return
        timestamp , request = self.current_request
        if timestamp is not None and now - timestamp < rospy.Duration(5):
            return
        self.current_request[0] = now
        self.send_q.put(self.current_request)

    def _update_and_get_command_id(self):
        id = self.command_id
        self.command_id += 1
        return id

    def create_agent_socket(self):
        self.socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

    def recv_thread(self):
        while not rospy.is_shutdown():
            self._rx()

    def start_rx(self):
        self.echo_rx_thread.daemon = True
        self.echo_rx_thread.start()

    def register_publisher(self):
        commander_state_topic = '/pegasus_ros/%s/state/commander' % (self.namespace,)
        rospy.loginfo('Publisher to commander_state_topic: %s' % (commander_state_topic,))
        self.publishers['commander_state'] = rospy.Publisher(commander_state_topic, UInt8,
                                                             queue_size=1000)
        mavros_state_topic = '/pegasus_ros/%s/state/mavros' % (self.namespace,)
        rospy.loginfo('Publisher to mavros_state_topic: %s' % (mavros_state_topic,))
        self.publishers['mavros_state'] = rospy.Publisher(mavros_state_topic, State,
                                                          queue_size=1000)
        local_pose_topic = '/pegasus_ros/%s/local_position/pose' % (self.namespace,)
        rospy.loginfo('Publisher to local_pose_topic: %s' % (local_pose_topic,))
        self.publishers['local_pose'] = rospy.Publisher(local_pose_topic, PoseStamped,
                                                        queue_size=1000)
        global_position_topic = '/pegasus_ros/%s/global_position/global' % (self.namespace,)
        rospy.loginfo('Publisher to global_position_topic: %s' % (global_position_topic,))
        self.publishers['global_position'] = rospy.Publisher(global_position_topic, NavSatFix,
                                                             queue_size=1000)

    def set_offboard(self):
        request = messages_pb2.Request()
        request.command = messages_pb2.Command.SET_OFFBOARD
        request.command_id = self._update_and_get_command_id()
        tx_data = request.SerializeToString()
        self.current_request = (rospy.get_rostime(), request)

    def set_arm(self):
        request = messages_pb2.Request()
        request.command = messages_pb2.Command.SET_ARM
        request.command_id = self._update_and_get_command_id()
        tx_data = request.SerializeToString()
        self.current_request = (rospy.get_rostime(), request)

    def takeoff(self):
        request = messages_pb2.Request()
        request.command = messages_pb2.Command.TAKE_OFF
        request.altitude = 10
        request.command_id = self._update_and_get_command_id()
        tx_data = request.SerializeToString()
        self.current_request = (rospy.get_rostime(), request)

    def spin(self, now):
        self._tx_heartbeat(now)
        self._tx_current_request(now)
        self._tx()
        self._rx_dispatcher()