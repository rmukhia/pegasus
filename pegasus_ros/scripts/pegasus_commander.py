#!/usr/bin/env python
"""
Pegasus Commander.
"""
import rospy
import threading
import Queue
from pegasus_commander import states
from pegasus_commander.runner import Runner
from pegasus_commander.mavros_gw import MavrosGw
from pegasus_commander.message_server import get_message_server_thread
import messages.pegasus_messages_pb2 as messages_pb2


class PegasusCommander(object):
    def __init__(self, queue, params):
        self.cmd_server_thread = threading.Thread(target=self.run_command_server)
        self.state_thread = threading.Thread(target=self.run_state_machine)
        self.queue = queue
        self.state = states.IdleState()
        self.path = {
            'path': None,
            'index': 0,
        }
        self.mavros_gw = MavrosGw(
            params['mavros_namespace'],
            params['map_transform'],
            params['baselink_transform'])
        self.mavros_gw.subscribe_and_publish()
        self.mavros_gw.create_mavros_service_clients()
        self.heartbeat_failure = int(params['heartbeat_failure'])
        self.heartbeat_time = rospy.get_rostime()
        self.command_ids = []

    def set_heartbeat_time(self, now):
        self.heartbeat_time = now

    def handle_hello_interval(self):
        """
        Return home if no heartbeat for heartbeat_failure seconds
        """
        now = rospy.get_rostime()
        if self.state.STATE in [states.IdleState.STATE, states.ReturnToHomeState.STATE, states.PrepState.STATE]:
            return
        if self.heartbeat_time is not None and now - self.heartbeat_time > rospy.Duration(self.heartbeat_failure):
            request = messages_pb2.Request()
            request.command = messages_pb2.Command.SET_RETURN_TO_HOME
            data = request.SerializeToString()
            rospy.loginfo('heartbeat unreceived')
            self.queue.put((None, None, data))

    def run_state_machine(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.handle_hello_interval()
            if not self.state.is_complete():
                self.state.run()
            else:
                self.state.step()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def run_command_server(self):
        while not rospy.is_shutdown():
            (socket, clientAddress, data) = self.queue.get()
            runner = Runner(self)
            runner.set_data(data).set_socket(socket).set_client_address(
                clientAddress)
            runner.start()
            self.queue.task_done()

    def spin(self):
        self.state_thread.daemon = True
        self.cmd_server_thread.daemon = True
        self.state_thread.start()
        self.cmd_server_thread.start()

    def set_path(self, path, index=-1):
        if path is not None:
            self.path['path'] = path
        self.path['index'] = index

    def get_path(self):
        return self.path


"""
# PyCharm debug server
import pydevd_pycharm
pydevd_pycharm.settrace('localhost', port=7777,
                        stdoutToServer=True, stderrToServer=True)
"""

if __name__ == '__main__':
    rospy.init_node('pegasus_commander')
    rospy.loginfo('Starting pegasus_commander')
    mavros_namespace = rospy.get_param('~mavros_namespace')
    heartbeat_failure = rospy.get_param('~heartbeat_failure')
    udp_port = rospy.get_param('~udp_port')
    map_transform = rospy.get_param('~map_transform')
    baselink_transform = rospy.get_param('~baselink_transform')
    rospy.loginfo('Mavros Namespace : %s', mavros_namespace)
    rospy.loginfo('Udp Port : %s', udp_port)

    q = Queue.Queue()
    commander = PegasusCommander(q, {
        'mavros_namespace': mavros_namespace,
        'map_transform': map_transform,
        'baselink_transform': baselink_transform,
        'heartbeat_failure': heartbeat_failure
    })
    server_thread, server = get_message_server_thread('0.0.0.0', udp_port, q)
    server_thread.start()

    commander.spin()
    rospy.spin()
