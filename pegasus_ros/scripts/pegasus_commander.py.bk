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


class PegasusCommander(object):
    def __init__(self, queue, params):
        self.cmdServerThread = threading.Thread(target=self.run_command_server)
        self.stateThread = threading.Thread(target=self.run_state_machine)
        self.queue = queue
        self.state = states.IdleState()
        self.path = {
            'path': None,
            'index': 0,
        }
        self.mavrosGw = MavrosGw(
            params['mavrosNamespace'],
            params['mapTransform'],
            params['baselinkTransform'])
        self.mavrosGw.subscribe_and_publish()
        self.mavrosGw.create_mavros_service_clients()

    def run_state_machine(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
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
        self.stateThread.daemon = True
        self.cmdServerThread.daemon = True
        self.stateThread.start()
        self.cmdServerThread.start()

    def set_path(self, path, index=-1):
        if path is not None:
            self.path['path'] = path
        self.path['index'] = index

    def get_path(self):
        return self.path


import pydevd_pycharm
pydevd_pycharm.settrace('localhost', port=7777, stdoutToServer=True, stderrToServer=True)


if __name__ == '__main__':
    rospy.init_node('pegasus_commander')
    rospy.loginfo('Starting pegasus_commander')
    mavros_namespace = rospy.get_param('~mavros_namespace')
    udp_port = rospy.get_param('~udp_port')
    map_transform = rospy.get_param('~map_transform')
    baselink_transform = rospy.get_param('~baselink_transform')
    rospy.loginfo('Mavros Namespace : %s', mavros_namespace)
    rospy.loginfo('Udp Port : %s', udp_port)

    q = Queue.Queue()
    commander = PegasusCommander(q, {
        'mavrosNamespace': mavros_namespace,
        'mapTransform': map_transform,
        'baselinkTransform': baselink_transform,
    })
    server_thread, server = get_message_server_thread('0.0.0.0', udp_port, q)
    server_thread.start()

    commander.spin()
    rospy.spin()
