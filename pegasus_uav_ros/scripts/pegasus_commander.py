#!/usr/bin/env python
"""
Pegasus Commander.
"""
import rospy
import threading
import Queue

from geometry_msgs.msg import PoseStamped
from pegasus_commander.runner import Runner, create_return_to_home_request
from pegasus_commander.mavros_gw import MavrosGw
from pegasus_commander.message_server import get_message_server_thread


class PegasusCommander(object):
    def __init__(self, queue, params):
        self.namespace = params['mavros_namespace']
        self.cmd_server_thread = threading.Thread(target=self.run_command_server)
        self.cmd_server_lock = threading.Lock()
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0
        self.current_pose.pose.position.y = 0
        self.current_pose.pose.position.z = 0
        self.running = False
        self.last_command = {
            'command': 1,
            'completed': True,

        }
        self.queue = queue

        self.mavros_gw = MavrosGw(
            params['mavros_namespace'],
            params['map_transform'],
            params['base_link_transform'])

        self.heartbeat_time = None
        self.mavros_gw.subscribe_and_publish()
        self.mavros_gw.create_mavros_service_clients()

    def set_last_command(self, command_id, completed):
        self.last_command = {
            'command': command_id,
            'completed': completed
        }

    def get_last_command(self):
        return self.last_command['command'], self.last_command['completed']

    def run_command_server(self):
        while not rospy.is_shutdown():
            (socket, clientAddress, data) = self.queue.get()
            runner = Runner(self)
            runner.set_data(data).set_socket(socket).set_client_address(
                clientAddress)
            runner.start()
            self.queue.task_done()

    def heartbeat_check(self):
        now = rospy.get_rostime()
        if self.heartbeat_time is not None and now - self.heartbeat_time  > rospy.Duration(15):
            # we lost heartbeat
            # empty command queue
            while not self.queue.empty():
                self.queue.get()
            rth_request = create_return_to_home_request()
            self.queue.put(rth_request)
            self.running = False

    def spin(self):
        self.cmd_server_thread.start()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if not self.cmd_server_lock.locked() and self.running:
                self.mavros_gw.set_mavros_local_pose(self.current_pose)
            self.heartbeat_check()
            rate.sleep()

"""
import pydevd_pycharm
pydevd_pycharm.settrace('localhost', port=7777, stdoutToServer=True, stderrToServer=True)
"""

if __name__ == '__main__':
    rospy.init_node('pegasus_commander')
    rospy.loginfo('Starting pegasus_commander')
    mavros_namespace = rospy.get_param('~mavros_namespace')
    udp_port = rospy.get_param('~udp_port')
    map_transform = rospy.get_param('~map_transform')
    base_link_transform = rospy.get_param('~base_link_transform')
    rospy.loginfo('Mavros Namespace : %s', mavros_namespace)
    rospy.loginfo('Udp Port : %s', udp_port)

    q = Queue.Queue()
    commander = PegasusCommander(q, {
        'mavros_namespace': mavros_namespace,
        'map_transform': map_transform,
        'base_link_transform': base_link_transform,
    })
    server_thread, server = get_message_server_thread('0.0.0.0', udp_port, q)
    server_thread.start()

    commander.spin()
    rospy.spin()
