#!/usr/bin/env python
import rospy
import socket
import os

from std_srvs.srv import Trigger, TriggerResponse


class PegasusVideoReceiver(object):
    def __init__(self, params):
        self.params = params
        self.address = (self.params['agent_ip'], self.params['agent_remote_port'])
        self.rx_port = self.params['udp_port']
        self.running = False
        self.interval = None
        self._create_socket(self.address)
        self.services = {}
        self._create_services()
        self.counter = 0

    def _create_socket(self, address):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.rx_port))
        self.socket.connect(address)

    def _create_services(self):
        self.services['start_capture'] = rospy.Service('start_capture', Trigger, self._start_capture)
        self.services['stop_capture'] = rospy.Service('stop_capture', Trigger, self._stop_capture)

    def _start_capture(self, request):
        rospy.loginfo('Start Capture')
        if not self.running:
            self.running = True
            return TriggerResponse(True, 'Capture started.')
        else:
            return TriggerResponse(False, 'Capture in progress.')

    def _stop_capture(self, request):
        rospy.loginfo('Stop Capture')
        if self.running:
            self.running = False
            return TriggerResponse(True, 'Capture stopped.')
        else:
            return TriggerResponse(False, 'Capture not started.')

    def _get_image(self):
        now = rospy.get_rostime()
        if self.interval is not None and now - self.interval < rospy.Duration(self.params['interval']):
            return
        self.socket.send('REQ')
        self.socket.settimeout(None)
        buf_size = 1024
        no_header = True
        self.interval = now
        while no_header:
            try:
                header = self.socket.recv(buf_size)
            except socket.timeout:
                break
            self.socket.settimeout(2)
            print header
            if header[0:3] == 'RES':
                n_fragments = int(header[4:])
                no_header = False
        if no_header:
            return
        data = ''
        for i in range(n_fragments):
            self.socket.settimeout(2)
            try:
                data += self.socket.recv(buf_size)
            except socket.timeout:
                return
        filename = os.path.join(self.params['save_directory'],
                                '%s-%s.jpeg' % (self.params['agent_name'], self.counter))
        f = open(filename, 'wb')
        f.write(data)
        f.close()
        self.counter += 1

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.running:
                self._get_image()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pegasus_video_receiver')
    rospy.loginfo('Starting pegasus_video_receiver...')
    agent_name = rospy.get_param('~agent_name')
    agent_ip = rospy.get_param('~agent_ip')
    agent_remote_port = rospy.get_param('~agent_remote_port')
    udp_port = rospy.get_param('~udp_port')
    interval = rospy.get_param('~interval')
    save_directory = rospy.get_param('~save_directory')
    pegasus_video_receiver = PegasusVideoReceiver({
        'agent_name': agent_name,
        'agent_ip': agent_ip,
        'agent_remote_port': agent_remote_port,
        'udp_port': udp_port,
        'interval': interval,
        'save_directory': save_directory,
    })
    pegasus_video_receiver.spin()
    rospy.spin()
