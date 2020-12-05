#!/usr/bin/env python
import rospy
import socket
import os
from std_srvs.srv import Trigger, TriggerResponse
import messages.pegasus_messages_pb2 as messages_pb2

sock_timeout = 5 # 5 seconds
sock_bufsize = 1024 # buffer of 1024 size


class ImageContainer(object):
    def __init__(self, data=None):
        super(ImageContainer, self).__init__()
        self.total_pkts = 0
        self.pkt_no = 0
        self.pkts = []
        self.reset()

    @staticmethod
    def parse_proto_buf(buf):
        """
        Call this method first.
        Then pass the result to other methods.
        """
        image_request = messages_pb2.ImageRequest()
        image_request.ParseFromString(buf)
        return image_request

    def reset(self):
        self.total_pkts = []
        self.total_pkts = 0
        self.pkt_no = 0
        self.pkts = []

    @staticmethod
    def get_meta_pkt():
        request = messages_pb2.ImageRequest()
        request.command = messages_pb2.ImageCommand.REQUEST
        return request

    def process_meta_pkt(self, response):
        self.reset()
        self.total_pkts = response.total_pkts

    def get_request_pkt(self):
        request = messages_pb2.ImageRequest()
        request.command = messages_pb2.ImageCommand.PKT
        request.pkt_no = self.pkt_no
        return request

    def process_pkt(self, response):
        if response.pkt_no != self.pkt_no:
            raise Exception('%s, %s : pkt number does not match' % (response.pkt_no, self.pkt_no))
        self.pkts.append(response.data)
        self.pkt_no += 1

    def get_image(self):
        data = b''
        for d in self.pkts:
            data += d
        return d

    def write_to_file(self, filename):
        f = open(filename, 'wb')
        for data in self.pkts:
            f.write(data)
        f.close()


class PegasusVideoReceiver(object):
    def __init__(self, params):
        self.params = params
        self.address = (self.params['agent_ip'], self.params['agent_remote_port'])
        self.rx_port = self.params['udp_port']
        self.running = False
        self.interval = None
        self._create_socket()
        self.services = {}
        self._create_services()
        self.counter = 0
        self.pkt_no = 0
        self.data = []
        self.max_pkt_no = 0
        self.image_container = ImageContainer()

    def _create_socket(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.rx_port))

    def _create_services(self):
        self.services['grab_image'] = rospy.Service('grab_image', Trigger, self._grab_image)

    def _clear_socket_buf(self):
        self.socket.settimeout(0)
        rospy.loginfo('clearing')
        try:
            while self.socket.recv(sock_bufsize):
                pass
        except:
            pass
        rospy.loginfo('cleared')
        self.socket.settimeout(sock_timeout)

    def _get_image_meta(self):
        self._clear_socket_buf()
        image_request = self.image_container.get_meta_pkt()
        rospy.loginfo('Receiver Sending REQ')
        self.socket.sendto(image_request.SerializeToString(), self.address)
        self.socket.settimeout(sock_timeout)
        try:
            data = self.socket.recv(sock_bufsize)
            image_response = ImageContainer.parse_proto_buf(data)
            rospy.loginfo('Total packets %s', image_response.total_pkts)
            if image_response.command == messages_pb2.ImageCommand.REPLY:
                self.image_container.process_meta_pkt(image_response)
            else:
                return TriggerResponse(False, 'wrong header data')
        except socket.timeout:
            return TriggerResponse(False, 'socket timeout')

        return None

    def _get_image_data(self):
        self.socket.settimeout(sock_timeout)
        retries = 0
        while self.image_container.pkt_no < self.image_container.total_pkts:
            if retries > 10:
                break
            image_request = self.image_container.get_request_pkt()
            # rospy.loginfo(image_request)
            self.socket.sendto(image_request.SerializeToString(),self.address)
            try:
                data = self.socket.recv(sock_bufsize)
                image_response = ImageContainer.parse_proto_buf(data)
                # rospy.loginfo(image_response)
                if image_response.command == messages_pb2.ImageCommand.PKT:
                    self.image_container.process_pkt(image_response)
                    retries = 0
                elif image_response.command == messages_pb2.ImageCommand.ERR:
                    break
            except socket.timeout:
                rospy.loginfo('timeout')
                retries += 1
            except Exception as e:
                retries += 1
                rospy.loginfo(e)

        rospy.loginfo("Received total packets %s", self.image_container.pkt_no)

        if self.image_container.total_pkts != self.image_container.pkt_no:
            return TriggerResponse(False, "Error")

        filename = os.path.join(self.params['save_directory'],
                                '%s-%s.jpeg' % (self.params['agent_name'], self.counter))
        self.image_container.write_to_file(filename)
        self.counter += 1
        return TriggerResponse(True, 'Image captured.')

    def _grab_image(self, request):
        result = self._get_image_meta()
        if result is not None:
            return result
        return self._get_image_data()

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
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
    # pegasus_video_receiver.spin()
    rospy.spin()
