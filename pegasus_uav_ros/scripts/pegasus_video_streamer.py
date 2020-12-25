#!/usr/bin/env python
import SocketServer
import copy
import cv2
import math
import os
import tempfile
import threading
from fractions import Fraction
from threading import Lock

import piexif
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix, Image

import messages.pegasus_messages_pb2 as messages_pb2
import pegasus_verify_data as verify_data

sock_buffsize = 1024

NUM_FRAMES = 7


class ImageContainer(object):
    def __init__(self, image_pkts=None):
        super(ImageContainer, self).__init__()
        self.total_pkts = 0
        self.pkts = []
        if image_pkts is not None:
            self.initialize(image_pkts)

    @staticmethod
    def parse_proto_buf(buf):
        """
        Call this method first.
        Then pass the result to other methods.
        """
        image_request = messages_pb2.ImageRequest()
        image_request.ParseFromString(buf)
        return image_request

    @staticmethod
    def get_err_pkt():
        response = messages_pb2.ImageRequest()
        response.command = messages_pb2.ImageCommand.ERR
        return response

    def reset(self):
        self.total_pkts = 0
        self.pkts = []

    def initialize(self, image_pkts):
        self.reset()
        self.pkts = image_pkts
        self.total_pkts = len(image_pkts)

    def get_meta_pkt(self):
        response = messages_pb2.ImageRequest()
        response.command = messages_pb2.ImageCommand.REPLY
        response.total_pkts = self.total_pkts
        return response

    def get_packet(self, pkt_no):
        response = messages_pb2.ImageRequest()
        if pkt_no < self.total_pkts:
            response.command = messages_pb2.ImageCommand.PKT
            response.pkt_no = pkt_no
            response.data = self.pkts[pkt_no]
            # rospy.loginfo(type(self.pkts[pkt_no]))
        else:
            response.command = messages_pb2.ImageCommand.ERR
        return response


class PegasusVideoStreamer(object):
    def __init__(self, params):
        self.params = params
        self.subscribers = {}
        self.data = {
            'global_gps': None,
            'image_raw': [],
            'image_ctr': 0,
        }
        self.lock = {
            'global_gps': Lock(),
            'image_raw': Lock(),
        }
        self.cv_bridge = CvBridge()
        self._subscribe()
        self.image_container = ImageContainer()
        self.pkts = []

    def _subscribe(self):
        global_gps_topic = '%s/global_position/global' % (self.params['mavros_namespace'],)
        rospy.loginfo('Subscriber to global gps: %s' % (global_gps_topic,))
        self.subscribers['global_gps'] = rospy.Subscriber(global_gps_topic,
                                                          NavSatFix, self._recv_global_gps)
        image_raw_topic = self.params['camera_topic']
        rospy.loginfo('Subscriber to image: %s' % (image_raw_topic,))
        self.subscribers['image_raw'] = rospy.Subscriber(image_raw_topic,
                                                         Image, self._recv_image_raw)

    def _recv_global_gps(self, data):
        with self.lock['global_gps']:
            self.data['global_gps'] = data

    def _recv_image_raw(self, data):
        with self.lock['image_raw']:
            if len(self.data['image_raw']) >= NUM_FRAMES:
                self.data['image_raw'].pop(0)
            self.data['image_raw'].append(data)
            self.data['image_ctr'] += 1

    @staticmethod
    def _get_degree_minute_second(value):
        fraction, degrees = math.modf(value)
        fraction, minutes = math.modf(fraction * 60)
        seconds = Fraction(fraction * 60).limit_denominator(10000)
        return (int(degrees), 1), (int(minutes), 1), (seconds.numerator, seconds.denominator)

    @staticmethod
    def _get_altitude(value):
        result = int(round(value))
        if result < 0:
            result = 0
        return result, 1

    def get_packet(self):
        ctr = self.data['image_ctr']
        rate = rospy.Rate(50)
        while ctr + NUM_FRAMES > self.data['image_ctr']:
            rate.sleep()
        images = []
        with self.lock['image_raw']:
            for image in self.data['image_raw']:
                images.append(copy.deepcopy(image))
        with self.lock['global_gps']:
            global_gps = copy.deepcopy(self.data['global_gps'])

        """
        with self.lock['image_raw']:
            image_raw = copy.deepcopy(self.data['image_raw'])
        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_raw, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return ['ERR']
        """

        cv_images = []
        for image in images:
            try:
                cv_images.append(self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8"))
            except CvBridgeError as e:
                rospy.logerr(e)
                return ['ERR']

        image = cv2.fastNlMeansDenoisingColoredMulti(cv_images, 3, 3)

        params = [cv2.IMWRITE_JPEG_QUALITY, self.params['jpeg_quality']]
        name = '%s_pegasus_video_streamer.jpg' % (os.path.dirname(self.params['mavros_namespace'])[1:],)
        filename = os.path.join(tempfile.gettempdir(), name)
        cv2.imwrite(filename, image, params)
        if global_gps is not None:
            exif_dict = piexif.load(filename)
            exif_dict['GPS'] = {
                piexif.GPSIFD.GPSLatitudeRef: "N",
                piexif.GPSIFD.GPSLatitude: self._get_degree_minute_second(global_gps.latitude),
                piexif.GPSIFD.GPSLongitudeRef: "W",
                piexif.GPSIFD.GPSLongitude: self._get_degree_minute_second(global_gps.longitude),
                piexif.GPSIFD.GPSAltitudeRef: 0,
                piexif.GPSIFD.GPSAltitude: self._get_altitude(global_gps.altitude + 50)
            }
            print (exif_dict)
            exif_bytes = piexif.dump(exif_dict)
            piexif.insert(exif_bytes, filename)
        data = []
        buf_size = sock_buffsize - 64 - verify_data.HASH_SIZE  # buffer size to fit in 1024 protobuf packet
        f = open(filename, 'rb')
        d = f.read(buf_size)
        while d:
            data.append(d)
            d = f.read(buf_size)
        f.close()
        return data


class UDPServerRequestHandler(SocketServer.BaseRequestHandler):
    def __init__(self, video_streamer):
        self.video_streamer = video_streamer

    def __call__(self, request, client_address, _server):
        h = UDPServerRequestHandler(self.video_streamer)
        SocketServer.BaseRequestHandler.__init__(h, request,
                                                 client_address, _server)

    def _clear_socket_buf(self, socket):
        sock_timeout = socket.gettimeout()
        socket.settimeout(0)
        rospy.loginfo('Streamer clearing udp buffer!')
        try:
            while socket.recv(1024):
                pass
        except:
            pass
        rospy.loginfo('Streamer cleared udp buffer!')
        socket.settimeout(sock_timeout)

    def _send_msg(self, socket, data):
        msg = verify_data.pack_msg(data)
        socket.sendto(msg, self.client_address)

    def handle(self):
        msg = self.request[0]
        socket = self.request[1]
        response = ImageContainer.get_err_pkt()
        try:
            data = verify_data.verify_msg(msg)
            image_request = ImageContainer.parse_proto_buf(data)
            # rospy.loginfo('Received:\n%s', image_request)
            if image_request.command == messages_pb2.ImageCommand.REQUEST:
                image_pkts = self.video_streamer.get_packet()
                self._clear_socket_buf(socket)
                self.video_streamer.image_container.initialize(image_pkts)
                response = self.video_streamer.image_container.get_meta_pkt()
            elif image_request.command == messages_pb2.ImageCommand.PKT:
                response = self.video_streamer.image_container.get_packet(image_request.pkt_no)
        except Exception as e:
            rospy.logerr(str(e))
        # rospy.loginfo('Replied:\n%s', response)
        self._send_msg(socket, response.SerializeToString())


class UDPServer(SocketServer.ThreadingMixIn, SocketServer.UDPServer):
    allow_reuse_address = True
    pass


def get_message_server_thread(host, port, video_streamer):
    _server = UDPServer((host, port), UDPServerRequestHandler(video_streamer))
    _server_thread = threading.Thread(target=_server.serve_forever)
    _server_thread.daemon = True
    return _server_thread, _server


if __name__ == '__main__':
    rospy.init_node('pegasus_video_streamer')
    rospy.loginfo('Starting pegasus_video_streamer...')
    mavros_namespace = rospy.get_param('~mavros_namespace')
    udp_port = rospy.get_param('~udp_port')
    camera_topic = rospy.get_param('~camera_topic')
    jpeg_quality = rospy.get_param('~jpeg_quality')

    pegasus_video_streamer = PegasusVideoStreamer({
        'mavros_namespace': mavros_namespace,
        'udp_port': udp_port,
        'camera_topic': camera_topic,
        'jpeg_quality': jpeg_quality,
    })

    server_thread, server = get_message_server_thread('0.0.0.0', udp_port, pegasus_video_streamer)
    server_thread.start()

    rospy.spin()
