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


class PegasusVideoStreamer(object):
    def __init__(self, params):
        self.params = params
        self.subscribers = {}
        self.data = {
            'global_gps': None,
            'image_raw': None,
        }
        self.lock = {
            'global_gps': Lock(),
            'image_raw': Lock(),
        }
        self.cv_bridge = CvBridge()
        self._subscribe()

    def _subscribe(self):
        global_gps_topic = '%s/global_position/global' % (self.params['mavros_namespace'],)
        rospy.loginfo('Subscriber to global gps: %s' % (global_gps_topic,))
        self.subscribers['global_gps'] = rospy.Subscriber(global_gps_topic,
                                                          NavSatFix, self._recv_global_gps)
        image_raw_topic = '%s/image_raw' % (self.params['camera_namespace'],)
        rospy.loginfo('Subscriber to image: %s' % (image_raw_topic,))
        self.subscribers['image_raw'] = rospy.Subscriber(image_raw_topic,
                                                         Image, self._recv_image_raw)

    def _recv_global_gps(self, data):
        with self.lock['global_gps']:
            self.data['global_gps'] = data

    def _recv_image_raw(self, data):
        with self.lock['image_raw']:
            self.data['image_raw'] = data

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
        with self.lock['image_raw']:
            image_raw = copy.deepcopy(self.data['image_raw'])
        with self.lock['global_gps']:
            global_gps = copy.deepcopy(self.data['global_gps'])
        try:
            image = self.cv_bridge.imgmsg_to_cv2(image_raw)
        except CvBridgeError as e:
            rospy.logerr(e)
            return ['ERR']
        params = [cv2.IMWRITE_JPEG_QUALITY, self.params['jpeg_quality']]
        name = '%s-pegasus_video_streamer.jpg' % (os.path.dirname(self.params['mavros_namespace'])[1:],)
        filename = os.path.join(tempfile.gettempdir(), name)
        cv2.imwrite(filename, image, params)
        exif_dict = piexif.load(filename)
        exif_dict['GPS'] = {
            piexif.GPSIFD.GPSLatitudeRef: "N",
            piexif.GPSIFD.GPSLatitude: self._get_degree_minute_second(global_gps.latitude),
            piexif.GPSIFD.GPSLongitudeRef: "W",
            piexif.GPSIFD.GPSLongitude: self._get_degree_minute_second(global_gps.longitude),
            piexif.GPSIFD.GPSAltitudeRef: 0,
            piexif.GPSIFD.GPSAltitude: self._get_altitude(global_gps.altitude)
        }
        print (exif_dict)
        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, filename)
        data = []
        buf_size = 1024
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

    def handle(self):
        data = self.request[0]
        socket = self.request[1]
        rospy.loginfo('Received %s', str(data))
        if data[0:3] == 'REQ':
            data = self.video_streamer.get_packet()
            rospy.loginfo('Sending image!')
            rospy.loginfo(self.client_address)
            socket.sendto('RES %s' % (len(data),), self.client_address)
            for d in data:
                socket.sendto(d, self.client_address)


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
    camera_namespace = rospy.get_param('~cam_name')
    jpeg_quality = rospy.get_param('~jpeg_quality')

    pegasus_video_streamer = PegasusVideoStreamer({
        'mavros_namespace': mavros_namespace,
        'udp_port': udp_port,
        'camera_namespace': camera_namespace,
        'jpeg_quality': jpeg_quality,
    })

    server_thread, server = get_message_server_thread('0.0.0.0', udp_port, pegasus_video_streamer)
    server_thread.start()

    rospy.spin()
