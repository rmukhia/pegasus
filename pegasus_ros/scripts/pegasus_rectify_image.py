#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
import glob
import os
import cv2
from cv_bridge import CvBridge
import yaml
import piexif


class Rectifier(object):
    def __init__(self, params):
        super(Rectifier, self).__init__()
        self.in_dir = params['in_dir']
        self.out_dir = params['out_dir']
        self.agent = params['agent']
        self.camera_name = '%s_cam' % (self.agent,)
        self.camera_info = CameraInfo()
        self.cv_bridge = CvBridge()
        self.load_camera_info(params['camera_info_file'])
        self.image_publisher = rospy.Publisher('/' + self.camera_name + '/image_raw', Image, queue_size=10)
        self.info_publisher = rospy.Publisher('/' + self.camera_name + '/camera_info', CameraInfo, queue_size=10)
        self.image_sub = rospy.Subscriber('/' + self.camera_name + '/image_rect_color', Image, self._recv_rect_image)
        self.images = []  # (filename, image_data)
        self.exif = []

    def load_camera_info(self, filename):
        data = None
        with open(filename, 'r') as stream:
            try:
                data = yaml.safe_load(stream)
                self.camera_info.height = data['image_height']
                self.camera_info.width = data['image_width']
                self.camera_info.distortion_model = data['camera_model']
                self.camera_info.D = data['distortion_coefficients']['data']
                self.camera_info.K = data['camera_matrix']['data']
                self.camera_info.R = data['rectification_matrix']['data']
                self.camera_info.P = data['projection_matrix']['data']
            except yaml.YAMLError as e:
                rospy.logerr(e)
                rospy.signal_shutdown(e)

    def load_images(self):
        file_pattern = os.path.join(self.in_dir, self.agent + '*')
        filenames = glob.glob(file_pattern)
        for filename in filenames:
            self.images.append((filename, cv2.imread(filename)))

    def publish(self):
        if len(self.images) == 0:
            rospy.signal_shutdown("Processing over!")
            return
        img = self.cv_bridge.cv2_to_imgmsg(self.images[0][1], "bgr8")
        self.image_publisher.publish(img)
        self.info_publisher.publish(self.camera_info)

    def _recv_rect_image(self, image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        filename, _ = self.images.pop(0)
        out_file = os.path.join(self.out_dir, os.path.basename(filename))
        rospy.loginfo("Processing %s", filename)
        try:
            os.makedirs(self.out_dir)
        except OSError as e:
            rospy.logerr(e)
        except e:
            rospy.logerr(e)
            return
        params = [cv2.IMWRITE_JPEG_QUALITY, 100]
        cv2.imwrite(out_file, cv_image, params)
        piexif.transplant(filename, out_file)
        rospy.loginfo("Saved %s", out_file)


    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('Pegasus Rectify Image Started')
    in_dir = rospy.get_param('~in_dir')
    out_dir = rospy.get_param('~out_dir')
    agent = rospy.get_param('~agent')
    camera_info_file = rospy.get_param('~camera_info_file')
    rectifier = Rectifier({
        'in_dir': in_dir,
        'out_dir': out_dir,
        'agent': agent,
        'camera_info_file': camera_info_file,
    })

    rectifier.load_images()
    rectifier.spin()
    rospy.spin()
