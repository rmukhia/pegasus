#!/usr/bin/env python
import math
import rospy
import cv2
import yaml

if __name__ == '__main__':
    rospy.init_node('pegasus_fov')
    rospy.loginfo('Starting pegasus_fov')
    camera_file = rospy.get_param('~camera_file')
    dist = rospy.get_param('~dist')
    rospy.loginfo('Opening file %s', camera_file)
    with open(camera_file) as yaml_file:
        data = yaml.load(yaml_file)
    camera_matrix = data['camera_matrix']['data']
    image_height = data['image_height']
    image_width = data['image_width']
    camera_name = data['camera_name']
    # fx
    fov_x = 2 * math.atan(image_width/(2 * camera_matrix[0]))
    #fy
    fov_y = 2 * math.atan(image_height/(2 * camera_matrix[4]))

    grid_x = math.tan(fov_x/2) * dist
    grid_y = math.tan(fov_y/2) * dist

    rospy.loginfo("distance %s, fov_x %s, fov_y %s, grid size x %s, grid size y %s",
                  dist, fov_x, fov_x, grid_x * 2, grid_y * 2)
