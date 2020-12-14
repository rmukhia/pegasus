import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from constraints import CONSTRAINTS


class Agent(object):
    def __init__(self, a_id):
        self.a_id = a_id
        self.initial_position = None
        self.path_plan = []  # { points: [], steps: int }
        self.path = Path()
        self.publishers = {}
        self.create_subscriber_and_publishers()

    def set_initial_position(self, initial_position):
        self.initial_position = initial_position

    def create_subscriber_and_publishers(self):
        self.publishers['path'] = rospy.Publisher('/pegasus/%s/path' % (self.a_id,), Path,
                                                  latch=True, queue_size=1000)

    def set_path_plan(self, path_plan):
        self.path_plan = path_plan

    def create_path(self):
        num_points = len(self.path_plan['points'])
        vectors1 = np.array(self.path_plan['points'])
        vectors2 = np.zeros((num_points, 2))
        vectors2[1:] = vectors1[0:-1]
        v1sv2 = vectors1 - vectors2
        direction = np.arctan2(v1sv2[:, 1], v1sv2[:, 0])
        poses = []
        for k in range(num_points):
            yaw = direction[k]
            q = quaternion_from_euler(0, 0, yaw)
            pose = PoseStamped()
            pose.header.frame_id = 'pegasus_map'
            x, y = self.path_plan['points'][k]
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = CONSTRAINTS['HEIGHT']
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            poses.append(pose)
        self.path.header.frame_id = 'pegasus_map'
        self.path.poses = poses
        rospy.loginfo("path len for %s: %s", self.a_id, len(poses))

    def get_path_markers(self):
        markers = []
        for i, pose_stamped in enumerate(self.path.poses):
            marker = Marker()
            marker.header.frame_id = 'pegasus_map'
            marker.ns = 'path_markers'
            marker.id = 0
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.pose.orientation.w = 1
            marker.text = '%s' % (i,)
            marker.lifetime = rospy.Duration()
            marker.scale.z = 10
            marker.color.g = 0
            marker.color.r = 1
            marker.color.b = 1
            marker.color.a = 1
            marker.pose = pose_stamped.pose
            markers.append(marker)
        return markers

    def get_start_end_markers(self):
        marker_start = Marker()
        marker_start.header.frame_id = 'pegasus_map'
        marker_start.ns = 'start_end'
        marker_start.id = 0
        marker_start.type = marker_start.TEXT_VIEW_FACING
        marker_start.action = marker_start.ADD
        marker_start.pose.orientation.w = 1
        marker_start.text = '%s start' % (self.a_id,)
        marker_start.lifetime = rospy.Duration()
        marker_start.scale.z = 10
        marker_start.color.g = 1
        marker_start.color.r = 1
        marker_start.color.b = 0
        marker_start.color.a = 1
        marker_start.pose = self.path.poses[0].pose

        marker_end = Marker()
        marker_end.header.frame_id = 'pegasus_map'
        marker_end.ns = 'start_end'
        marker_end.id = 0
        marker_end.type = marker_start.TEXT_VIEW_FACING
        marker_end.action = marker_start.ADD
        marker_end.pose.orientation.w = 1
        marker_end.text = '%s end' % (self.a_id,)
        marker_end.lifetime = rospy.Duration()
        marker_end.scale.z = 10
        marker_end.color.g = 1
        marker_end.color.r = 1
        marker_end.color.b = 0
        marker_end.color.a = 1
        marker_end.pose = self.path.poses[-1].pose
        return marker_start, marker_end

    def publish_path(self):
        self.publishers['path'].publish(self.path)