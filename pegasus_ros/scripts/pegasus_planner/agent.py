import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler


class Agent(object):
    def __init__(self, a_id, hover_height):
        self.a_id = a_id
        self.hover_height = hover_height
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
            pose.pose.position.z = self.hover_height
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            poses.append(pose)
        self.path.header.frame_id = 'pegasus_map'
        self.path.poses = poses

    def publish_path(self):
        self.publishers['path'].publish(self.path)


"""
    def create_agent_markers(self):
        num_agents = len(self.mavros_namespaces)

        while True:
            is_empty = True
            for i in range(num_agents):
                if len(agentspose[i]['points']) > 0:
                    is_empty = False

            if is_empty:
                break

            for i in range(num_agents):
                if len(agentspose[i]['points']) == 0:
                    continue
                marker = Marker()
                marker.header.frame_id = 'pegasus_map'
                marker.ns = 'agents'
                marker.id = i
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.pose.orientation.w = 1
                x, y = agentspose[i]['points'].pop()

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 10
                marker.lifetime = rospy.Duration()
                marker.scale.x = 5
                marker.scale.y = 5
                marker.scale.z = 5
                marker.color.g = 0.7 + (i * 0.2) / num_agents
                marker.color.r = 0.5 + (i * 0.2) / num_agents
                marker.color.b = 0.1 + (i * 0.05) / num_agents
                marker.color.a = 1

                self.marker_pub.publish(marker)

                # for k in range(len(self.unvisitedPoints)):
                #  p = self.unvisitedPoints[k]
                #  if p.x == x and p.y == y:
                #    del self.unvisitedPoints[k]
                #    break
             # self.visitedPoints.append(Point(x, y, 0))
"""

