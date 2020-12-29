#!/usr/bin/env python
import threading

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from std_srvs.srv import Trigger, TriggerResponse


class Agent(object):
    def __init__(self, a_id, _height):
        super(Agent, self).__init__()
        self.a_id = a_id
        self.height = _height
        self.path = Path()
        self.path_sub = None
        self.distance_pub = False
        self.proximity_pub = False
        self.path_index = 0
        self.distance_covered = 0
        self.create_subscriber_and_publishers()
        self.is_broadcasting = False
        self.transform = TransformStamped()

    def create_subscriber_and_publishers(self):
        self.path_sub = rospy.Subscriber('/pegasus/%s/path' % (self.a_id,), Path, self.get_path)
        self.distance_pub = rospy.Publisher('/pegasus/path_validator/%s/distance_covered' % (self.a_id,), Float32,
                                            queue_size=100)
        self.proximity_pub = rospy.Publisher('/pegasus/path_validator/%s/proximity' % (self.a_id,), Float32,
                                            queue_size=100)

    def publish_proximity(self, min_distance):
        data = Float32()
        data.data = min_distance
        self.proximity_pub.publish(data)

    def get_path(self, data):
        self.path = data
        rospy.loginfo('Got path of length %s', len(data.poses))
        pose = self.path.poses[0].pose
        self.reset()
        self.transform.transform.translation.x = pose.position.x
        self.transform.transform.translation.y = pose.position.y
        self.transform.transform.translation.z = pose.position.z
        self.transform.transform.rotation.x = pose.orientation.x
        self.transform.transform.rotation.y = pose.orientation.y
        self.transform.transform.rotation.z = pose.orientation.z
        self.transform.transform.rotation.w = pose.orientation.w
        self.is_broadcasting = True

    def broadcast_transform(self):
        dist_msg = Float32()
        dist_msg.data = self.distance_covered
        self.distance_pub.publish(dist_msg)
        if not self.is_broadcasting:
            return
        bf = tf2_ros.TransformBroadcaster()
        self.transform.header.stamp = rospy.Time.now()
        self.transform.header.frame_id = 'pegasus_map'
        self.transform.child_frame_id = '%s_validator' % (self.a_id,)
        bf.sendTransform(self.transform)

    def reset(self):
        self.path_index = 0
        self.distance_covered = 0
        if self.is_broadcasting and len(self.path.poses) > 0:
            pose = self.path.poses[0].pose
            self.transform.transform.translation.x = pose.position.x
            self.transform.transform.translation.y = pose.position.y
            self.transform.transform.translation.z = pose.position.z
            self.transform.transform.rotation.x = pose.orientation.x
            self.transform.transform.rotation.y = pose.orientation.y
            self.transform.transform.rotation.z = pose.orientation.z
            self.transform.transform.rotation.w = pose.orientation.w

    def move_thread(self, period):
        t = threading.Thread(target=self.move_forward, args=(float(period),))
        t.start()
        return t

    def move_forward(self, period):
        br = tf2_ros.TransformBroadcaster()
        start_point = self.path.poses[self.path_index]
        end_point = self.path.poses[self.path_index + 1]
        self.path_index += 1
        diff_x = end_point.pose.position.x - start_point.pose.position.x
        diff_y = end_point.pose.position.y - start_point.pose.position.y
        frequency = 10
        rate = rospy.Rate(frequency)
        total_ticks = frequency * period
        increment_x = diff_x / total_ticks
        increment_y = diff_y / total_ticks
        x = start_point.pose.position.x
        y = start_point.pose.position.y
        ticks = 0
        while not rospy.is_shutdown() and ticks < total_ticks:
            self.transform.transform.translation.x = x
            self.transform.transform.translation.y = y
            self.transform.transform.translation.z = self.height
            self.transform.transform.rotation.x = end_point.pose.orientation.x
            self.transform.transform.rotation.y = end_point.pose.orientation.y
            self.transform.transform.rotation.z = end_point.pose.orientation.z
            self.transform.transform.rotation.w = end_point.pose.orientation.w
            x += increment_x
            y += increment_y
            self.distance_covered += (increment_x ** 2 + increment_y ** 2) ** 0.5
            ticks += 1
            rate.sleep()

    def has_ended(self):
        return self.path_index >= len(self.path.poses) - 1

    def get_position(self):
        return (
            self.transform.transform.translation.x,
            self.transform.transform.translation.y,
            self.transform.transform.translation.z)


class ControlStation(object):
    def __init__(self, position):
        super(ControlStation, self).__init__()
        self.transform = TransformStamped()
        self.transform.transform.translation.x = position[0]
        self.transform.transform.translation.y = position[1]
        self.transform.transform.translation.z = 0
        self.transform.transform.rotation.x = 0
        self.transform.transform.rotation.y = 0
        self.transform.transform.rotation.z = 0
        self.transform.transform.rotation.w = 1
        self.proximity_pub = rospy.Publisher('/pegasus/path_validator/control_station/proximity', Float32,
                                             queue_size=100)

    def publish_proximity(self, min_distance):
        data = Float32()
        data.data = min_distance
        self.proximity_pub.publish(data)

    def broadcast_transform(self):
        bf = tf2_ros.TransformBroadcaster()
        self.transform.header.stamp = rospy.Time.now()
        self.transform.header.frame_id = 'pegasus_map'
        self.transform.child_frame_id = 'control_station_validator'
        bf.sendTransform(self.transform)

    def get_position(self):
        return (
            self.transform.transform.translation.x,
            self.transform.transform.translation.y,
            self.transform.transform.translation.z)


class PathValidator(object):
    def __init__(self, _agent_names, _height, _control_station_position, _filename):
        super(PathValidator, self).__init__()
        self.agent_names = _agent_names
        self.agents = [Agent(agent, _height) for agent in self.agent_names]
        self.control_station = ControlStation(_control_station_position)
        self.filename = _filename
        self.validate_service = None
        self.create_service()
        t = threading.Thread(target=self.run_broadcasters)
        t.setDaemon(True)
        t.start()

    def create_service(self):
        self.validate_service = rospy.Service('validate_plan', Trigger, self._validate_plan)

    def run_broadcasters(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            for agent in self.agents:
                agent.broadcast_transform()
            self.control_station.broadcast_transform()
            rate.sleep()

    def _make_csv(self, index, _filename):
        pieces = [self.control_station] + self.agents
        num_pieces = len(pieces)
        positions = np.array([piece.get_position() for piece in pieces])
        for i, piece in enumerate(pieces):
            pos = np.array([piece.get_position()] * num_pieces)
            distances = np.sqrt(np.sum((positions - pos) ** 2, axis = 1))
            distances = distances[np.not_equal(distances, 0)]
            min_dist = np.min(distances)
            piece.publish_proximity(min_dist)

    def _validate_plan(self, request):
        for agent in self.agents:
            agent.reset()
        time_period = 3.  # period to cover one path
        index = 0
        while not rospy.is_shutdown():
            t = []
            self._make_csv(index, self.filename)
            for agent in self.agents:
                if not agent.has_ended():
                    t.append(agent.move_thread(time_period))
            if len(t) == 0:
                break
            for thread in t:
                thread.join()

        return TriggerResponse(True, 'Validated')


if __name__ == '__main__':
    agent_names = [
        'uav0',
        'uav1',
        'uav2'
    ]
    height = 40
    control_station_position = (0,0)
    rospy.init_node('pegasus_path_validator')
    rospy.loginfo('Starting pegasus path validator...')
    filename = rospy.get_param('~filename')
    path_validator = PathValidator(agent_names, 40, control_station_position, filename)
    rospy.spin()
