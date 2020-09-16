import socket
import threading
from io import BytesIO
import cv2
import rospy
import Queue
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, translation_from_matrix
from geodesy import utm
import messages.pegasus_messages_pb2 as messages_pb2


def _set_offboard():
    request = messages_pb2.Request()
    request.command = messages_pb2.Command.SET_OFFBOARD
    return request


def _set_arm():
    request = messages_pb2.Request()
    request.command = messages_pb2.Command.SET_ARM
    return request


def _goto(param):
    request = messages_pb2.Request()
    request.command = messages_pb2.Command.GOTO
    pose = param[0]
    b_pose = BytesIO()
    pose.serialize(b_pose)
    request.req_data = b_pose.getvalue()
    return request


class Agent(object):
    def __init__(self, controller, a_id, namespace, address):
        self.calibration_path = Path()
        self.controller = controller
        self.a_id = a_id
        self.namespace = namespace
        self.local_map_name = '%s_map' % (namespace, )
        self.heartbeat_time = None
        self.command_id = 1
        self.mavros_state = MavrosState()
        self.local_pose = PoseStamped()
        self.global_position = NavSatFix()
        self._create_socket(address)
        self.recv_q = Queue.Queue()
        self.recv_thread = threading.Thread(target=self._recv_thread)
        self.recv_thread.start()
        self.publishers = {}
        self._register_publisher()
        self._create_calibration_path()
        self.calibration_poses = []
        self.command_thread = None
        self.last_command = (1, True)  # last_command_id, last_command_completed

    def _get_command_id(self):
        return self.command_id

    def _increment_and_get_command_id(self):
        self.command_id += 1
        return self._get_command_id()

    def _create_socket(self, address):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.connect(tuple(address))

    def _recv_thread(self):
        while not rospy.is_shutdown():
            data = self.socket.recv(1024)
            reply = messages_pb2.Reply()
            try:
                reply.ParseFromString(data)
                self.recv_q.put(reply)
            except Exception as e:
                rospy.logerr(str(e) + ' data: ' + str(self.data))
                return

    def _process_reply(self, reply):
        self.last_command = (
            reply.heartbeat_data.last_command_id,
            reply.heartbeat_data.last_command_completed
        )
        rospy.loginfo(self.last_command)
        self.mavros_state.deserialize(reply.heartbeat_data.mavros_state)
        self.local_pose.deserialize(reply.heartbeat_data.local_pose)
        self.global_position.deserialize(reply.heartbeat_data.gps_nav_sat)
        self.publishers['mavros_state'].publish(self.mavros_state)
        self.publishers['local_pose'].publish(self.local_pose)
        self.publishers['global_position'].publish(self.global_position)

    def _send_heartbeat(self):
        now = rospy.get_rostime()
        if self.heartbeat_time is not None and now - self.heartbeat_time < rospy.Duration(1):
            return
        request = messages_pb2.Request()
        request.timestamp = int(now.to_sec())
        request.command = messages_pb2.Command.HEARTBEAT
        self.socket.send(request.SerializeToString())
        self.heartbeat_time = now

    def _register_publisher(self):
        """
        commander_state_topic = '/pegasus_ros/%s/state/commander' % (self.namespace,)
        rospy.loginfo('Publisher to commander_state_topic: %s' % (commander_state_topic,))
        self.publishers['commander_state'] = rospy.Publisher(commander_state_topic, UInt8,
                                                             queue_size=1000)
        """
        mavros_state_topic = '/pegasus/%s/state/mavros' % (self.namespace,)
        rospy.loginfo('Publisher to mavros_state_topic: %s' % (mavros_state_topic,))
        self.publishers['mavros_state'] = rospy.Publisher(mavros_state_topic, MavrosState,
                                                          queue_size=1000)
        local_pose_topic = '/pegasus/%s/local_position/pose' % (self.namespace,)
        rospy.loginfo('Publisher to local_pose_topic: %s' % (local_pose_topic,))
        self.publishers['local_pose'] = rospy.Publisher(local_pose_topic, PoseStamped,
                                                        queue_size=1000)
        global_position_topic = '/pegasus/%s/global_position/global' % (self.namespace,)
        rospy.loginfo('Publisher to global_position_topic: %s' % (global_position_topic,))
        self.publishers['global_position'] = rospy.Publisher(global_position_topic, NavSatFix,
                                                             queue_size=1000)

        calibration_path_topic = '/pegasus/%s/path/calibration' % (self.namespace,)
        rospy.loginfo('Publisher to calibration path: %s' % (calibration_path_topic,))
        self.publishers['calibration_path'] = rospy.Publisher(calibration_path_topic, Path, latch=True,
                                                              queue_size=1000)

    def _create_calibration_path(self):
        side_length = self.controller.params['grid_size']
        z_height = self.controller.params['z_height']
        box_points = ((0., 0.), (side_length, 0.), (side_length, side_length), (0., side_length))
        # box_points = ((0, 0), (sideLength, 0))
        num_points = len(box_points)
        v1 = np.array(box_points)
        v2 = np.zeros((num_points, 2))
        v2[1:] = v1[0:-1]
        v1sv2 = v1 - v2
        direction = np.arctan2(v1sv2[:, 1], v1sv2[:, 0])
        for k in range(num_points):
            yaw = direction[k]
            q = quaternion_from_euler(0, 0, yaw)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = box_points[k][0]
            pose.pose.position.y = box_points[k][1]
            pose.pose.position.z = z_height + (self.a_id * 2)  # 2 meters distance
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self.calibration_path.header.frame_id = 'map'
            self.calibration_path.poses.append(pose)
            self.publishers['calibration_path'].publish(self.calibration_path)

    def _run_command_thread(self, command_id, command, param):
        request = None
        if command == messages_pb2.Command.SET_OFFBOARD:
            request = _set_offboard()
        elif command == messages_pb2.Command.SET_ARM:
            request = _set_arm()
        elif command == messages_pb2.Command.GOTO:
            request = _goto(param)
        else:
            return

        request.id = command_id
        request.timestamp = int(rospy.get_rostime().to_sec())

        # Busy wait for completion/retry
        rate = rospy.Rate(20)
        retry_counter = 0
        while not rospy.is_shutdown():
            if retry_counter == 0:
                self.socket.send(request.SerializeToString())
            retry_counter = (retry_counter + 1) % 100  # 5 seconds?
            if self.last_command[1] == True and self.last_command[0] == command_id:
                return
            rate.sleep()

    def run_command(self, command, param=()):
        command_id = self._increment_and_get_command_id()
        self.command_thread = threading.Thread(target=self._run_command_thread, args=(command_id, command, param))
        self.command_thread.start()

    def wait_for_command(self):
        if self.command_thread is not None:
            self.command_thread.join()

    def capture_calibration_pose(self):
        utm_pose = utm.fromLatLong(self.global_position.latitude,
                                   self.global_position.longitude,
                                   self.global_position.altitude)
        pose = (
            (self.local_pose.pose.position.x, self.local_pose.pose.position.y),
            (utm_pose.toPoint().x, utm_pose.toPoint().y)
        )
        rospy.loginfo(pose)
        self.calibration_poses.append(pose)

    def generate_transforms(self):
        size = len(self.calibration_poses)
        local_points = np.empty((size,2))
        global_points = np.empty((size,2))
        for i, correspondence_pts in enumerate(self.calibration_poses):
            local_points[i] = correspondence_pts[0]
            global_points[i] = correspondence_pts[1]
        map_points = np.subtract(global_points, self.controller.map_origin_point)
        rospy.loginfo(local_points)
        rospy.loginfo(map_points)
        h, mask = cv2.findHomography(local_points, map_points, method=cv2.RANSAC)
        homography = np.eye(4)
        homography[0:2, 0:2] = h[0:2, 0:2]
        homography[0:2, 3] = h[0:2, 2]
        homography[3, 0:2] = h[2, 0:2]
        homography[3, 3] = h[2, 2]
        transform_q = quaternion_from_matrix(homography)
        transform_t = translation_from_matrix(homography)
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.controller.global_map_name
        t.child_frame_id = self.local_map_name
        t.transform.rotation.x = transform_q[0]
        t.transform.rotation.y = transform_q[1]
        t.transform.rotation.z = transform_q[2]
        t.transform.rotation.w = transform_q[3]
        t.transform.translation.x = transform_t[0]
        t.transform.translation.y = transform_t[1]
        t.transform.translation.z = 0
        self.controller.tf_broadcaster.sendTransform(t)
        rospy.loginfo(transform_q)
        rospy.loginfo(transform_t)

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._send_heartbeat()
            try:
                reply = self.recv_q.get_nowait()
                self._process_reply(reply)
            except Queue.Empty:
                pass
            rate.sleep()
