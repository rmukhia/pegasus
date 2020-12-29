import Queue
import cv2
import numpy as np
import socket
import threading
from io import BytesIO

import rospy
from geodesy import utm
from geometry_msgs.msg import PoseStamped, TransformStamped
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, translation_from_matrix
from tf2_geometry_msgs import tf2_geometry_msgs

import messages.pegasus_messages_pb2 as messages_pb2
import pegasus_verify_data as verify_data
from pegasus_controller.state import State


def _set_offboard():
    request = messages_pb2.Request()
    request.command = messages_pb2.Command.SET_OFFBOARD
    return request


def _set_arm():
    request = messages_pb2.Request()
    request.command = messages_pb2.Command.SET_ARM
    return request


def _set_return_to_home():
    request = messages_pb2.Request()
    request.command = messages_pb2.Command.SET_RETURN_TO_HOME
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
    def __init__(self, controller, a_id, namespace, address, rx_port, calibration_size):
        self.calibration_path = Path()
        self.controller = controller
        self.a_id = a_id
        self.namespace = namespace
        self.local_map_name = '%s_map' % (namespace,)
        self.local_map_transform = TransformStamped()
        self.local_map_transform_calculated = False
        self.heartbeat_time = None
        self.command_id = 1
        self.mavros_state = MavrosState()
        self.local_pose = PoseStamped()
        self.global_position = NavSatFix()
        self.rx_port = rx_port
        self.server_address = address
        self.calibration_size = calibration_size
        self._create_socket(address)
        self.recv_q = Queue.Queue()
        self.recv_thread = threading.Thread(target=self._recv_thread)
        self.recv_thread.start()
        self.publishers = {}
        self.subscribers = {}
        self._register_publisher()
        self._register_subscribers()
        self._create_calibration_path()
        self.calibration_poses = []
        self.command_thread = None
        self.last_command = (1, True)  # last_command_id, last_command_completed
        self.traversed_points = None
        rospy.loginfo("%s: Initialized", self.namespace)

    def _get_command_id(self):
        return self.command_id

    def _increment_and_get_command_id(self):
        self.command_id += 1
        return self._get_command_id()

    def _create_socket(self, address):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("0.0.0.0", self.rx_port))
        rospy.loginfo('connecting to %s, bound to %s' % (address, self.rx_port))

    def _send_msg(self, data):
        msg = verify_data.pack_msg(data)
        self.socket.sendto(msg, tuple(self.server_address))

    def _recv_msg(self):
        msg = self.socket.recv(1024)
        data = verify_data.verify_msg(msg)
        return data

    def _broadcast_transforms(self):
        if self.local_map_transform_calculated:
            self.controller.tf_broadcaster.sendTransform(self.local_map_transform)

    def _recv_thread(self):
        while not rospy.is_shutdown():
            try:
                data = self._recv_msg()
                reply = messages_pb2.Reply()
                reply.ParseFromString(data)
                self.recv_q.put(reply)
            except verify_data.VerifyError as e:
                rospy.logerr(str(e))
            except Exception as e:
                rospy.logerr(str(e))

    def _process_reply(self, reply):
        self.last_command = (
            reply.heartbeat_data.last_command_id,
            reply.heartbeat_data.last_command_completed
        )
        rospy.loginfo(self.last_command)
        if reply.heartbeat_data.mavros_state is not None and len(reply.heartbeat_data.mavros_state) > 0:
            self.mavros_state.deserialize(reply.heartbeat_data.mavros_state)
            self.publishers['mavros_state'].publish(self.mavros_state)
        if reply.heartbeat_data.local_pose is not None and len(reply.heartbeat_data.local_pose) > 0:
            self.local_pose.deserialize(reply.heartbeat_data.local_pose)
            self.publishers['local_pose'].publish(self.local_pose)
        if reply.heartbeat_data.gps_nav_sat is not None and len(reply.heartbeat_data.gps_nav_sat) > 0:
            self.global_position.deserialize(reply.heartbeat_data.gps_nav_sat)
            self.publishers['global_position'].publish(self.global_position)
        if self.local_map_transform_calculated:
            transformed_pose = self.local_pose_to_global_pose(self.local_pose)
            self.publishers['global_pose'].publish(transformed_pose)

        # capture points in calibration process
        if self.controller.state == State.CALIBRATE:
            self.capture_calibration_pose()

    def _send_heartbeat(self):
        now = rospy.get_rostime()
        if self.heartbeat_time is not None and now - self.heartbeat_time < rospy.Duration(1):
            return
        request = messages_pb2.Request()
        request.timestamp = int(now.to_sec())
        request.command = messages_pb2.Command.HEARTBEAT
        self._send_msg(request.SerializeToString())
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
        local_pose_topic = '/pegasus/%s/local_position/pose_stamped' % (self.namespace,)
        rospy.loginfo('Publisher to local_pose_topic: %s' % (local_pose_topic,))
        self.publishers['local_pose'] = rospy.Publisher(local_pose_topic, PoseStamped,
                                                        queue_size=1000)
        global_position_topic = '/pegasus/%s/global_position/global' % (self.namespace,)
        rospy.loginfo('Publisher to global_position_topic: %s' % (global_position_topic,))
        self.publishers['global_position'] = rospy.Publisher(global_position_topic, NavSatFix,
                                                             queue_size=1000)

        global_pose_topic = '/pegasus/%s/global_position/pose_stamped' % (self.namespace,)
        rospy.loginfo('Publisher to global_pose_topic: %s' % (global_pose_topic,))
        self.publishers['global_pose'] = rospy.Publisher(global_pose_topic, PoseStamped,
                                                         queue_size=1000)

        calibration_path_topic = '/pegasus/%s/path/calibration' % (self.namespace,)
        rospy.loginfo('Publisher to calibration path: %s' % (calibration_path_topic,))
        self.publishers['calibration_path'] = rospy.Publisher(calibration_path_topic, Path, latch=True,
                                                              queue_size=1000)

    def _register_subscribers(self):
        path_topic = '/pegasus/%s/path' % (self.namespace,)
        rospy.loginfo('Subscriber to path topic: %s' % (path_topic,))
        self.subscribers['path'] = rospy.Subscriber(path_topic, Path, self._recv_path)

    def _recv_path(self, path):
        self.path = path

    def _create_calibration_path(self):
        z_height = self.controller.params['z_height']
        box_points = (
            (0., 0.),
            (self.calibration_size, 0.),
            (self.calibration_size, self.calibration_size),
            (0, self.calibration_size),
            (0., 0.))
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
        elif command == messages_pb2.Command.SET_RETURN_TO_HOME:
            request = _set_return_to_home()
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
            if self.controller.state == State.COMPLETE:
                return
            if retry_counter == 0:
                self._send_msg(request.SerializeToString())
            retry_counter = (retry_counter + 1) % 100  # 5 seconds?
            if self.last_command[1] is True and self.last_command[0] == command_id:
                return
            rate.sleep()

    def run_command(self, command, param=()):
        command_id = self._increment_and_get_command_id()
        self.command_thread = threading.Thread(target=self._run_command_thread, args=(command_id, command, param))
        self.command_thread.start()

    def wait_for_command(self):
        if self.command_thread is not None:
            self.command_thread.join()
            self.command_thread = None

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
        local_points = np.empty((size, 2))
        global_points = np.empty((size, 2))
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
        # normalize the quaternion d = norm(q) ;
        q_arr = np.array(transform_q)
        d = np.linalg.norm(q_arr)
        q_arr = q_arr/d
        transform_t = translation_from_matrix(homography)
        self.local_map_transform.header.stamp = rospy.Time.now()
        self.local_map_transform.header.frame_id = self.controller.global_map_name
        self.local_map_transform.child_frame_id = self.local_map_name
        self.local_map_transform.transform.rotation.x = q_arr[0]
        self.local_map_transform.transform.rotation.y = q_arr[1]
        self.local_map_transform.transform.rotation.z = q_arr[2]
        self.local_map_transform.transform.rotation.w = q_arr[3]
        self.local_map_transform.transform.translation.x = transform_t[0]
        self.local_map_transform.transform.translation.y = transform_t[1]
        self.local_map_transform.transform.translation.z = 0
        rospy.loginfo(q_arr)
        rospy.loginfo(transform_t)
        self.local_map_transform_calculated = True

    def global_pose_to_local_pose(self, global_pose):
        trans = self.controller.tf_buffer.lookup_transform(
            self.local_map_name,  # target
            self.controller.global_map_name,  # source
            rospy.Time(),
            rospy.Duration(10.0))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(global_pose, trans)
        transformed_pose.header.frame_id = self.local_map_name
        return transformed_pose

    def local_pose_to_global_pose(self, local_pose):
        trans = self.controller.tf_buffer.lookup_transform(
            self.controller.global_map_name,  # target
            self.local_map_name,  # source
            rospy.Time(),
            rospy.Duration(10.0))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(local_pose, trans)
        transformed_pose.header.frame_id = self.controller.global_map_name
        return transformed_pose

    def grab_image(self):
        image_service_path = '/%s/grab_image' % (self.namespace, )
        success = False
        while not success:
            rospy.loginfo('waiting for service %s' % (image_service_path, ))
            rospy.wait_for_service(image_service_path)
            try:
                image_service = rospy.ServiceProxy(image_service_path, Trigger)
                resp = image_service()
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s", e)
                return
            if not resp.success:
                rospy.logerr(resp)
            else:
                success = True

    def add_pose_to_traversed(self, pose):
        p = (pose.position.x, pose.position.y, pose.position.z)
        if self.traversed_points is None:
            self.traversed_points = np.array([p])
        else:
            self.traversed_points = np.append(self.traversed_points, [p], axis=0)

    def check_if_traversed(self, pose):
        if self.traversed_points is None:
            return False
        p = (pose.position.x, pose.position.y, pose.position.z)
        return np.equal(self.traversed_points, p).all(axis=1).any()

    def clear_traversed(self):
        self.traversed_points = None

    def spin(self):
        if self.controller.heartbeat_active:
            self._send_heartbeat()
            self._broadcast_transforms()
        try:
            reply = self.recv_q.get_nowait()
            self._process_reply(reply)
        except Queue.Empty:
            pass
