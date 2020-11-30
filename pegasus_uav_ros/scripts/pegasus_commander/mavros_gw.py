import rospy
import tf2_ros
from threading import Lock
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool


class MavrosGw(object):
    def __init__(self, mavros_namespace, map_transform, base_link_transform):
        self.tf_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.mavros_namespace = mavros_namespace
        self.map_transform = map_transform
        self.base_link_transform = base_link_transform
        self.mavros_sub = {}
        self.mavros_pub = {}
        self.mavros_service = {}
        self.data = {
            'state': None,
            'local_pose': None,
            'global_gps': None,
        }

        self.lock = {
            'state': Lock(),
            'local_pose': Lock(),
            'global_gps': Lock(),
        }

    def subscribe_and_publish(self):
        state_topic = '%s/state' % (self.mavros_namespace,)
        rospy.loginfo('Subscriber to agent state: %s' % (state_topic,))
        self.mavros_sub['state'] = rospy.Subscriber(state_topic, State,
                                                    self.recv_mavros_state)

        local_position_topic = '%s/local_position/pose' % (self.mavros_namespace,)
        rospy.loginfo('Subscriber to local position: %s' % (local_position_topic,))
        self.mavros_sub['local_position_subscriber'] = rospy.Subscriber(local_position_topic, PoseStamped,
                                                                        self.recv_mavros_pose)

        global_gps_topic = '%s/global_position/global' % (self.mavros_namespace,)
        rospy.loginfo('Subscriber to global gps: %s' % (global_gps_topic,))
        self.mavros_sub['global_gps_subscriber'] = rospy.Subscriber(global_gps_topic,
                                                                    NavSatFix, self.recv_mavros_global_gps)

        set_point_topic = '%s/setpoint_position/local' % (self.mavros_namespace,)
        rospy.loginfo('Publisher to setpoint_position: %s' % (set_point_topic,))
        self.mavros_pub['set_point_publisher'] = rospy.Publisher(set_point_topic, PoseStamped, queue_size=1000)

    def create_mavros_service_clients(self):
        set_mode_service = '%s/set_mode' % (self.mavros_namespace,)
        arming_service = '%s/cmd/arming' % (self.mavros_namespace,)
        rospy.wait_for_service(set_mode_service)
        rospy.loginfo('Set Mode service: %s' % (set_mode_service,))
        self.mavros_service['set_mode'] = rospy.ServiceProxy(set_mode_service, SetMode)
        rospy.wait_for_service(arming_service)
        rospy.loginfo('Arming service: %s' % (arming_service,))
        self.mavros_service['arming'] = rospy.ServiceProxy(arming_service, CommandBool)

    def recv_mavros_state(self, data):
        with self.lock['state']:
            self.data['state'] = data

    def recv_mavros_pose(self, data):
        with self.lock['local_pose']:
            self.data['local_pose'] = data

    def recv_mavros_global_gps(self, data):
        with self.lock['global_gps']:
            self.data['global_gps'] = data

    def get_mavros_state(self):
        with self.lock['state']:
            data = self.data['state']
        return data

    def get_mavros_local_pose(self):
        with self.lock['local_pose']:
            data = self.data['local_pose']
        return data

    def get_mavros_global_gps(self):
        with self.lock['global_gps']:
            data = self.data['global_gps']
        return data

    def set_mavros_local_pose(self, pose):
        pose.header.frame_id = self.map_transform
        pose.header.stamp.secs = 0
        pose.header.stamp.nsecs = 0
        self.mavros_pub['set_point_publisher'].publish(pose)

    def get_mavros_base_link_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.map_transform, self.base_link_transform, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None
        return trans
