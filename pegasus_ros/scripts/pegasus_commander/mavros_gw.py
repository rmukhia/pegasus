import rospy
import tf2_ros
from threading import Lock
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool


class MavrosGw(object):
    def __init__(self, mavros_namespace, map_transform, baselink_transform):
        self.tfBuffer = tf2_ros.Buffer()
        self.transformListener = tf2_ros.TransformListener(self.tfBuffer)
        self.mavrosNamespace = mavros_namespace
        self.mapTransform = map_transform
        self.baselinkTransform = baselink_transform
        self.mavrosSub = {}
        self.mavrosPub = {}
        self.mavrosService = {}
        self.data = {
            'state': None,
            'localPose': None,
            'globalGps': None,
        }

        self.lock = {
            'state': Lock(),
            'localPose': Lock(),
            'globalGps': Lock(),
        }

    def subscribe_and_publish(self):
        state_topic = '%s/state' % (self.mavrosNamespace,)
        rospy.loginfo('Subscriber to agent state: %s' % (state_topic,))
        self.mavrosSub['state'] = rospy.Subscriber(state_topic, State,
                                                   self.recv_mavros_state)

        local_position_topic = '%s/local_position/pose' % (self.mavrosNamespace,)
        rospy.loginfo('Subscriber to local position: %s' % (local_position_topic,))
        self.mavrosSub['localPositionSubscriber'] = rospy.Subscriber(local_position_topic, PoseStamped,
                                                                     self.recv_mavros_pose)

        global_gps_topic = '%s/global_position/global' % (self.mavrosNamespace,)
        rospy.loginfo('Subscriber to global gps: %s' % (global_gps_topic,))
        self.mavrosSub['globalGpsSubscriber'] = rospy.Subscriber(global_gps_topic,
                                                                 NavSatFix, self.recv_mavros_global_gps)

        set_point_topic = '%s/setpoint_position/local' % (self.mavrosNamespace,)
        rospy.loginfo('Publisher to setpoint_position: %s' % (set_point_topic,))
        self.mavrosPub['setPointPublisher'] = rospy.Publisher(set_point_topic, PoseStamped, queue_size=1000)

    def create_mavros_service_clients(self):
        set_mode_service = '%s/set_mode' % (self.mavrosNamespace,)
        arming_service = '%s/cmd/arming' % (self.mavrosNamespace,)
        rospy.wait_for_service(set_mode_service)
        rospy.loginfo('Set Mode service: %s' % (set_mode_service,))
        self.mavrosService['setMode'] = rospy.ServiceProxy(set_mode_service, SetMode)
        rospy.wait_for_service(arming_service)
        rospy.loginfo('Arming service: %s' % (arming_service,))
        self.mavrosService['arming'] = rospy.ServiceProxy(arming_service, CommandBool)

    def recv_mavros_state(self, data):
        with self.lock['state']:
            self.data['state'] = data

    def recv_mavros_pose(self, data):
        with self.lock['localPose']:
            self.data['localPose'] = data

    def recv_mavros_global_gps(self, data):
        with self.lock['globalGps']:
            self.data['globalGps'] = data

    def get_mavros_state(self):
        with self.lock['state']:
            data = self.data['state']
        return data

    def get_mavros_local_pose(self):
        with self.lock['localPose']:
            data = self.data['localPose']
        return data

    def get_mavros_global_gps(self):
        with self.lock['globalGps']:
            data = self.data['globalGps']
        return data

    def set_mavros_local_pose(self, pose):
        pose.header.frame_id = self.mapTransform
        pose.header.stamp.secs = 0
        pose.header.stamp.nsecs = 0
        self.mavrosPub['setPointPublisher'].publish(pose)

    def get_mavros_baselink_transform(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.mapTransform, self.baselinkTransform, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None
        return trans
