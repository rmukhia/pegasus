import threading
import rospy
import numpy as np
import cv2

from geodesy import utm
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_matrix, translation_from_matrix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

from pegasus_controller_state import PegasusControllerState


class PegasusControllerThread(threading.Thread):
    def __init__(self, threadId, pegasusController):
        threading.Thread.__init__(self)
        self.threadId = threadId
        self.pegasusController = pegasusController
        self.state = PegasusControllerState.IDLE
        self.prepIterator = 100
        self.setModeLastTime = None
        self.calibCaptureTime = None

    def is_at_position(self, localPose, x, y, z, offset):
        desired = np.array((x, y, z))
        pos = np.array((localPose.pose.position.x,
                        localPose.pose.position.y,
                        localPose.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def _idleState(self):
        rospy.loginfo('_idleState')
        for agent in self.pegasusController.agents.values():
            if agent['state'] is None:
                return
            if not agent['state'].connected:
                return
            if agent['path'] is None:
                return
            if agent['localPose'] is None:
                return
        self.state = PegasusControllerState.OFFBOARD_MODE

    def _prepState(self):
        rospy.loginfo('_prepState')
        pose = PoseStamped()
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        for agent in self.pegasusController.agents.values():
            agent['setPointPublisher'].publish(pose)

        self.prepIterator -= 1
        if (self.prepIterator == 0):
            self.state = PegasusControllerState.OFFBOARD_MODE
            self.prepIterator = 100

    def _offboardState(self):
        now = rospy.get_rostime()
        # Run every 5 sec
        if self.setModeLastTime is not None and now - self.setModeLastTime < rospy.Duration(5.):
            return

        rospy.loginfo('_offboardState')
        for agent in self.pegasusController.agents.values():
            if agent['state'].mode != 'OFFBOARD':
                try:
                    res = agent['setModeService'](base_mode=0, custom_mode='OFFBOARD')
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                        self.setModeLastTime = now
                        return
                except rospy.ServiceException as e:
                    self.setModeLastTime = now
                    rospy.logerr(e)
                    return

        self.setModeLastTime = now
        self.state = PegasusControllerState.ARMING

    def _armingState(self):
        now = rospy.get_rostime()
        # Run every 5 sec
        if self.setModeLastTime is not None and now - self.setModeLastTime < rospy.Duration(5.):
            return

        rospy.loginfo('_armingState')
        for agent in self.pegasusController.agents.values():
            if not agent['state'].armed:
                try:
                    res = agent['armingService'](True)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                        self.setModeLastTime = now
                        return
                except rospy.ServiceException as e:
                    self.setModeLastTime = now
                    rospy.logerr(e)
                    return

        self.setModeLastTime = now
        self.state = PegasusControllerState.TAKE_OFF

    def _takeOffState(self):
        rospy.loginfo('_takeOffState')
        pose = PoseStamped()
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 10;
        for agent in self.pegasusController.agents.values():
            agent['setPointPublisher'].publish(pose)

        reached = True
        for agent in self.pegasusController.agents.values():
            if not self.is_at_position(agent['localPose'],
                                       pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, 0.5):
                reached = False

        if reached:
            self.state = PegasusControllerState.CALIBERATE

    def _caliberateState(self):
        rospy.loginfo('_calibrateState')
        pathCompleted = True
        for agent in self.pegasusController.agents.values():
            if len(agent['calibrationPath'].poses) <= agent['currentPoseInCalibrationPath']:
                continue
            pathCompleted = False
            currentPose = agent['calibrationPath'].poses[agent['currentPoseInCalibrationPath']]
            agent['setPointPublisher'].publish(currentPose)
            if not agent['reached'] and self.is_at_position(agent['localPose'],
                                                            currentPose.pose.position.x,
                                                            currentPose.pose.position.y,
                                                            currentPose.pose.position.z, 0.5):
                agent['reached'] = True

        if all([agent['reached'] for agent in self.pegasusController.agents.values()]):
            # all agents have reached the pose
            for agent in self.pegasusController.agents.values():
                agent['captureLocalPose'] = True
                agent['captureGlobalGps'] = True
                agent['currentPoseInCalibrationPath'] += 1
                agent['reached'] = False

        if pathCompleted:
            # all agents have completed their path
            self.state = PegasusControllerState.GENERATE_TRANSFORMS

        now = rospy.get_rostime()
        # capture pose every 2 seconds
        if self.calibCaptureTime is not None and now - self.calibCaptureTime < rospy.Duration(1.):
            for agent in self.pegasusController.agents.values():
                print('Capture signal')
                agent['captureLocalPose'] = True
                agent['captureGlobalGps'] = True
            self.calibCaptureTime = now

    def _generateTransformsState(self):
        rospy.loginfo('_generateTransformsState')

        for agent in self.pegasusController.agents.values():
            localPoints = np.array([
                (
                    p.pose.position.x,
                    p.pose.position.y,
                    p.pose.position.z
                ) for p in agent['calibLocalPoses']])

            globalPoints = agent['calibGlobalGpses']

            # globalPoseUTM = np.array([(p.x, p.y, p.z) for p in globalGpsUTM])
            # Is this correct?
            mapPoints = np.subtract(globalPoints, self.pegasusController.mapOrigin)

            h, mask = cv2.findHomography(localPoints[:, 0:2], mapPoints[:, 0:2], method=cv2.RANSAC)
            agent['homography'] = np.eye(4)
            agent['homography'][0:2, 0:2] = h[0:2, 0:2]
            agent['homography'][0:2, 3] = h[0:2, 2]
            agent['homography'][3, 0:2] = h[2, 0:2]
            agent['homography'][3, 3] = h[2, 2]

            agent['transform']['q'] = quaternion_from_matrix(agent['homography'])
            agent['transform']['t'] = translation_from_matrix(agent['homography'])

        self.state = PegasusControllerState.RUN

    def _transformMapToLocal(self, pose, agent):
        localMap = agent['localTransformMap']
        try:
            trans = self.pegasusController.tfBuffer.lookup_transform(localMap, 'map', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            return None

        return tf2_geometry_msgs.do_transform_pose(pose, trans)

    def _runState(self):
        rospy.loginfo('_runState')
        pathCompleted = True
        reached = True
        for agent in self.pegasusController.agents.values():
            if len(agent['path'].poses) <= agent['currentPoseInPath']:
                continue
            pathCompleted = False
            currentPose = agent['path'].poses[agent['currentPoseInPath']]
            transformedPose = self._transformMapToLocal(currentPose, agent)
            transformedPose.pose.position.z = self.pegasusController.params['agentsHoverHeight']
            if transformedPose is None:
                rospy.loginfo('Invalid transform')
                continue
            transformedPose.header.frame_id = agent['localTransformMap']
            agent['setPointPublisher'].publish(transformedPose)
            if not agent['reached'] and self.is_at_position(agent['localPose'],
                                                            transformedPose.pose.position.x,
                                                            transformedPose.pose.position.y,
                                                            transformedPose.pose.position.z, 0.5):
                agent['reached'] = True

        if all([agent['reached'] for agent in self.pegasusController.agents.values()]):
            # all agents have reached the pose
            for agent in self.pegasusController.agents.values():
                agent['currentPoseInPath'] += 1
                agent['reached'] = False

        if pathCompleted:
            # all agents have completed their path
            self.state = PegasusControllerState.COMPLETE

    def run(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            if self.state == PegasusControllerState.IDLE:
                self._idleState()

            elif self.state == PegasusControllerState.OFFBOARD_MODE:
                self._prepState()
                self._offboardState()
            elif self.state == PegasusControllerState.ARMING:
                self._armingState()
            elif self.state == PegasusControllerState.TAKE_OFF:
                self._takeOffState()
            elif self.state == PegasusControllerState.CALIBERATE:
                self._caliberateState()
            elif self.state == PegasusControllerState.GENERATE_TRANSFORMS:
                self._generateTransformsState()
            elif self.state == PegasusControllerState.RUN:
                self._runState()
            elif self.state == PegasusControllerState.COMPLETE:
                rospy.loginfo("Completed. Restart to run again")
            # Publish State
            self.pegasusController.statePublisher.publish(self.state)
            self.pegasusController.broadcastTransforms()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
