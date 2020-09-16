import cv2
import numpy as np
import threading

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix, translation_from_matrix

from messages.pegasus_messages_pb2 import Command

from pegasus_controller_state import PegasusControllerState


def is_at_position(self, localPose, x, y, z, offset):
    desired = np.array((x, y, z))
    pos = np.array((localPose.pose.position.x,
                    localPose.pose.position.y,
                    localPose.pose.position.z))
    return np.linalg.norm(desired - pos) < offset


class PegasusControllerThread(threading.Thread):
    def __init__(self, controller, thread_id):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.controller = controller
        self.state = PegasusControllerState.IDLE

    def _wait_for_agents(self):
        for agent in self.controller.agents:
            agent.wait_for_command()

    def _idle_state(self):
        rospy.loginfo('_idleState')
        """
        for agent in self.controller.agents:
            if agent['state'] is None:
                return
            if not agent['state'].connected:
                return
            if agent['path'] is None:
                return
            if agent['localPose'] is None:
                return
        """
        self.state = PegasusControllerState.OFFBOARD_MODE

    def _offboard_state(self):
        rospy.loginfo('_offboardState')
        for agent in self.controller.agents:
            agent.run_command(Command.SET_OFFBOARD)
        self._wait_for_agents()
        self.state = PegasusControllerState.ARMING

    def _arming_state(self):
        rospy.loginfo('_armingState')
        for agent in self.controller.agents:
            agent.run_command(Command.SET_ARM)
        self._wait_for_agents()
        self.state = PegasusControllerState.TAKE_OFF

    def _takeoff_state(self):
        rospy.loginfo('_takeOffState')
        pose = PoseStamped()
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 10;
        for agent in self.controller.agents:
            agent.run_command(Command.GOTO, (pose,))
        self._wait_for_agents()
        self.state = PegasusControllerState.CALIBRATE

    def _calibrate_state(self):
        rospy.loginfo('_calibrate_state')
        path_len = len(self.controller.agents[0].calibration_path.poses)
        for i in range(path_len):
            for agent in self.controller.agents:
                agent.run_command(Command.GOTO, (agent.calibration_path.poses[i], ))
            self._wait_for_agents()
            for agent in self.controller.agents:
                agent.capture_calibration_pose()
        self.state = PegasusControllerState.GENERATE_TRANSFORMS

    def _generate_transforms_state(self):
        rospy.loginfo('_generateTransformsState')
        for agent in self.controller.agents:
            agent.generate_transforms()

        self.state = PegasusControllerState.RUN

    def _run_state(self):
        # rospy.loginfo('_run_state')
        pass

    '''
        rospy.loginfo('_calibrateState')
        pathCompleted = True
        for agent in self.controller.agents.values():
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

        if all([agent['reached'] for agent in self.controller.agents.values()]):
            # all agents have reached the pose
            for agent in self.controller.agents.values():
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
            for agent in self.controller.agents.values():
                print('Capture signal')
                agent['captureLocalPose'] = True
                agent['captureGlobalGps'] = True
            self.calibCaptureTime = now

    def _generateTransformsState(self):
        rospy.loginfo('_generateTransformsState')

        for agent in self.controller.agents.values():
            localPoints = np.array([
                (
                    p.pose.position.x,
                    p.pose.position.y,
                    p.pose.position.z
                ) for p in agent['calibLocalPoses']])

            globalPoints = agent['calibGlobalGpses']

            # globalPoseUTM = np.array([(p.x, p.y, p.z) for p in globalGpsUTM])
            # Is this correct?
            mapPoints = np.subtract(globalPoints, self.controller.mapOrigin)

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
            trans = self.controller.tf_buffer.lookup_transform(localMap, 'map', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.loginfo(e)
            return None

        return tf2_geometry_msgs.do_transform_pose(pose, trans)

    def _runState(self):
        rospy.loginfo('_runState')
        pathCompleted = True
        reached = True
        for agent in self.controller.agents.values():
            if len(agent['path'].poses) <= agent['currentPoseInPath']:
                continue
            pathCompleted = False
            currentPose = agent['path'].poses[agent['currentPoseInPath']]
            transformedPose = self._transformMapToLocal(currentPose, agent)
            transformedPose.pose.position.z = self.controller.params['agentsHoverHeight']
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

        if all([agent['reached'] for agent in self.controller.agents.values()]):
            # all agents have reached the pose
            for agent in self.controller.agents.values():
                agent['currentPoseInPath'] += 1
                agent['reached'] = False

        if pathCompleted:
            # all agents have completed their path
            self.state = PegasusControllerState.COMPLETE
    '''
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.state == PegasusControllerState.IDLE:
                self._idle_state()
            elif self.state == PegasusControllerState.OFFBOARD_MODE:
                self._offboard_state()
            elif self.state == PegasusControllerState.ARMING:
                self._arming_state()
            elif self.state == PegasusControllerState.TAKE_OFF:
                self._takeoff_state()
            elif self.state == PegasusControllerState.CALIBRATE:
                self._calibrate_state()
            elif self.state == PegasusControllerState.GENERATE_TRANSFORMS:
                self._generate_transforms_state()
            elif self.state == PegasusControllerState.RUN:
                self._run_state()
            elif self.state == PegasusControllerState.COMPLETE:
                rospy.loginfo("Completed. Restart to run again")
            # Publish State
            # self.controller.statePublisher.publish(self.state)
            # self.controller.broadcastTransforms()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
