import cv2
import numpy as np
import threading

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from tf.transformations import quaternion_from_matrix, translation_from_matrix
from messages.pegasus_messages_pb2 import Command
from state import State


class ControllerThread(threading.Thread):
    def __init__(self, controller, thread_id):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.controller = controller

    def _wait_for_agents(self):
        for agent in self.controller.agents:
            agent.wait_for_command()

    # noinspection PyMethodMayBeStatic
    def _idle_state(self):
        rospy.loginfo_throttle(0.5, '_idleState')

    def _plan_state(self):
        rospy.loginfo('_plan_state')
        rospy.wait_for_service('start_planning')
        try:
            start_planning = rospy.ServiceProxy('start_planning', Trigger)
            resp = start_planning()
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)
            self.controller.state = State.IDLE
            return
        if not resp.success:
            self.controller.state = State.IDLE
            rospy.logerr(resp)
        else:
            self.controller.state = State.OFFBOARD_MODE

    def _offboard_state(self):
        rospy.loginfo('_offboard_state')
        for agent in self.controller.agents:
            agent.run_command(Command.SET_OFFBOARD)
        self._wait_for_agents()
        self.controller.state = State.ARMING

    def _arming_state(self):
        rospy.loginfo('_arming_state')
        for agent in self.controller.agents:
            agent.run_command(Command.SET_ARM)
        self._wait_for_agents()
        self.controller.state = State.TAKE_OFF

    def _takeoff_state(self):
        rospy.loginfo('_takeOff_state')
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = self.controller.params['z_height']
        for agent in self.controller.agents:
            agent.run_command(Command.GOTO, (pose,))
        self._wait_for_agents()
        self.controller.state = State.CALIBRATE

    def _calibrate_state(self):
        rospy.loginfo('_calibrate_state')
        path_len = len(self.controller.agents[0].calibration_path.poses)
        for i in range(path_len):
            for agent in self.controller.agents:
                agent.run_command(Command.GOTO, (agent.calibration_path.poses[i], ))
            self._wait_for_agents()
            for agent in self.controller.agents:
                agent.capture_calibration_pose()
        self.controller.state = State.GENERATE_TRANSFORMS

    def _generate_transforms_state(self):
        rospy.loginfo('_generate_transforms_state')
        for agent in self.controller.agents:
            agent.generate_transforms()
        self.controller.state = State.PRE_RUN

    def _pre_run_state(self):
        rospy.loginfo('_pre_run_state')
        for agent in self.controller.agents:
            global_pose = PoseStamped()
            global_pose.header.frame_id = self.controller.global_map_name
            global_pose.pose.position.x = 0
            global_pose.pose.position.y = 0
            z_height = self.controller.params['z_height']
            global_pose.pose.position.z = z_height + (agent.a_id * 2)  # 2 meters distance
            pose = agent.global_pose_to_local_pose(global_pose)
            pose.header.frame_id = 'map'
            agent.run_command(Command.GOTO, (pose,))
        self._wait_for_agents()
        self.controller.state = State.RUN

    def _run_state(self):
        rospy.loginfo('_run_state')
        rospy.sleep(10)
        running = True
        path_index = 0
        while running:
            running = False
            for agent in self.controller.agents:
                if len(agent.path.poses) <= path_index:
                    continue
                rospy.loginfo("%s pose index %s", agent.a_id, path_index)
                agent.run_command(Command.GOTO, (agent.global_pose_to_local_pose(agent.path.poses[path_index]), ))
                running = True
            path_index += 1
            self._wait_for_agents()
        self.controller.state = State.COMPLETE

    def _complete_state(self):
        rospy.loginfo('_complete_state')
        for agent in self.controller.agents:
            agent.run_command(Command.SET_RETURN_TO_HOME)
        self._wait_for_agents()
        self.controller.state = State.IDLE



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
            if self.controller.state == State.IDLE:
                self._idle_state()
            elif self.controller.state == State.PLAN:
                self._plan_state()
            elif self.controller.state == State.OFFBOARD_MODE:
                self._offboard_state()
            elif self.controller.state == State.ARMING:
                self._arming_state()
            elif self.controller.state == State.TAKE_OFF:
                self._takeoff_state()
            elif self.controller.state == State.CALIBRATE:
                self._calibrate_state()
            elif self.controller.state == State.GENERATE_TRANSFORMS:
                self._generate_transforms_state()
            elif self.controller.state == State.PRE_RUN:
                self._pre_run_state()
            elif self.controller.state == State.RUN:
                self._run_state()
            elif self.controller.state == State.COMPLETE:
                self._complete_state()
            # Publish State
            # self.controller.statePublisher.publish(self.controller.state)
            # self.controller.broadcastTransforms()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass
