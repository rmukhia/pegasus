import threading

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

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
            rospy.loginfo("Service call failed: %s", e)
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
            for agent in self.controller.agents:
                agent.grab_image()
        self.controller.state = State.COMPLETE

    def _complete_state(self):
        rospy.loginfo('_complete_state')
        for agent in self.controller.agents:
            agent.run_command(Command.SET_RETURN_TO_HOME)
        self._wait_for_agents()
        self.controller.state = State.IDLE

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
