import threading
import copy
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

    def _wait_for_agents(self, agents):
        for agent in agents:
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
            return
        self.controller.state = State.PREP

    def _prep_state(self):
        rospy.loginfo('_prep_state')
        rospy.wait_for_service('start_heartbeat')
        try:
            start_heartbeat = rospy.ServiceProxy('start_heartbeat', Trigger)
            resp = start_heartbeat()
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
            self.controller.state = State.IDLE
            return
        if not resp.success:
            self.controller.state = State.IDLE
            rospy.logerr(resp)
            return
        if self.controller.state != State.COMPLETE:
            self.controller.state = State.OFFBOARD_MODE

    def _offboard_state(self):
        rospy.loginfo('_offboard_state')
        agents = []
        for agent in self.controller.agents:
            agent.run_command(Command.SET_OFFBOARD)
            agents.append(agent)
        self._wait_for_agents(agents)
        if self.controller.state != State.COMPLETE:
            self.controller.state = State.ARMING

    def _arming_state(self):
        rospy.loginfo('_arming_state')
        agents = []
        for agent in self.controller.agents:
            agent.run_command(Command.SET_ARM)
            agents.append(agent)
        self._wait_for_agents(agents)
        if self.controller.state != State.COMPLETE:
            self.controller.state = State.TAKE_OFF

    def _takeoff_state(self):
        rospy.loginfo('_takeOff_state')
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = self.controller.params['z_height']
        agents = []
        for agent in self.controller.agents:
            agent.run_command(Command.GOTO, (pose,))
            agents.append(agent)
        self._wait_for_agents(agents)
        if self.controller.state != State.COMPLETE:
            self.controller.state = State.CALIBRATE

    def _calibrate_state(self):
        rospy.loginfo('_calibrate_state')
        path_len = len(self.controller.agents[0].calibration_path.poses)
        agents = []
        for i in range(path_len):
            if self.controller.state == State.COMPLETE:
                break
            for agent in self.controller.agents:
                agent.run_command(Command.GOTO, (agent.calibration_path.poses[i], ))
                agents.append(agent)
            self._wait_for_agents(agents)
            for agent in self.controller.agents:
                agent.capture_calibration_pose()
        if self.controller.state != State.COMPLETE:
            self.controller.state = State.GENERATE_TRANSFORMS

    def _generate_transforms_state(self):
        rospy.loginfo('_generate_transforms_state')
        for agent in self.controller.agents:
            agent.generate_transforms()
        if self.controller.state != State.COMPLETE:
            self.controller.state = State.PRE_RUN

    def _pre_run_state(self):
        rospy.loginfo('_pre_run_state')
        agents = []
        for agent in self.controller.agents:
            global_pose_stamped = PoseStamped()
            global_pose_stamped.header.frame_id = self.controller.global_map_name
            global_pose_stamped.pose.position.x = 0
            global_pose_stamped.pose.position.y = 0
            z_height = self.controller.params['z_height']
            global_pose_stamped.pose.orientation.x = 0
            global_pose_stamped.pose.orientation.y = 0
            global_pose_stamped.pose.orientation.z = 0
            global_pose_stamped.pose.orientation.w = 1
            global_pose_stamped.pose.position.z = z_height + (agent.a_id * 2)  # 2 meters distance
            pose_stamped = agent.global_pose_to_local_pose(global_pose_stamped)
            pose_stamped.header.frame_id = 'map'
            agent.run_command(Command.GOTO, (pose_stamped,))
            agents.append(agent)
        self._wait_for_agents(agents)
        if self.controller.state != State.COMPLETE:
            self.controller.state = State.RUN

    def _realign_pose_stamped(self, pose_stamped):
        new_pose = copy.deepcopy(pose_stamped)
        new_pose.pose.orientation.x = 0
        new_pose.pose.orientation.y = 0
        new_pose.pose.orientation.z = 0
        new_pose.pose.orientation.w = 1
        return new_pose

    def _run_state(self):
        rospy.loginfo('_run_state')
        for agent in self.controller.agents:
            agent.clear_traversed()
        rospy.sleep(2)
        running = True
        path_index = 0
        while running:
            if self.controller.state == State.COMPLETE:
                break
            running = False
            agents = []
            camera_agents = []
            realigned_pose_stamped = []
            for agent in self.controller.agents:
                if len(agent.path.poses) <= path_index:
                    continue
                rospy.loginfo("%s pose_stamped index %s", agent.a_id, path_index)
                pose_stamped = agent.global_pose_to_local_pose(agent.path.poses[path_index])
                agent.run_command(Command.GOTO, (pose_stamped, ))
                agents.append(agent)
                running = True
                if not agent.check_if_traversed(pose_stamped.pose):
                    camera_agents.append(agent)
                    realigned_pose_stamped.append(self._realign_pose_stamped(pose_stamped))
                    agent.add_pose_to_traversed(pose_stamped.pose)
            self._wait_for_agents(agents)
            for agent, new_pose in zip(camera_agents, realigned_pose_stamped):
                agent.run_command(Command.GOTO, (new_pose, ))
            self._wait_for_agents(camera_agents)
            # let the agents settle in capture position
            rospy.sleep(0.2)
            for agent in camera_agents:
                agent.grab_image()
            path_index += 1
        self.controller.state = State.COMPLETE

    def _complete_state(self):
        rospy.loginfo('_complete_state')
        agents = []
        for agent in self.controller.agents:
            agent.run_command(Command.SET_RETURN_TO_HOME)
            agents.append(agent)
        self._wait_for_agents(agents)
        try:
            stop_heartbeat = rospy.ServiceProxy('stop_heartbeat', Trigger)
            resp = stop_heartbeat()
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s", e)
            self.controller.state = State.IDLE
        if not resp.success:
            self.controller.state = State.IDLE
            rospy.logerr(resp)
        self.controller.state = State.IDLE

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.controller.state == State.IDLE:
                self._idle_state()
            elif self.controller.state == State.PLAN:
                self._plan_state()
            elif self.controller.state == State.PREP:
                self._prep_state()
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
