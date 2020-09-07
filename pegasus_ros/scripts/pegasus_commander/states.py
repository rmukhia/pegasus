import rospy
from enum import IntEnum
import numpy as np
from geometry_msgs.msg import PoseStamped


class StateEnum(IntEnum):
    IDLE = 0
    PREP_STATE = 1
    OFFBOARD_MODE = 2
    ARMING = 3
    HOVER = 4
    RUN = 5
    RETURN_TO_HOME = 6


def is_at_position(expected_pose, actual_pose, offset):
    """ Actually its PoseStamped"""
    desired = np.array((expected_pose.pose.position.x,
                        expected_pose.pose.position.y,
                        expected_pose.pose.position.z))
    actual = np.array((actual_pose.pose.position.x,
                       actual_pose.pose.position.y,
                       actual_pose.pose.position.z))
    return np.linalg.norm(desired - actual) < offset


class State(object):
    STATE = -1

    def __init__(self):
        self.mavrosGw = None
        self.data = None
        self.complete = False
        self.callback = None

    def set_data(self, data):
        self.data = data

    def set_mavros_gw(self, mavros_gw):
        self.mavrosGw = mavros_gw
        pass

    def set_callback(self, callback):
        self.callback = callback

    def run(self):
        raise NotImplementedError('%s must override run' % (type(self),))

    def next(self, next_state):
        raise NotImplementedError('%s must override next' % (type(self),))

    def is_complete(self):
        raise NotImplementedError('%s must override isComplete'
                                  % (type(self),))

    def step(self):
        pose = self.mavrosGw.get_mavros_local_pose()
        self.mavrosGw.set_mavros_local_pose(pose)

    def runCallback(self):
        if self.complete and self.callback is not None:
            self.callback[0](*self.callback[1])

    def set_state(self, next_state, mavros_gw, data=None, callback=None):
        if self.is_complete():
            state = self.next(next_state)
            if state.STATE == next_state:
                state.set_mavros_gw(mavros_gw)
                state.set_data(data)
                state.set_callback(callback)
                return state
        return self


class IdleState(State):
    STATE = StateEnum.IDLE

    def __init__(self):
        super(State, self).__init__()

    def run(self):
        pass

    def next(self, next_state):
        if next_state == PrepState.STATE:
            return PrepState()
        return self

    def is_complete(self):
        return True

    def step(self):
        # do nothing
        pass


class PrepState(State):
    STATE = StateEnum.PREP_STATE

    def __init__(self):
        super(State, self).__init__()
        self.complete = False
        self.i = 0

    def run(self):
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        self.mavrosGw.set_mavros_local_pose(pose)
        self.i += 1
        if self.i >= 100:
            self.complete = True
        self.runCallback()

    def next(self, next_state):
        if next_state == OffboardModeState.STATE:
            return OffboardModeState()
        return self

    def is_complete(self):
        return self.complete


class OffboardModeState(State):
    STATE = StateEnum.OFFBOARD_MODE

    def __init__(self):
        super(State, self).__init__()
        self.complete = False
        self.prev_time = None

    def run(self):
        now = rospy.get_rostime()
        if self.prev_time is not None and now - self.prev_time < rospy.Duration(5):
            return
        if self.mavrosGw.get_mavros_state().mode != 'OFFBOARD':
            try:
                res = self.mavrosGw.mavrosService['setMode'](
                    base_mode=0, custom_mode='OFFBOARD')
                if not res.mode_sent:
                    rospy.logerr("failed to send mode command")
                    self.prev_time = now
                    return
            except rospy.ServiceException as e:
                rospy.logerr(e)
                self.prev_time = now
                return
        self.complete = True
        self.runCallback()

    def next(self, next_state):
        if next_state == ArmingState.STATE:
            return ArmingState()
        return self

    def is_complete(self):
        return self.complete


class ArmingState(State):
    STATE = StateEnum.ARMING

    def __init__(self):
        super(State, self).__init__()
        self.complete = False
        self.prev_time = None

    def run(self):
        now = rospy.get_rostime()
        if self.prev_time is not None and now - self.prev_time < rospy.Duration(5):
            return
        if not self.mavrosGw.get_mavros_state().armed:
            try:
                res = self.mavrosGw.mavrosService['arming'](True)
                if not res.success:
                    rospy.logerr("failed to send arm command")
                    self.prev_time = now
                    return
            except rospy.ServiceException as e:
                self.prev_time = now
                rospy.logerr(e)
                return
        self.complete = True
        self.runCallback()

    def next(self, next_state):
        if next_state == HoverState.STATE:
            return HoverState()
        elif next_state == RunState.STATE:
            return RunState()
        elif next_state == ReturnToHomeState.STATE:
            return ReturnToHomeState()
        return self

    def is_complete(self):
        return self.complete


class HoverState(State):
    STATE = StateEnum.HOVER

    def __init__(self):
        super(State, self).__init__()
        self.complete = True

    def run(self):
        self.complete = True
        self.runCallback()

    def step(self):
        pose = self.data
        rospy.loginfo(pose)
        self.mavrosGw.set_mavros_local_pose(pose)

    def next(self, next_state):
        if next_state == RunState.STATE:
            return RunState()
        elif next_state == ReturnToHomeState.STATE:
            return ReturnToHomeState()
        return self

    def is_complete(self):
        return self.complete


class RunState(State):
    STATE = StateEnum.RUN

    def __init__(self):
        super(State, self).__init__()
        self.complete = False

    def run(self):
        actual_pose = self.mavrosGw.get_mavros_local_pose()
        expected_pose = self.data
        if is_at_position(expected_pose, actual_pose, 0.5):
            self.complete = True
            self.runCallback()
            return
        rospy.loginfo(expected_pose)
        self.mavrosGw.set_mavros_local_pose(expected_pose)

    def next(self, next_state):
        if next_state == HoverState.STATE:
            return HoverState()
        elif next_state == ReturnToHomeState.STATE:
            return ReturnToHomeState()
        return self

    def is_complete(self):
        return self.complete


class ReturnToHomeState(State):
    STATE = StateEnum.RETURN_TO_HOME

    def __init__(self):
        super(State, self).__init__()
        self.complete = False
        self.prev_time = None

    def run(self):
        now = rospy.get_rostime()
        if self.prev_time is not None and now - self.prev_time < rospy.Duration(5):
            return
        if self.mavrosGw.get_mavros_state().armed:
            try:
                res = self.mavrosGw.mavrosService['arming'](False)
                if not res.success:
                    rospy.logerr("failed to send disarm command")
                    self.prev_time = now
                    return
            except rospy.ServiceException as e:
                self.prev_time = now
                rospy.logerr(e)
                return
        self.complete = True
        self.runCallback()

    def next(self, next_state):
        if next_state == IdleState.STATE:
            return IdleState()
        return self

    def is_complete(self):
        return self.complete
