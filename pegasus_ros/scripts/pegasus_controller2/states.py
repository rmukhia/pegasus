from enum import IntEnum


class StateEnum(IntEnum):
    IDLE = 1
    OFFBOARD_MODE = 2
    ARMING = 3
    TAKE_OFF = 4
    CALIBRATE = 5
    GENERATE_TRANSFORMS = 6
    RUN = 7
    COMPLETE = 8


class State(object):
    def __init__(self):
        self.controller = None
        self.complete = False
        self.success = True

    def set_controller(self, controller):
        self.controller = controller

    def run(self):
        raise NotImplementedError('%s must override run' % (type(self),))

    def next(self):
        raise NotImplementedError('%s must override next' % (type(self),))

    def set_complete(self, complete):
        self.complete = complete

    def is_complete(self):
        raise NotImplementedError('%s must override isComplete'
                                  % (type(self),))

    def set_success(self, success=True):
        self.success = success

    def is_success(self):
        return self.success


class IdleState(State):
    STATE = StateEnum.IDLE

    def __init__(self):
        super(State, self).__init__()
        self.success = True

    def run(self):
        pass

    def next(self):
        if self.success:
            return OffboardModeState()
        return IdleState()

    def is_complete(self):
        return self.complete


class OffboardModeState(State):
    STATE = StateEnum.OFFBOARD_MODE

    def __init__(self):
        super(State, self).__init__()
        self.success = True
        self.sent = False

    def run(self):
        if self.sent:
            return
        for agent in self.controller.agents:
            agent.set_offboard()
        self.sent = True

    def next(self):
        if self.success:
            return ArmingState()
        return IdleState()

    def is_complete(self):
        return self.complete


class ArmingState(State):
    STATE = StateEnum.ARMING

    def __init__(self):
        super(State, self).__init__()
        self.success = True
        self.sent = False

    def run(self):
        if self.sent:
            return
        for agent in self.controller.agents:
            agent.set_arm()
        self.sent = True

    def next(self):
        if self.success:
            return TakeOffState()
        return IdleState()

    def is_complete(self):
        return self.complete


class TakeOffState(State):
    STATE = StateEnum.TAKE_OFF

    def __init__(self):
        super(State, self).__init__()
        self.success = True
        self.sent = False

    def run(self):
        if self.sent:
            return
        for agent in self.controller.agents:
            agent.takeoff()
        self.sent = True

    def next(self):
        if self.success:
            return CalibrateState()
        return IdleState()

    def is_complete(self):
        return self.complete


class CalibrateState(State):
    STATE = StateEnum.CALIBRATE

    def __init__(self):
        super(State, self).__init__()

    def run(self):
        pass

    def next(self):
        if self.success:
            return GenerateTransformsState()
        return IdleState()

    def is_complete(self):
        return True


class GenerateTransformsState(State):
    STATE = StateEnum.GENERATE_TRANSFORMS

    def __init__(self):
        super(State, self).__init__()

    def run(self):
        pass

    def next(self):
        if self.success:
            return RunState()
        return IdleState()

    def is_complete(self):
        return True


class RunState(State):
    STATE = StateEnum.RUN

    def __init__(self):
        super(State, self).__init__()

    def run(self):
        pass

    def next(self):
        if self.success:
            return CompleteState()
        return IdleState()

    def is_complete(self):
        return True


class CompleteState(State):
    STATE = StateEnum.IDLE

    def __init__(self):
        super(State, self).__init__()

    def run(self):
        pass

    def next(self):
        return IdleState()

    def is_complete(self):
        return True
