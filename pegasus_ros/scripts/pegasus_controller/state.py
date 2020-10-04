from enum import IntEnum


class State(IntEnum):
    IDLE = 1
    PLAN = 2
    PREP = 3
    OFFBOARD_MODE = 4
    ARMING = 5
    TAKE_OFF = 6
    CALIBRATE = 7
    GENERATE_TRANSFORMS = 8
    PRE_RUN = 9
    RUN = 10
    COMPLETE = 11
