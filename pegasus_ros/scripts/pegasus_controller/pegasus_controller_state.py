from enum import IntEnum


class PegasusControllerState(IntEnum):
    IDLE = 1
    PREP = 2
    OFFBOARD_MODE = 3
    ARMING = 4
    TAKE_OFF = 5
    CALIBRATE = 6
    GENERATE_TRANSFORMS = 7
    RUN = 8
    COMPLETE = 9
