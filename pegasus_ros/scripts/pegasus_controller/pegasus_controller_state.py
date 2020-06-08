from enum import Enum

class PegasusControllerState(Enum):
  IDLE = 1
  PREP = 2
  OFFBOARD_MODE = 4
  ARMING = 3
  TAKE_OFF = 5
  RUN = 6
  COMPLETE = 7

