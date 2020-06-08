import threading
import rospy
import numpy as np

import tf2_ros
import tf2_geometry_msgs
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
    self.state = PegasusControllerState.PREP

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
      # TODO: make height a ros parameter
      transformedPose.pose.position.z = 10
      if transformedPose is None:
        rospy.loginfo('Invalid transform')
        continue
      transformedPose.header.frame_id = '/' + agent['localTransformMap']
      agent['setPointPublisher'].publish(transformedPose)
      if not self.is_at_position(agent['localPose'],
          transformedPose.pose.position.x, 
          transformedPose.pose.position.y,
          transformedPose.pose.position.z, 0.5):
        reached = False

    if reached:
      # all agents have reached the pose
      for agent in self.pegasusController.agents.values():
        agent['currentPoseInPath'] += 1

    if pathCompleted:
      # all agents have completed their path
      self.state = PegasusControllerState.COMPLETE


  def run(self):
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
      if self.state == PegasusControllerState.IDLE:
        self._idleState()
      elif self.state == PegasusControllerState.PREP:
        self._prepState()
      elif self.state == PegasusControllerState.OFFBOARD_MODE:
        self._offboardState()
      elif self.state == PegasusControllerState.ARMING:
        self._armingState()
      elif self.state == PegasusControllerState.TAKE_OFF:
        self._takeOffState()
      elif self.state == PegasusControllerState.RUN:
        self._runState()
      elif self.state == PegasusControllerState.COMPLETE:
        pass
      try:
        rate.sleep()
      except rospy.ROSInterruptException:
        pass
