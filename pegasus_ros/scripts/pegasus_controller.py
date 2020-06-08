#!/usr/bin/env python

import rospy

import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

from pegasus_controller.pegasus_controller_thread import PegasusControllerThread

class PegasusController(object):
  def __init__(self, mavrosNamespaces, localTransforms):
    self.agents = {}
    for i, agent in enumerate(mavrosNamespaces):
      self.agents[agent] = {
          'localTransformMap' : localTransforms[i][0],
          'localTransformBaseLink' : localTransforms[i][1],
          'path': None,
          'currentPoseInPath': 0, # counter for PoseStamped in Path
          'state': None,
          'localPose': None,
          'pathSubscriber': None,
          'stateSubscriber': None,
          'setPointPublisher': None,
          'localPositionSubscriber': None,
          'setModeService': None,
          'armingService': None
        }

    self.createMavrosServiceClients()
    self.subscribeAndPublishMavrosAgents()
    self.createTransformListener()
    self.subscribePathTopics()

  def subscribePathTopics(self):
    for agent in self.agents:
      topic = '/pegasus/path/%s' % (agent ,)
      rospy.loginfo('Listening to path topic: %s' % (topic,))
      self.agents[agent]['pathSubscriber'] = rospy.Subscriber(topic, Path, self.recvPegasusPath, (agent, topic))

  def subscribeAndPublishMavrosAgents(self):
    for agent in self.agents:
      stateTopic = '/%s/mavros/state' % (agent,)
      rospy.loginfo('Subscriber to agent state: %s' % (stateTopic,))
      self.agents[agent]['stateSubscriber'] = rospy.Subscriber(stateTopic, State,
              self.recvMavrosState, (agent, stateTopic))

      setPointTopic = '/%s/mavros/setpoint_position/local' % (agent,)
      rospy.loginfo('Publisher to setpoint_position: %s' % (setPointTopic,))
      self.agents[agent]['setPointPublisher'] = rospy.Publisher(setPointTopic, PoseStamped, queue_size=1000)

      localPositionTopic = '/%s/mavros/local_position/pose' % (agent,)
      rospy.loginfo('Subscriber to local position: %s' % (localPositionTopic,))
      self.agents[agent]['localPositionSubscriber'] = rospy.Subscriber(localPositionTopic, PoseStamped,
              self.recvMavrosPose, (agent, localPositionTopic))

  def createMavrosServiceClients(self):
    for agent in self.agents:
      setModeService = '/%s/mavros/set_mode' % (agent, )
      armingService = '/%s/mavros/cmd/arming' % (agent, )
      rospy.wait_for_service(setModeService)
      rospy.loginfo('Set Mode service: %s' % (setModeService,))
      self.agents[agent]['setModeService'] = rospy.ServiceProxy(setModeService, SetMode)
      rospy.wait_for_service(setModeService)
      rospy.loginfo('Arming service: %s' % (armingService,))
      self.agents[agent]['armingService'] = rospy.ServiceProxy(armingService, CommandBool)


  def createTransformListener(self):
    self.tfBuffer = tf2_ros.Buffer()
    self.transformListener = tf2_ros.TransformListener(self.tfBuffer)

  def recvPegasusPath(self, data, args):
    agent = args[0]
    self.agents[agent]['path'] = data
    #rospy.loginfo("Received path for %s" % (agent, ))

  def recvMavrosState(self, data, args):
    agent = args[0]
    self.agents[agent]['state'] = data
    #print(data)

  def recvMavrosPose(self, data, args):
    agent = args[0]
    self.agents[agent]['localPose'] = data
    #print(data)

if __name__ == '__main__':
  rospy.init_node('pegasus_controller')
  rospy.loginfo('Starting pegasus_controller...')
  localTransforms = rospy.get_param('pegasus_local_transforms')
  mavrosNamespaces = rospy.get_param('pegasus_mavros_namespaces')

  pegasusController = PegasusController(mavrosNamespaces, localTransforms)
  pegasusControllerThread = PegasusControllerThread(1, pegasusController)

  pegasusControllerThread.start()
  
  rospy.spin()
