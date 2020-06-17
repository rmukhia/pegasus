#!/usr/bin/env python

'''
Pegasus Controller. Manages the transformations between different frame of references.
Provides '/map' poses to other components.
Controls mavros.
All GPS coordinates are normalized to UTM.
'''

import rospy
import numpy as np
from geodesy import utm
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import UInt8
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool

from pegasus_controller.pegasus_controller_thread import PegasusControllerThread

class PegasusController(object):
  def __init__(self, mavrosNamespaces, localTransforms, params):
    self.agents = {}
    self.params = params
    self.mavrosNamespaces = mavrosNamespaces
    self.mapOrigin = None # PoseStamped
    self.tf2Br = None

    for i, agent in enumerate(mavrosNamespaces):
      self.agents[agent] = {
          'reached': False,
          'localTransformMap' : localTransforms[i][0],
          'localTransformBaseLink' : localTransforms[i][1],
          'path': None,
          'currentPoseInPath': 0, # counter for PoseStamped in Path
          'state': None,
          'localPose': None,
          'globalGps': None,
          'pathSubscriber': None,
          'stateSubscriber': None,
          'setPointPublisher': None,
          'localPositionSubscriber': None,
          'globalGpsPositionSubscriber': None,
          'setModeService': None,
          'armingService': None,
          'mapLPosePublisher': None,
          'mapGPointPublisher': None,
          'calibrationPath' : None,
          'calibrationPathPublisher': None,
          'currentPoseInCalibrationPath': 0,
          'captureLocalPose': False,
          'captureGlobalGps': False,
          'calibLocalPoses': [],
          'calibGlobalGpses': [],
          'homography': None,
          'transform': {
              'q': None,
              't': None,
              },
        }

    self.subscribeMapOrigin()
    self.createTransformBroadcaster()
    self.createTransformListener()
    self.createMavrosServiceClients()
    self.createCalibrationPathPublisher()
    self.publishControllerState()
    self.publishAgentsMapPosition()
    self.subscribeAndPublishMavrosAgents()
    self.subscribePathTopics()
    self.createCalibrationPath()


  def subscribeMapOrigin(self):
    topic = self.params['mapOriginTopic']
    self.mapOriginSub = rospy.Subscriber(topic, PoseStamped, self.recvMapOrigin)

  def subscribePathTopics(self):
    for agent in self.agents:
      topic = '/pegasus/%s/path' % (agent ,)
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
      self.agents[agent]['localPositionSubscriber'] = rospy.Subscriber(localPositionTopic, PoseStamped, self.recvMavrosPose, (agent, localPositionTopic))

      globalGpsTopic = '/%s/mavros/global_position/global' % (agent,)
      rospy.loginfo('Subscriber to global gps: %s' % (globalGpsTopic,))
      self.agents[agent]['globalGpsSubscriber'] = rospy.Subscriber(globalGpsTopic, NavSatFix, self.recvMavrosGlobalGps, (agent, globalGpsTopic))

  def publishControllerState(self):
    stateTopic = '/pegasus/state/controller'
    rospy.loginfo('Publisher controller state: %s' % (stateTopic,))
    self.statePublisher = rospy.Publisher(stateTopic, UInt8, queue_size=1000)

  def publishAgentsMapPosition(self):
    for agent in self.agents:
      mapLPositionTopic = '/pegasus/%s/position/localToMap' % ( agent, )
      rospy.loginfo('Publisher to local to map position: %s' % (mapLPositionTopic,))
      self.agents[agent]['mapLPosePublisher'] = rospy.Publisher(mapLPositionTopic, PoseStamped, queue_size=1000)
      mapGPositionTopic = '/pegasus/%s/position/gpsToMap' % ( agent, )
      rospy.loginfo('Publisher to gps to map point: %s' % (mapGPositionTopic,))
      self.agents[agent]['mapGPointPublisher'] = rospy.Publisher(mapGPositionTopic, PointStamped, queue_size=1000)

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


  def createTransformBroadcaster(self):
    self.tf2Br = tf2_ros.TransformBroadcaster()

  def createTransformListener(self):
    self.tfBuffer = tf2_ros.Buffer()
    self.transformListener = tf2_ros.TransformListener(self.tfBuffer)

  def createCalibrationPathPublisher(self):
    for agent in self.agents:
      calibrationPathTopic = '/pegasus/%s/calibration/path' % ( agent, )
      rospy.loginfo('Publisher to calibration path: %s' % (calibrationPathTopic,))
      self.agents[agent]['calibrationPathPublisher'] = rospy.Publisher(calibrationPathTopic, Path, latch=True, queue_size=1000)

  def createCalibrationPath(self):
    sideLength = self.params['gridSize']
    agentsHoverHeight = self.params['agentsHoverHeight']
    boxPoints = ((0., 0.), (sideLength, 0.), (sideLength, sideLength), (0., sideLength))
    #boxPoints = ((0, 0), (sideLength, 0))
    numPoints = len(boxPoints)
    vectors1 = np.array(boxPoints)
    vectors2 = np.zeros((numPoints, 2))
    vectors2[1:] = vectors1[0:-1]
    v1sv2 = vectors1 - vectors2
    direction = np.arctan2(v1sv2[:,1], v1sv2[:,0])
    for i, agent in enumerate(self.agents.values()):
      agent['calibrationPath'] = Path()
      for k in range(numPoints):
        yaw = direction[k]
        q = quaternion_from_euler(0, 0, yaw)
        pose = PoseStamped()
        pose.header.frame_id = agent['localTransformMap']
        pose.pose.position.x = boxPoints[k][0]
        pose.pose.position.y = boxPoints[k][1]
        pose.pose.position.z = agentsHoverHeight + (i * 2)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        agent['calibrationPath'].header.frame_id = agent['localTransformMap']
        agent['calibrationPath'].poses.append(pose)
        agent['calibrationPathPublisher'].publish(agent['calibrationPath'])

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
    if self.agents[agent]['captureLocalPose']:
      print ('capturing local pose for %s' % (agent,))
      self.agents[agent]['captureLocalPose'] = False
      self.agents[agent]['calibLocalPoses'].append(data)

    now = rospy.get_rostime()

    localMap = self.agents[agent]['localTransformMap']
    try:
      trans = self.tfBuffer.lookup_transform('map', localMap, rospy.Time())
      transformedPose = tf2_geometry_msgs.do_transform_pose(data, trans)
      self.agents[agent]['mapLPosePublisher'].publish(transformedPose)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      #rospy.loginfo(e)
      return None

  def recvMavrosGlobalGps(self, data, args):
    agent = args[0]
    utmPos = utm.fromLatLong(data.latitude, data.longitude, data.altitude)
    pt = utmPos.toPoint()
    d = np.array((pt.x, pt.y, pt.z))
    self.agents[agent]['globalGps'] = d
    if self.agents[agent]['captureGlobalGps']:
      print ('capturing globalpose for %s' % (agent,))
      self.agents[agent]['captureGlobalGps'] = False
      self.agents[agent]['calibGlobalGpses'].append(d)

    mapPoint = PointStamped()
    mapPoint.header.frame_id = 'map'
    mapPt = np.subtract(d, self.mapOrigin)
    mapPoint.point.x = mapPt[0]
    mapPoint.point.y = mapPt[1]
    mapPoint.point.z = mapPt[2]
    self.agents[agent]['mapGPointPublisher'].publish(mapPoint)


  def recvMapOrigin(self, data):
    mapOriginUTM = utm.fromLatLong(
      data.pose.position.y,
      data.pose.position.x,
      data.pose.position.z)

    self.mapOrigin = np.array((
      mapOriginUTM.toPoint().x,
      mapOriginUTM.toPoint().y,
      mapOriginUTM.toPoint().z))

    self.mapOriginSub.unregister()

  def broadcastTransforms(self):
    for agent in self.agents.values():
      if agent['transform']['q'] is not None and agent['transform']['t'] is not None:
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = agent['localTransformMap']
        t.transform.translation.x = agent['transform']['t'][0]
        t.transform.translation.y = agent['transform']['t'][1]
        t.transform.translation.z = 0
        t.transform.rotation.x = agent['transform']['q'][0]
        t.transform.rotation.y = agent['transform']['q'][1]
        t.transform.rotation.z = agent['transform']['q'][2]
        t.transform.rotation.w = agent['transform']['q'][3]
        
        self.tf2Br.sendTransform(t)

if __name__ == '__main__':
  rospy.init_node('pegasus_controller')
  rospy.loginfo('Starting pegasus_controller...')
  localTransforms = rospy.get_param('pegasus_local_transforms')
  mavrosNamespaces = rospy.get_param('pegasus_mavros_namespaces')
  zHeight = rospy.get_param('agents_hover_height')
  gridSize = rospy.get_param('grid_size')
  mapOriginTopic = rospy.get_param('map_origin_topic')

  pegasusController = PegasusController(mavrosNamespaces, localTransforms, { 'agentsHoverHeight': float(zHeight), 'gridSize': float(gridSize), 'mapOriginTopic': mapOriginTopic })
  pegasusControllerThread = PegasusControllerThread(1, pegasusController)

  pegasusControllerThread.start()
  
  rospy.spin()
