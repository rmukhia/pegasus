#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, Point, PoseStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from pegasus_planner.pegasus_grid_cells import getGridCells
from pegasus_planner.pegasus_cell import Cell
from pegasus_planner.pegasus_cell_container import CellContainer
from pegasus_planner.pegasus_agent import Agent
from pegasus_planner.pegasus_path_finder import PathFinder
from pegasus_planner.pegasus_state import State

class PegasusPlanner(object):
  def __init__(self, agents):
    self.gridCells = None
    self.cellContainer = None
    self.agents = agents
    self.agentPathPublisher = {}
    self.polygonSub = None
    self.markerPub = None
    self.unvisitedPoints = []
    self.visitedPoints = []
    pass

  def createGridMarkers(self):
    for k in range(self.gridCells.minMax[0] * self.gridCells.minMax[1]):
      ith, jth = self.gridCells.getIndex(self.gridCells.minMax[0], self.gridCells.minMax[1], k)
      if self.gridCells.valid[k]:
        # append markers here
        p = Point()
        p.x, p.y = self.gridCells.cellsCenter[k]
        p.z = 0
        self.unvisitedPoints.append(p)

  def publishGrid(self):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.ns = 'unvisited'
    marker.id = 0
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1

    marker.points = self.unvisitedPoints
    marker.lifetime = rospy.Duration()
    marker.scale.x = 5
    marker.scale.y = 5
    marker.scale.z = 5
    marker.color.g = 1
    marker.color.r = 1
    marker.color.b = 0
    marker.color.a = 1

    self.markerPub.publish(marker)

    marker = Marker()
    marker.header.frame_id = '/map'
    marker.ns = 'visited'
    marker.id = 1
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1
    marker.points = self.visitedPoints
    marker.color.g = 0
    marker.color.r = 1
    marker.color.b = 1
    self.markerPub.publish(marker)

    marker = Marker()
    marker.header.frame_id = '/map'
    marker.ns = 'test'
    marker.id = 1
    marker.type = marker.ARROW
    marker.action = marker.ADD
    q = quaternion_from_euler(0, 0, 1.57)
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 10
    marker.scale.x = 5
    marker.scale.y = 5
    marker.scale.z = 5
    marker.color.g = 1
    marker.color.r = 1
    marker.color.b = 0
    marker.color.a = 1
    self.markerPub.publish(marker)

  def createAgentMarkers(self, agentspose):
    numAgents = len(self.agents)

    while True:
      isEmpty = True
      for i in range(numAgents):
        if len(agentspose[i]['points']) > 0:
          isEmpty = False

      if isEmpty:
        break

      for i in range(numAgents):
        if (len(agentspose[i]['points']) == 0):
          continue
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.ns = 'agents'
        marker.id = i
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        x, y = agentspose[i]['points'].pop()

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 10
        marker.lifetime = rospy.Duration()
        marker.scale.x = 5
        marker.scale.y = 5
        marker.scale.z = 5
        marker.color.g = 0.7 + (i * 0.2)/numAgents 
        marker.color.r = 0.5 + (i * 0.2)/numAgents 
        marker.color.b = 0.1 + (i * 0.05)/numAgents 
        marker.color.a = 1

        self.markerPub.publish(marker)


        for k in range(len(self.unvisitedPoints)):
          p = self.unvisitedPoints[k]
          if p.x == x and p.y == y:
            del self.unvisitedPoints[k]
            break
        self.visitedPoints.append(Point(x, y, 0))

      self.publishGrid()
        
      rospy.sleep(0.5)

  def createCellContainer(self):
    self.cellContainer = CellContainer(self.gridCells.boundingBox, self.gridCells, 10, numDirections = 8)

  def calculatePath(self):
    state = State(0, np.zeros((self.cellContainer.iMax, self.cellContainer.jMax), dtype = float),
        self.cellContainer)
    self.pathFinder = PathFinder(self.agents, self.cellContainer)
    goal = self.pathFinder.find(8, 4)
    return goal

  def createPath(self, agentspose):
    # to get the angle between two vectors, tan -1 (y1 -y2/ x1 -x2)
    # basically transpose x2,y2 to the origin and get the direction of
    # the transposed vector.
    numAgents = len(self.agents)
    for i in range(numAgents):
      numPoints = len(agentspose[i]['points'])  
      vectors1 = np.array(agentspose[i]['points'])
      vectors2 = np.zeros((numPoints, 2))
      vectors2[1:] = vectors1[0:-1]

      v1sv2 = vectors1 - vectors2
      direction = np.arctan2(v1sv2[:,1], v1sv2[:,0]) 


      poses = []

      for k in range(numPoints):
        '''
        q = None
        yaw = 0
        if (v1sv2[k, 0] == 0 or v1sv2[k, 1] == 0):
          # parallel to x axis or y axis
          if v1sv2[k, 1] == 0:
            # y axis is parallel
            if (vectors1[k, 0] > vectors2[k, 0]):
              # going left
              yaw = 0
            else:
              # going right
              yaw = np.pi
          elif v1sv2[k, 0] ==0:
            # x axis is parallel
            if (vectors1[k, 1] > vectors2[k, 1]):
              # going up
              yaw = 0.5 * np.pi
            else:
              # going down
              yaw = 1.5 * np.pi
        else:
        '''
        yaw = direction[k]

        # Don't know why np.pi has to be put. Maybe coordinate difference
        q = quaternion_from_euler(0, 0, yaw + np.pi)

        pose = PoseStamped()
        pose.header.frame_id = '/map'
        x, y = agentspose[i]['points'][k]
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 10
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        poses.append(pose)
      path = Path()
      path.header.frame_id = '/map'
      path.poses = poses
      self.agentPathPublisher[self.agents[i].id].publish(path)

  def recvPolygon(self, data):
    rospy.loginfo('Received polygon...')
    numPoints = len(data.polygon.points)
    polygon = np.zeros((numPoints, 2))
    for i in range(numPoints):
      polygon[i] = (data.polygon.points[i].x , data.polygon.points[i].y)

    self.gridCells = getGridCells(polygon, 10)
    rospy.loginfo('Calculated cells...')
    self.createGridMarkers()
    self.visitedPoints = []
    self.publishGrid()
    self.createCellContainer()
    goal = self.calculatePath()
    agentspose = self.pathFinder.getMovementPlanFromGoal(goal)
    self.createPath(agentspose)
    self.createAgentMarkers(agentspose)

  def subscribeAndPublish(self):
    self.polygonSub = rospy.Subscriber('/mapviz/region_selected', PolygonStamped, self.recvPolygon)
    self.markerPub = rospy.Publisher('/pegasus/grid_markers', Marker, latch=True, queue_size=1000)
    for agent in self.agents:
      pub = rospy.Publisher('/pegasus/path/%s' % ( agent.id,), Path, latch= True, queue_size=1000)
      self.agentPathPublisher[agent.id] = pub


if __name__ == '__main__':
  rospy.init_node('pegasus_planner')
  rospy.loginfo('Starting pegasus planner...')
  agents = [
      Agent('uav0', (-21.9,19.0)),
      Agent('uav1', (-22.5,-2.4)),
      Agent('uav2', (-17.5,5.0)),
      ]
  pegasusPlanner = PegasusPlanner(agents)
  pegasusPlanner.subscribeAndPublish()

  rospy.spin()
