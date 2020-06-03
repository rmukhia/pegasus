#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, Point
from visualization_msgs.msg import Marker
from pegasus_grid_cells import getGridCells
from pegasus_cell import Cell
from pegasus_cell_container import CellContainer
from pegasus_agent import Agent
from pegasus_path_finder import PathFinder
from pegasus_state import State

class GridMaker(object):
  def __init__(self, agents):
    self.gridCells = None
    self.cellContainer = None
    self.agents = agents
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

    marker.ns = 'visited'
    marker.id = 1
    marker.points = self.visitedPoints
    marker.color.g = 0
    marker.color.r = 1
    marker.color.b = 1
    self.markerPub.publish(marker)

  def createCellContainer(self):
    self.cellContainer = CellContainer(self.gridCells.boundingBox, self.gridCells, 10, numDirections = 8)

  def calculatePath(self):
    state = State(0, np.zeros((self.cellContainer.iMax, self.cellContainer.jMax), dtype = float), self.cellContainer)
    self.pathFinder = PathFinder(self.agents, self.cellContainer)
    goal = self.pathFinder.find(8, 16)
    return goal

  def createAgentMarkers(self, finalGoal):
    agentpose = self.pathFinder.getMovementPlanFromGoal(finalGoal)
    numAgents = len(self.agents)

    while True:
      isEmpty = True
      for i in range(numAgents):
        if len(agentpose[i]['x']) > 0:
          isEmpty = False

      if isEmpty:
        break

      for i in range(numAgents):
        if (len(agentpose[i]['x']) == 0):
          continue
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.ns = 'agents'
        marker.id = i
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.pose.orientation.w = 1
        x = agentpose[i]['x'].pop()
        y = agentpose[i]['y'].pop()

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
        
      rospy.sleep(2)


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
    self.createAgentMarkers(goal)

  def subscribeAndPublish(self):
    self.polygonSub = rospy.Subscriber('/mapviz/region_selected', PolygonStamped, self.recvPolygon)
    self.markerPub = rospy.Publisher('/pegasus/grid_markers', Marker, latch=True, queue_size=1000)


if __name__ == '__main__':
  rospy.init_node('pegasus')
  rospy.loginfo('Starting pegasus...')
  agents = [
      Agent('001', (-21.9,19.0)),
      Agent('007', (-22.5,-2.4)),
      Agent('002', (-17.5,5.0)),
      ]
  gridMaker = GridMaker(agents)
  gridMaker.subscribeAndPublish()

  rospy.spin()
