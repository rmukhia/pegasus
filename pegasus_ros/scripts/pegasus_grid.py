#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PolygonStamped, Point
from visualization_msgs.msg import Marker
from pegasus_grid_maker import getGridCells

class GridMaker(object):
  def __init__(self):
    self.gridCells = None
    pass

  def createMarkers(self):
    points = []
    for k in range(self.gridCells.minMax[0] * self.gridCells.minMax[1]):
      ith, jth = self.gridCells.getIndex(self.gridCells.minMax[0], self.gridCells.minMax[1], k)
      if self.gridCells.valid[k]:
        # append markers here
        p = Point()
        p.x, p.y = self.gridCells.cellsCenter[k]
        p.z = 0
        points.append(p)

    marker = Marker()
    marker.header.frame_id = '/map'
    marker.type = marker.POINTS
    marker.action = marker.ADD
    marker.pose.orientation.w = 1

    marker.points = points
    marker.lifetime = rospy.Duration()
    marker.scale.x = 5
    marker.scale.y = 5
    marker.scale.z = 5
    marker.color.b = 1
    marker.color.a = 1

    self.markerPub.publish(marker)


  def recvPolygon(self, data):
    rospy.loginfo('Received polygon...')
    numPoints = len(data.polygon.points)
    polygon = np.zeros((numPoints, 2))
    for i in range(numPoints):
      polygon[i] = (data.polygon.points[i].x , data.polygon.points[i].y)

    self.gridCells = getGridCells(polygon, 10)
    rospy.loginfo('Calculated cells...')
    self.createMarkers()

  def subscribeAndPublish(self):
    self.polygonSub = rospy.Subscriber('/mapviz/region_selected', PolygonStamped, self.recvPolygon)
    self.markerPub = rospy.Publisher('/pegasus/grid_markers', Marker, latch=True, queue_size=1000)

if __name__ == '__main__':
  rospy.init_node('pegasus')
  rospy.loginfo('Starting pegasus...')
  gridMaker = GridMaker()
  gridMaker.subscribeAndPublish()

  rospy.spin()
