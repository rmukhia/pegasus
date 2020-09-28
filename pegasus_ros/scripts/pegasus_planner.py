#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import PolygonStamped, Point, PoseStamped, PointStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

from pegasus_planner.pegasus_agent import Agent
from pegasus_planner.pegasus_cell_container import CellContainer
from pegasus_planner.pegasus_grid_cells import getGridCells
from pegasus_planner.pegasus_path_finder import PathFinder
from pegasus_planner.pegasus_state import State


class PegasusPlanner(object):
    def __init__(self, mavrosNamespaces, params):
        self.gridCells = None
        self.cellContainer = None
        self.agentPathPublisher = {}
        self.polygonSub = None
        self.markerPub = None
        self.unvisitedPoints = []
        self.visitedPoints = []
        self.params = params
        self.agents = {}
        self.mavrosNamespaces = mavrosNamespaces
        for agent in mavrosNamespaces:
            self.agents[agent] = {
                'mapPoint': None,
                'mapPointSubscriber': None,
                'pathPublisher': None,
            }

        self.subscribeAgentPoints()
        self.subscribeAndPublish()

    def subscribeAgentPoints(self):
        for agent in self.agents:
            self.agents[agent]['mapPointSubscriber'] = rospy.Subscriber('/pegasus/%s/position/gpsToMap' % (agent,),
                                                                        PointStamped, self.recvAgentMapPoint, (agent,))

    def subscribeAndPublish(self):
        self.polygonSub = rospy.Subscriber('/mapviz/region_selected', PolygonStamped, self.recvPolygon)
        self.markerPub = rospy.Publisher('/pegasus/grid_markers', Marker, latch=True, queue_size=1000)
        for agent in self.agents:
            self.agents[agent]['pathPublisher'] = rospy.Publisher('/pegasus/%s/path' % (agent,), Path,
                                                                  latch=True, queue_size=1000)

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
        marker.header.frame_id = 'map'
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

        '''
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
        '''

    def createAgentMarkers(self, agentspose):
        numAgents = len(self.mavrosNamespaces)

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
                marker.header.frame_id = 'map'
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
                marker.color.g = 0.7 + (i * 0.2) / numAgents
                marker.color.r = 0.5 + (i * 0.2) / numAgents
                marker.color.b = 0.1 + (i * 0.05) / numAgents
                marker.color.a = 1

                self.markerPub.publish(marker)

                # for k in range(len(self.unvisitedPoints)):
                #  p = self.unvisitedPoints[k]
                #  if p.x == x and p.y == y:
                #    del self.unvisitedPoints[k]
                #    break
                # self.visitedPoints.append(Point(x, y, 0))

            self.publishGrid()

            rospy.sleep(0.5)

    def createCellContainer(self):
        self.cellContainer = CellContainer(self.gridCells.boundingBox, self.gridCells, self.params['gridSize'],
                                           numDirections=8, agentsHoverHeight=self.params['agentsHoverHeight'])

    def calculatePath(self):
        state = State(0, np.zeros((self.cellContainer.iMax, self.cellContainer.jMax), dtype=float),
                      self.cellContainer)
        agentObjects = []
        for agent in self.mavrosNamespaces:
            mapPoint = self.agents[agent]['mapPoint']
            obj = Agent(agent, (mapPoint.point.x, mapPoint.point.y))
            agentObjects.append(obj)

        self.pathFinder = PathFinder(agentObjects, self.cellContainer)
        goal = self.pathFinder.find(8, 4)
        return goal

    def createPath(self, agentspose):
        # to get the angle between two vectors, tan -1 (y1 -y2/ x1 -x2)
        # basically transpose x2,y2 to the origin and get the direction of
        # the transposed vector.
        numAgents = len(self.mavrosNamespaces)
        for i in range(numAgents):
            numPoints = len(agentspose[i]['points'])
            vectors1 = np.array(agentspose[i]['points'])
            vectors2 = np.zeros((numPoints, 2))
            vectors2[1:] = vectors1[0:-1]

            v1sv2 = vectors1 - vectors2
            direction = np.arctan2(v1sv2[:, 1], v1sv2[:, 0])

            poses = []

            for k in range(numPoints):
                yaw = direction[k]
                q = quaternion_from_euler(0, 0, yaw)
                pose = PoseStamped()
                pose.header.frame_id = 'map'
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
            path.header.frame_id = 'map'
            path.poses = poses
            self.agents[self.mavrosNamespaces[i]]['pathPublisher'].publish(path)

    def recvPolygon(self, data):
        rospy.loginfo('Received polygon...')
        numPoints = len(data.polygon.points)
        polygon = np.zeros((numPoints, 2))
        for i in range(numPoints):
            polygon[i] = (data.polygon.points[i].x, data.polygon.points[i].y)

        self.gridCells = getGridCells(polygon, self.params['gridSize'])
        rospy.loginfo('Calculated cells...')
        self.createGridMarkers()
        self.visitedPoints = []
        self.publishGrid()
        self.createCellContainer()

        isPoseAvailable = True
        for agent in self.agents:
            if self.agents[agent]['mapPoint'] is None:
                isPoseAvailable = False
                break

        if isPoseAvailable:
            goal = self.calculatePath()
            agentspose = self.pathFinder.getMovementPlanFromGoal(goal)
            self.createPath(agentspose)
            # self.createAgentMarkers(agentspose)

    def recvAgentMapPoint(self, data, args):
        agent = args[0]
        self.agents[agent]['mapPoint'] = data


if __name__ == '__main__':
    rospy.init_node('pegasus_planner')
    rospy.loginfo('Starting pegasus planner...')
    mavrosNamespaces = rospy.get_param('pegasus_mavros_namespaces')
    zHeight = rospy.get_param('agents_hover_height')
    gridSize = rospy.get_param('grid_size')

    pegasusPlanner = PegasusPlanner(mavrosNamespaces, {
        'agentsHoverHeight': float(zHeight),
        'gridSize': float(gridSize)
    }
                                    )

    rospy.spin()
