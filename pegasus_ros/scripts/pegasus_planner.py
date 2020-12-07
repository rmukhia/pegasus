#!/usr/bin/env python
import numpy as np

import rospy
from geometry_msgs.msg import PolygonStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker

from pegasus_planner_2.agent import Agent
from pegasus_planner_2.cell_container import CellContainer
from pegasus_planner_2.grid_cells import get_grid_cells
from pegasus_planner_2.path_finder import PathFinder
from pegasus_planner_2.state import State
'''
from pegasus_planner.agent import Agent
from pegasus_planner.cell_container import CellContainer
from pegasus_planner.grid_cells import get_grid_cells
from pegasus_planner.path_finder import PathFinder
from pegasus_planner.state import State
'''


class PegasusPlanner(object):
    def __init__(self, _mavros_namespaces, params):
        self.grid_cells = None
        self.cell_container = None
        self.path_finder = None
        self.agent_path_publisher = {}
        self.subscriber_polygon = None
        self.publisher_marker = None
        self.unvisited_points = []
        self.visited_points = []
        self.params = params
        self.agents = []
        self.mavros_namespaces = _mavros_namespaces
        for agent in _mavros_namespaces:
            self.agents.append(Agent(agent, params['agents_hover_height']))
        self.services = {}
        self.subscribe_and_publish()
        self.create_service()

    def create_service(self):
        self.services['start_planning'] = rospy.Service('start_planning', Trigger, self._start_planning)

    def subscribe_and_publish(self):
        self.subscriber_polygon = rospy.Subscriber('/mapviz/region_selected', PolygonStamped, self.recv_polygon)
        self.publisher_marker = rospy.Publisher('/pegasus/grid_markers', Marker, latch=True, queue_size=1000)

    def create_grid_markers(self):
        for k in range(self.grid_cells.min_max[0] * self.grid_cells.min_max[1]):
            ith, jth = self.grid_cells.get_index(self.grid_cells.min_max[0], self.grid_cells.min_max[1], k)
            if self.grid_cells.valid[k]:
                # append markers here
                p = Point()
                p.x, p.y = self.grid_cells.cells_center[k]
                p.z = 0
                self.unvisited_points.append(p)

    def publish_grid(self):
        marker = Marker()
        marker.header.frame_id = 'pegasus_map'
        marker.ns = 'unvisited'
        marker.id = 0
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1

        marker.points = self.unvisited_points
        marker.lifetime = rospy.Duration()
        marker.scale.x = 5
        marker.scale.y = 5
        marker.scale.z = 5
        marker.color.g = 1
        marker.color.r = 1
        marker.color.b = 0
        marker.color.a = 1
        self.publisher_marker.publish(marker)

    def create_cell_container(self):
        self.cell_container = CellContainer(self.grid_cells.bounding_box, self.grid_cells, self.params['grid_size'],
                                            num_directions=8, agents_hover_height=self.params['agents_hover_height'])

    def calculate_path(self):
        state = State(0, np.zeros((self.cell_container.i_max, self.cell_container.j_max), dtype=float),
                      self.cell_container)
        agent_objects = []
        for agent in self.agents:
            agent.set_initial_position((0, 0))

        self.path_finder = PathFinder(self.agents, self.cell_container)
        goal = self.path_finder.find(self.params['depth_exit'],
                                     self.params['retry_threshold'],
                                     self.params['early_exit'])
        return goal

    def recv_polygon(self, data):
        rospy.loginfo('Received polygon...')
        num_points = len(data.polygon.points)
        polygon = np.zeros((num_points, 2))
        for i in range(num_points):
            polygon[i] = (data.polygon.points[i].x, data.polygon.points[i].y)

        self.grid_cells = get_grid_cells(polygon, self.params['grid_size'])
        rospy.loginfo('Calculated cells...')
        self.create_grid_markers()
        self.visited_points = []
        self.publish_grid()
        self.create_cell_container()

    def _start_planning(self, request):
        rospy.loginfo('Start planning path...')
        goal = self.calculate_path()
        agents_pose = self.path_finder.get_movement_plan_from_goal(goal)
        for i, poses in enumerate(agents_pose):
            self.agents[i].set_path_plan(poses)
        for agent in self.agents:
            agent.create_path()
            agent.publish_path()
        return TriggerResponse(True, 'Planned')


if __name__ == '__main__':
    rospy.init_node('pegasus_planner')
    rospy.loginfo('Starting pegasus planner...')
    mavros_namespaces = rospy.get_param('pegasus_mavros_namespaces')
    z_height = rospy.get_param('agents_hover_height')
    grid_size = rospy.get_param('grid_size')
    depth_exit = rospy.get_param('param_depth_exit')
    retry_threshold = rospy.get_param('param_retry_threshold')
    early_exit = rospy.get_param('param_early_exit')

    pegasus_planner = PegasusPlanner(mavros_namespaces, {
        'agents_hover_height': float(z_height),
        'grid_size': float(grid_size),
        'depth_exit': int(depth_exit),
        'retry_threshold': int(retry_threshold),
        'early_exit': int(early_exit)
    })

    rospy.spin()
