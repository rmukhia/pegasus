#!/usr/bin/env python
import numpy as np

import rospy
from geometry_msgs.msg import PolygonStamped, Point
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker

from pegasus_planner.agent import Agent
from pegasus_planner.cell_container import CellContainer
from pegasus_planner.grid_cells import get_grid_cells
from pegasus_planner.path_finder import PathFinder
from pegasus_planner.state import State


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

    # noinspection PyMethodMayBeStatic
    def create_grid_markers(self, grid_cells):
        points = []
        for k in range(grid_cells.min_max[0] * grid_cells.min_max[1]):
            ith, jth = grid_cells.get_index(grid_cells.min_max[0], grid_cells.min_max[1], k)
            if grid_cells.valid[k]:
                # append markers here
                p = Point()
                p.x, p.y = grid_cells.cells_center[k]
                p.z = 0
                points.append(p)
        return points

    def publish_grid(self, points):
        marker = Marker()
        marker.header.frame_id = 'pegasus_map'
        marker.ns = 'grids'
        marker.id = 0
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1

        marker.points = points
        marker.lifetime = rospy.Duration()
        marker.scale.x = 3
        marker.scale.y = 3
        marker.scale.z = 3
        marker.color.g = 1
        marker.color.r = 1
        marker.color.b = 0
        marker.color.a = 1
        self.publisher_marker.publish(marker)

    def create_cell_container(self, grid_cells):
        self.cell_container = CellContainer(grid_cells.bounding_box, grid_cells, self.params['grid_size'],
                                            num_directions=self.params['num_directions'],
                                            agents_hover_height=self.params['agents_hover_height'])

    def calculate_path(self):
        # state = State(0, np.zeros((self.cell_container.i_max, self.cell_container.j_max), dtype=float),
        #              self.cell_container)
        # agent_objects = []
        for agent in self.agents:
            agent.set_initial_position((0, 0))

        self.path_finder = PathFinder(self.agents, self.cell_container)
        goal = self.path_finder.find(self.params['depth_exit'],
                                     self.params['retry_threshold'],
                                     self.params['early_exit'],
                                     self.params['c_power'])
        return goal

    def recv_polygon(self, data):
        rospy.loginfo('Received polygon...')
        num_points = len(data.polygon.points)
        polygon = np.zeros((num_points, 2))
        for i in range(num_points):
            polygon[i] = (data.polygon.points[i].x, data.polygon.points[i].y)

        grid_cells = get_grid_cells(polygon, self.params['grid_size'])
        rospy.loginfo('Calculated cells...')
        self.visited_points = []
        self.create_cell_container(grid_cells)
        points = self.create_grid_markers(grid_cells)
        self.publish_grid(points)
        rospy.loginfo('Published grids...')

    def _start_planning(self, request):
        rospy.loginfo('Start planning path...')
        goal = self.calculate_path()
        agents_pose = self.path_finder.get_movement_plan_from_goal(goal)
        for i, poses in enumerate(agents_pose):
            self.agents[i].set_path_plan(poses)
        marker_id = 0
        path_markers = []
        for agent in self.agents:
            agent.create_path()
            agent.publish_path()
            marker_start, marker_end = agent.get_start_end_markers()
            marker_start.id = marker_id
            marker_end.id = marker_id + 1
            marker_id += 2
            self.publisher_marker.publish(marker_start)
            self.publisher_marker.publish(marker_end)
            path_markers.extend(agent.get_path_markers())
        for i in range(len(path_markers)):
            path_markers[i].id = i
            self.publisher_marker.publish(path_markers[i])

        return TriggerResponse(True, 'Planned')



# import pydevd_pycharm
# pydevd_pycharm.settrace('localhost', port=7779, stdoutToServer=True, stderrToServer=True)
if __name__ == '__main__':
    rospy.init_node('pegasus_planner')
    rospy.loginfo('Starting pegasus planner...')
    mavros_namespaces = rospy.get_param('pegasus_mavros_namespaces')
    z_height = rospy.get_param('agents_hover_height')
    grid_size = rospy.get_param('grid_size')
    depth_exit = rospy.get_param('param_depth_exit')
    retry_threshold = rospy.get_param('param_retry_threshold')
    num_directions = rospy.get_param('param_num_directions')
    early_exit = rospy.get_param('param_early_exit')
    c_power = rospy.get_param('param_c_power')

    pegasus_planner = PegasusPlanner(mavros_namespaces, {
        'agents_hover_height': float(z_height),
        'grid_size': float(grid_size),
        'depth_exit': int(depth_exit),
        'retry_threshold': int(retry_threshold),
        'early_exit': int(early_exit),
        'num_directions': int(num_directions),
        'c_power': float(c_power)
    })

    rospy.spin()
