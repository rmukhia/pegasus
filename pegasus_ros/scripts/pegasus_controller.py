#!/usr/bin/env python

"""
Pegasus Controller. Manages the transformations between different frame of references.
Provides '/map' poses to other components.
Controls mavros.
All GPS coordinates are normalized to UTM.
"""

import numpy as np

import rospy
import tf2_ros
from geodesy import utm
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse

from pegasus_controller.agent import Agent
from pegasus_controller.controller_thread import ControllerThread
from pegasus_controller.state import State


class PegasusController(object):
    def __init__(self, params):
        self.params = params
        self.agents = []
        for a_id, agent in enumerate(params['agents']):
            self.agents.append(
                Agent(
                    self,  # controller
                    a_id,  # agent id
                    agent[0],  # namespace
                    agent[1],  # remote address
                    agent[2],  # incoming port
                    params['calibration_size'])
            )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.global_map_name = 'pegasus_map'
        self.subscribers = {}
        self._subscribe_map_origin()
        self.services = {}
        self._create_services()
        self.state = State.IDLE
        self.heartbeat_active = False

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for agent in self.agents:
                agent.spin()
            rate.sleep()

    def _subscribe_map_origin(self):
        topic = self.params['map_origin_topic']
        self.subscribers['map_origin'] = rospy.Subscriber(topic, PoseStamped, self._recv_map_origin)

    def _create_services(self):
        self.services['start_mission'] = rospy.Service('start_mission', Trigger, self._start_mission)
        self.services['abort_mission'] = rospy.Service('abort_mission', Trigger, self._abort_mission)
        self.services['start_mission_no_plan'] = rospy.Service('start_mission_no_plan', Trigger,
                                                               self._start_mission_no_plan)
        self.services['start_heartbeat'] = rospy.Service('start_heartbeat', Trigger,
                                                         self._start_heartbeat)
        self.services['stop_heartbeat'] = rospy.Service('stop_heartbeat', Trigger,
                                                        self._stop_heartbeat)

    def _start_mission(self, request):
        rospy.loginfo('Starting mission')
        if self.state == State.IDLE:
            self.state = State.PLAN
            return TriggerResponse(True, 'Mission started.')
        else:
            return TriggerResponse(False, 'Mission in progress.')

    def _abort_mission(self, request):
        rospy.loginfo('Aborting mission')
        if self.state not in (State.COMPLETE, State.IDLE):
            self.state = State.COMPLETE
            return TriggerResponse(True, 'Mission aborted.')
        else:
            return TriggerResponse(False, 'Mission not running.')

    def _start_heartbeat(self, request):
        rospy.loginfo('Starting heartbeat')
        self.heartbeat_active = True
        return TriggerResponse(True, 'Heartbeat started.')

    def _stop_heartbeat(self, request):
        rospy.loginfo('Stopping heartbeat')
        self.heartbeat_active = False
        return TriggerResponse(True, 'Heartbeat stopped.')

    def _start_mission_no_plan(self, request):
        rospy.loginfo('Starting mission')
        if self.state == State.IDLE:
            self.state = State.PREP
            return TriggerResponse(True, 'Mission started.')
        else:
            return TriggerResponse(False, 'Mission in progress.')

    def _recv_map_origin(self, data):
        map_origin_utm = utm.fromLatLong(
            data.pose.position.y,
            data.pose.position.x,
            data.pose.position.z)

        self.map_origin_point = np.array((
            map_origin_utm.toPoint().x,
            map_origin_utm.toPoint().y))

        self.subscribers['map_origin'].unregister()


"""
import pydevd_pycharm
pydevd_pycharm.settrace('localhost', port=7778, stdoutToServer=True, stderrToServer=True)
"""
if __name__ == '__main__':
    rospy.init_node('pegasus_controller')
    rospy.loginfo('Starting pegasus_controller...')
    agents = rospy.get_param('~agents')
    z_height = rospy.get_param('~agents_hover_height')
    map_origin_topic = rospy.get_param('~map_origin_topic')
    calibration_size = rospy.get_param('~calibration_size')
    rospy.loginfo(agents[0])
    controller = PegasusController({
        'agents': agents,
        'z_height': z_height,
        'map_origin_topic': map_origin_topic,
        'calibration_size': calibration_size,
    })
    controller_thread = ControllerThread(controller, 0)
    controller_thread.start()
    controller.spin()
    rospy.spin()
