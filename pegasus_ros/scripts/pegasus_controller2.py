#!/usr/bin/env python

"""
Pegasus Controller
"""
import rospy
from pegasus_controller2.agent import Agent
import pegasus_controller2.states as states


class PegasusController(object):
    def __init__(self, params):
        rospy.loginfo(params)
        self.z_height = params['agents_hover_height']
        self.grid_size = params['grid_size']
        self.map_origin_topic = params['map_origin_topic']
        self.agents = []
        for i, agent in enumerate(params['agents']):
            self.agents.append(Agent(i, self, agent[0], (agent[1], agent[2])))
        self.state = states.IdleState()
        self.state.set_controller(self)

    def start(self):
        for agent in self.agents:
            agent.create_agent_socket()
            agent.start_rx()
            agent.register_publisher()

    def run_state_machine(self):
        if self.state.is_complete():
            self.state = self.state.next()
            self.state.set_controller(self)
        self.state.run()

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            for agent in self.agents:
                agent.spin(rospy.get_rostime())
            self.run_state_machine()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                pass


# PyCharm debug server
import pydevd_pycharm
pydevd_pycharm.settrace('localhost', port=7777,
                        stdoutToServer=True, stderrToServer=True)

if __name__ == '__main__':
    rospy.init_node('pegasus_controller')
    rospy.loginfo('Starting pegasus_controller')
    z_height = rospy.get_param('~agents_hover_height')
    grid_size = rospy.get_param('~grid_size')
    map_origin_topic = rospy.get_param('~map_origin_topic')
    agents = rospy.get_param('~agents')
    controller = PegasusController({
        'agents_hover_height': float(z_height),
        'grid_size': float(grid_size),
        'map_origin_topic': map_origin_topic,
        'agents': agents})

    controller.start()
    controller.spin()
    rospy.spin()
