import time
import copy
import numpy as np
import rospy
from state import State
import copy
import numpy as np
import time
from state import State


class PathFinder(object):
    def __init__(self, agents, cell_container):
        self.agents = agents
        self.cell_container = cell_container

    def get_movement(self, conf):
        pass

    def get_initial_state(self):
        visited_cells = np.zeros((self.cell_container.i_max, self.cell_container.j_max), dtype=float)
        state = State(0, visited_cells, self.cell_container)
        # TODO change this algorithm
        for i, agent in enumerate(self.agents):
            # cell = self.cell_container.cells[i][i]

            direction = self.cell_container.MOVE['STAY']
            while direction >= 0:
                try:
                    cell = self.cell_container.position_to_cell(agent.initial_position)
                    cell = self.cell_container.move_and_get_cell(direction, current_cell=cell)
                    direction -= 1
                    state.add_agent_action(agent.a_id, 99, cell)
                    break
                except:
                    pass

            state.check_constraints(self.cell_container.NUM_DIRECTIONS)
            state.update_visited_cells()
        state.calculate_G(self.cell_container.NUM_DIRECTIONS)
        state.calculate_H(self.cell_container.valid_cells)
        return state

    def get_neighbour(self, state, movement):
        agent_ctr = self.getNextAgent(state.agent_ctr)
        agent = self.agents[agent_ctr]
        new_state = State(state.g, state.visited_cells, self.cell_container)
        cell = self.cell_container.move_and_get_cell(movement, state.agent_cells[agent.a_id])
        new_state.add_agent_action(agent.a_id, movement, cell, state.agent_cells, state.movement_counter)
        new_state.check_constraints(self.cell_container.NUM_DIRECTIONS, state)
        new_state.update_visited_cells()
        new_state.calculate_G(self.cell_container.NUM_DIRECTIONS)
        new_state.calculate_H(self.cell_container.valid_cells)
        new_state.set_agent_ctr(agent_ctr)
        return new_state

    def getNextAgent(self, agent_ctr):
        return (agent_ctr + 1) % len(self.agents)

    def print_parents(self, state):
        current = state
        rospy.loginfo('--------child start------------')
        while current is not None:
            rospy.loginfo(current)
            current = current.parent
        rospy.loginfo('--------parent end------------')

    def get_depth(self, state):
        i = 0;
        current = state
        while current is not None:
            i += 1
            current = current.parent
        return i

    @staticmethod
    def get_base_parent(state):
        current = state
        while current.parent is not None:
            current = current.parent
        return current

    def concatenate_goals(self, goals):
        goal_last = None
        for g in goals:
            base = self.get_base_parent(g)
            if goal_last is not None and goal_last.parent is not None:
                base.setParent(goal_last.parent)
            goal_last = g
        return goal_last

    def trim_goals(self, goal):
        current = goal
        h = current.h
        while current.parent is not None:
            if current.parent.h > h:
                return current
            current = current.parent
        return goa

    def search(self, depth_exit=0, epoch_stop=0, previous_goal=None):
        num_conf = self.cell_container.NUM_DIRECTIONS
        open_list = []
        closed_list = []

        early_exit = None

        initial_state = None

        if previous_goal:
            previous_goal.setParent(None)
            initial_state = previous_goal
        else:
            try:
                initial_state = self.get_initial_state()
            except Exception as e:
                rospy.logerr(e)
                return 'not-found', None

        open_list.append(initial_state)

        if epoch_stop != 0:
            # If heuristics does not increase for certain epochs..then stop
            early_exit = {
                'h': len(self.cell_container.valid_cells),
                'epochs': 0,
                'minCostState': initial_state
            }

        while True:
            if len(open_list) == 0:
                return 'not-found', None

            # current - lowest f() in open list
            current = open_list[0]
            for state in open_list:
                if current.f() > state.f():
                    current = state

            open_list.remove(current)

            if epoch_stop != 0:
                if current.h < early_exit['h']:
                    early_exit['h'] = current.h
                    early_exit['epochs'] = 0
                    early_exit['minCostState'] = current
                else:
                    early_exit['epochs'] += 1
                    if early_exit['minCostState'].f() > current.f():
                        early_exit['minCostState'] = current

                # early exit
                if early_exit['epochs'] > epoch_stop:
                    return 'early-exit', early_exit['minCostState']

            if depth_exit != 0 and self.get_depth(current) > depth_exit:
                return 'depth-exit', current.parent

            if current.h == 0:
                return 'found', current

            # confgs = [x for x in range(numConf)]
            for cnf in range(num_conf):
                # while len(confgs) > 0:
                successor = None
                successor_cost = current.g + 1
                # print (successorCost)
                try:
                    # cnf = confgs.pop(random.randrange(len(confgs)))
                    successor = self.get_neighbour(current, cnf)
                except Exception as e:
                    # print (e, current.h)
                    continue
                if successor in open_list:
                    if successor.g <= successor_cost:
                        continue
                elif successor in closed_list:
                    if successor.g <= successor_cost:
                        continue
                    open_list.append(successor)
                    closed_list.remove(successor)
                else:
                    open_list.append(successor)
                successor.g = successor_cost
                successor.setParent(current)

            closed_list.append(current)

    def find(self, depth_exit, sigma_t, early_exit):
        start_time = time.time()
        goal = None
        goals = []
        prev_h = len(self.cell_container.valid_cells)
        sigma = 0
        while True:
            ret, goal = self.search(epoch_stop=early_exit, depth_exit=depth_exit, previous_goal=copy.deepcopy(goal))
            rospy.loginfo(ret)
            goals.append(goal)
            h = goal.h
            rospy.loginfo('h= %s, g= %s, f=%s' % (h, goal.g, goal.f()))
            rospy.loginfo(goal.visited_cells.T)
            if prev_h <= h:
                # Does not converge
                sigma += 1
                if sigma >= sigma_t:
                    break
            elif h == 0:
                # converged
                break
            else:
                sigma = 0
            prev_h = h
        rospy.loginfo("Total Time Taken: %s seconds." % (time.time() - start_time,))
        final_goal = self.concatenate_goals(goals)
        final_goal = self.trim_goals(final_goal)
        rospy.loginfo(final_goal.visited_cells.T)
        rospy.loginfo("Steps : %s " % (self.get_depth(final_goal)))
        return final_goal

    @staticmethod
    def get_movement_plan_from_goal(final_goal):
        num_agents = len(final_goal.agent_cells.values())
        agents_pose = []
        for i in range(num_agents):
            agents_pose.append({'points': [], 'steps': []})
        cur = final_goal
        while cur is not None:
            for i, agent_id in enumerate(cur.agent_cells):
                # truncate all steps which are redundant
                if len(agents_pose[i]['steps']) > 0 and cur.movement_counter[agent_id] >= agents_pose[i]['steps'][-1]:
                    continue
                agents_pose[i]['points'].append((cur.agent_cells[agent_id].position[0],
                                                 cur.agent_cells[agent_id].position[1]))
                agents_pose[i]['steps'].append(cur.movement_counter[agent_id])
            cur = cur.parent
        for i in range(num_agents):
            agents_pose[i]['points'].reverse()
        return agents_pose
