import copy
import math
import numpy as np
import time
import rospy
import random
# import traceback
from pegasus_planner.constraints import CONSTRAINTS
from state_helper import StateHelper
from state import State, StateKeyWrapper

timing = 0


class PathFinder(object):
    def __init__(self, agents, cell_container):
        self.agents = agents
        self.cell_container = cell_container

    def get_movement(self, conf):
        pass

    def get_initial_state(self):
        cell_cost = np.zeros((self.cell_container.get_size_valid_cells()), dtype=float)
        state = State(cell_cost, self.cell_container)
        # TODO change this algorithm
        for i, agent in enumerate(self.agents):
            # cell = self.cell_container.cells[i][i]
            direction = self.cell_container.MOVE['STAY']
            while direction >= 0:
                try:
                    cell = self.cell_container.position_to_cell(agent.initial_position)
                    cell = self.cell_container.move_and_get_cell(direction, current_cell=cell)
                    StateHelper.add_agent_action(state, agent.a_id, 99, cell)
                    break
                except:
                    # traceback.print_exc()
                    pass
                direction -= 1
            StateHelper.check_constraints(state, self.cell_container.NUM_DIRECTIONS)
            StateHelper.update_visited_cells(state)
        StateHelper.calculate_G(state, self.cell_container.NUM_DIRECTIONS)
        StateHelper.calculate_H(state)
        return state

    def get_neighbour(self, state, movement):
        agent_ctr = self.getNextAgent(state.agent_ctr)
        agent = self.agents[agent_ctr]
        new_state = State(state.cell_cost, self.cell_container)
        cell = self.cell_container.move_and_get_cell(movement, state.agent_cells[agent.a_id])
        StateHelper.add_agent_action(new_state, agent.a_id, movement, cell, state.agent_cells, state.movement_counter)
        StateHelper.check_constraints(new_state, self.cell_container.NUM_DIRECTIONS, state)
        StateHelper.update_visited_cells(new_state)
        StateHelper.calculate_G(new_state, self.cell_container.NUM_DIRECTIONS)
        StateHelper.calculate_H(new_state)
        StateHelper.set_agent_ctr(new_state, agent_ctr)
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
        i = 0
        current = state
        while current is not None:
            i += 1
            current = current.parent
        return i

    def search(self, depth_exit=0, epoch_stop=0, previous_goal=None, configurations=range(9)):
        open_list = []
        closed_list = []

        early_exit = None

        initial_state = None

        if previous_goal:
            StateHelper.set_parent(previous_goal, None)
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
                'cost': len(self.cell_container.valid_cells),
                'epochs': 0,
                'minCostState': initial_state
            }

        while True:
            if len(open_list) == 0:
                return 'not-found', current

            #for s in open_list:
            #    print(s)

            # current - lowest f() in open list
            current = open_list.pop(0)
            # rospy.loginfo('start state cost g: %s, h: %s', current.g, current.h)
            if epoch_stop != 0:
                if CONSTRAINTS['HEURISTIC_TYPE'] == 1 and current.h < early_exit['cost']:
                    early_exit['cost'] = current.h
                    early_exit['epochs'] = 0
                    early_exit['minCostState'] = current
                elif CONSTRAINTS['HEURISTIC_TYPE'] == 2 and StateHelper.get_free_cell_number(current)[0] < early_exit['cost']:
                    early_exit['cost'] = StateHelper.get_free_cell_number(current)[0]
                    early_exit['epochs'] = 0
                    early_exit['minCostState'] = current
                else:
                    early_exit['epochs'] += 1
                    if StateHelper.f(early_exit['minCostState']) > StateHelper.f(current):
                        early_exit['minCostState'] = current

                # early exit
                if early_exit['epochs'] > epoch_stop:
                    return 'early-exit', early_exit['minCostState']

            if depth_exit != 0 and self.get_depth(current) > depth_exit:
                return 'depth-exit', early_exit['minCostState']
                # return 'depth-exit', current.parent

            if current.h == 0:
                return 'found', current

            closed_list.append(current)

            # for cnf in random.sample(range(num_conf), num_conf):
            for cnf in configurations:
                successor = None
                try:
                    successor = self.get_neighbour(current, cnf)
                except Exception as e:
                    # print e
                    continue

                if successor in closed_list:
                    continue
                    '''
                    index = closed_list.index(successor)
                    prev_node = closed_list[index]
                    if prev_node.g > successor.g:
                        StateHelper.set_parent(successor, current)
                        closed_list.pop(index)
                        StateKeyWrapper.insert_sorted(open_list, successor)
                        continue
                    '''

                if successor in open_list:
                    index = open_list.index(successor)
                    prev_node = open_list[index]
                    if prev_node.g > successor.g:
                        # print ('Prev node g %s new g %s' % (prev_node.g, successor.g))
                        StateHelper.set_parent(successor, current)
                        open_list.pop(index)
                        StateKeyWrapper.insert_sorted(open_list, successor)
                else:
                    StateHelper.set_parent(successor, current)
                    StateKeyWrapper.insert_sorted(open_list, successor)

    def get_weighted_curve(self, current_h, depth_exit, c):
        total_h = self.cell_container.get_size_valid_cells()
        # power curve
        # y = ax^c + b, a = 1, b= 0, c = 4

        # limit to 0
        total_covered = total_h - current_h if total_h - current_h >= 0 else 0
        x = float(total_covered)/total_h
        a = 1
        b = 0
        print (x, a, b, c)
        y = a * math.pow(x, c) + b
        weight = y * depth_exit
        # cutoff at 2 least
        if weight < 2:
            weight = 2
        # cutoff at depth_exit
        if weight > depth_exit:
            weight = depth_exit
        rospy.loginfo ('Depth weight %s', int(weight))
        return int(weight)

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
                StateHelper.set_parent(base, goal_last.parent)
            goal_last = g
        return goal_last

    def trim_goals(self, goal):
        current = goal
        h = current.h
        while current.parent is not None:
            if current.parent.h > h:
                return current
            current = current.parent
        return goal

    def remove_inactive_steps(self, goal):
        real_goal = goal
        # remove dangling inactive steps
        while real_goal.parent is not None:
            movement = real_goal.movements.values()[0]
            if movement != real_goal.cell_container.MOVE['STAY']:
                break
            real_goal = real_goal.parent
        current = real_goal
        while current.parent is not None and current.parent.parent is not None:
            parent = current.parent
            movement = parent.movements.values()[0]
            if movement == real_goal.cell_container.MOVE['STAY']:
                current.parent = parent.parent
                current = parent.parent
            current = parent
        return real_goal

    def find(self, depth_exit, sigma_t, early_exit, c_power):
        start_time = time.time()
        goal = None
        goals = []
        prev_h = self.cell_container.get_size_valid_cells()
        sigma = 0
        depth_exit_weight = self.get_weighted_curve(int(prev_h), depth_exit, c_power)
        start_h = 0
        # Get the configurations list
        num_conf = self.cell_container.NUM_DIRECTIONS
        configurations_list = [
            list(range(num_conf)),  # normal
            list(range(num_conf-1, -1, -1)),  # reverse
        ]
        for i in range(2, sigma_t):
            configurations_list.append(random.sample(configurations_list[0], num_conf))

        configurations = configurations_list[0]
        while True:
            ret, goal = self.search(
                epoch_stop=early_exit,
                depth_exit=depth_exit_weight,
                previous_goal=copy.deepcopy(goal),
                configurations=configurations)
            configurations = configurations_list[0]  # default configuration is normal sequence of movements
            rospy.loginfo(ret)
            goals.append(goal)
            h = goal.h
            if CONSTRAINTS['HEURISTIC_TYPE'] == 2:
                h = StateHelper.get_free_cell_number(goal)[0]
            depth_exit_weight = self.get_weighted_curve(h, depth_exit, c_power)
            rospy.loginfo('h= %s,  g= %s, f=%s depth_weight= %s' % (h, goal.g, StateHelper.f(goal), depth_exit_weight))
            rospy.loginfo(goal.cell_cost)
            if prev_h <= h:
                # Does not converge
                sigma += 1
                if sigma >= sigma_t:
                    goals = goals[0:-sigma]
                    break
            elif h == 0:
                # converged
                break
            else:
                sigma = 0
            prev_h = h
        rospy.loginfo("Total Time Taken: %s seconds.", time.time() - start_time)
        rospy.loginfo("Total Cells to Travel %s.", self.cell_container.cell_data.shape[0])
        final_goal = self.concatenate_goals(goals)
        final_goal = self.trim_goals(final_goal)
        if CONSTRAINTS['HEURISTIC_TYPE'] == 2:
            final_goal = self.remove_inactive_steps(final_goal)
        rospy.loginfo("Steps : %s " % (self.get_depth(final_goal)))
        rospy.loginfo(final_goal.cell_cost)
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
