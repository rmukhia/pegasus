import math
import numpy as np

from constraints import CONSTRAINTS
from grid_cells import is_intersecting


class StateHelper(object):
    @staticmethod
    def f(state):
        return state.g + state.h

    @staticmethod
    def set_parent(state, parent):
        state.parent = parent

    @staticmethod
    def constraint1(state, num_directions):
        moved = False
        # At least one agent needs to move
        if 99 in state.movements.values():
            # Initial Condition
            moved = True
        else:
            for movement in state.movements.values():
                if movement < num_directions:
                    moved = True
                    break
        if not moved:
            raise Exception('At least one agent needs to move')

    @staticmethod
    def constraint2(state):
        # Check distance between two agents
        max_distance = CONSTRAINTS['MAX_DISTANCE']
        if len(state.agent_cells.values()) == 1:
            return
        for id_l in state.agent_cells:
            one_agent_in_range = False
            for id_m in state.agent_cells:
                if id_l == id_m:
                    continue
                # pythogorims theorem
                x1, y1 = state.agent_cells[id_l].position
                x2, y2 = state.agent_cells[id_m].position
                xx = x1 - x2
                yy = y1 - y2
                dist = math.sqrt(xx * xx + yy * yy)
                if dist <= max_distance:
                    one_agent_in_range = True
                    break
            if not one_agent_in_range:
                raise Exception('Agents cannot leave each others sphere.')

    @staticmethod
    def constraint3(state):
        # check distance of any agent with controlStation
        max_distance = CONSTRAINTS['MAX_DISTANCE']
        control_station = CONSTRAINTS['CS_POSITION']
        num_agents = len(state.agent_cells.values())

        # At least one agent should be within reach.
        cpos = np.full((num_agents, 2), control_station[0])
        apos = np.zeros((num_agents, 2))

        for i, id_l in enumerate(state.agent_cells):
            apos[i] = state.agent_cells[id_l].position

        dist = np.sqrt(np.sum((cpos - apos) ** 2, axis=1))

        if np.min(dist) >= max_distance:
            raise Exception('At least one agent needs to be in control station range.')

    @staticmethod
    def constraint4(state, old_state):
        # two agents cannot swap position
        for id_n in state.agent_cells:
            for id_m in old_state.agent_cells:
                if id_n == id_m:
                    continue
                if state.agent_cells[id_n] == old_state.agent_cells[id_m] and \
                        state.agent_cells[id_m] == old_state.agent_cells[id_n]:
                    raise Exception('Agents cannot swap position.')

    @staticmethod
    def constraint5(state, old_state):
        # two agents should not intersect each others line
        # line equation for paths.
        num_agents = len(state.agent_cells.values())
        line_eqs = np.zeros((num_agents, 4))
        for i, id_a in enumerate(state.agent_cells):
            line_eqs[i, 0:2] = state.agent_cells[id_a].position
            line_eqs[i:, 2:4] = old_state.agent_cells[id_a].position

        for i in range(num_agents):
            eq = line_eqs[i, :]
            other_eqs = np.delete(line_eqs, [i], axis=0)
            if np.sum(is_intersecting(eq, other_eqs)) > 0:
                raise Exception('Agents cannot have intersecting paths.')

    @staticmethod
    def check_constraints(state, num_directions, old_state=None):
        StateHelper.constraint1(state, num_directions)
        StateHelper.constraint2(state)
        StateHelper.constraint3(state)

        if old_state is not None:
            StateHelper.constraint4(state, old_state)
            StateHelper.constraint5(state, old_state)

    @staticmethod
    def add_agent_action(state, agent_id, movement, new_cell, agent_cells=None, movement_counter=None):
        if agent_cells is not None:
            state.agent_cells = agent_cells.copy()

        if movement_counter is not None:
            state.movement_counter = movement_counter.copy()

        if new_cell in state.agent_cells.values():
            raise Exception('Invalid move, two agents cannot occupy the same node')
        state.movements[agent_id] = movement
        state.agent_cells[agent_id] = new_cell
        if agent_id not in state.movement_counter:
            state.movement_counter[agent_id] = 0
        state.movement_counter[agent_id] += 1

        state.current_agent_id = agent_id

    @staticmethod
    def set_agent_ctr(state, agent_ctr):
        state.agent_ctr = agent_ctr

    @staticmethod
    def update_visited_cells(state):
        agent_id = state.current_agent_id
        if state.visited_cells[state.agent_cells[agent_id].index] == 0:
            state.visited_cells[state.agent_cells[agent_id].index] = 1
        else:
            state.visited_cells[state.agent_cells[agent_id].index] *= 2

    @staticmethod
    def calculate_G(state, num_directions):
        # print(state.agent_cells)
        agent_id = state.current_agent_id
        # stay move is numDirection, so has to be <=
        if 0 <= state.movements[agent_id] <= num_directions:
            if state.visited_cells[state.agent_cells[agent_id].index] == 0:
                state.g += state.visited_cells[state.agent_cells[agent_id].index] + 1
            else:
                state.g += state.visited_cells[state.agent_cells[agent_id].index] * 2

    @staticmethod
    def calculate_H(state, valid_cells):
        # call this after all add cells
        for cell in state.cell_container.valid_cells:
            if state.visited_cells[cell.index] == 0:
                state.h += 1
        return state.h


class State(object):
    def __init__(self, prev_g, prev_visited_cells, cell_container):
        self.g = prev_g
        self.h = 0
        self.visited_cells = np.copy(prev_visited_cells)
        self.agent_cells = {}
        self.agent_position = {}
        self.movements = {}
        self.movement_counter = {}
        self.parent = None
        self.agent_ctr = -1  # start from 0
        self.current_agent_id = None
        self.cell_container = cell_container

    def __eq__(self, other_state):
        # if the agents occupy the same position, then the state is the same
        for agentId in self.agent_cells:
            if self.agent_cells[agentId] != other_state.agent_cells[agentId]:
                return False
        return True

    def __str__(self):
        return str(self.agent_cells) + ' g: %s, h: %s %s' % (self.g, self.h, str(self.movements))

    def __repr__(self):
        self.__str__()
