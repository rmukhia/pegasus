import math
import numpy as np

from constraints import CONSTRAINTS
from grid_cells import is_intersecting


class State(object):
    def __init__(self, prev_g, prev_visited_cells, cell_container):
        self.g = prev_g
        self.h = 0
        self.visited_cells = np.copy(prev_visited_cells)
        self.agent_cells = {}
        self.movements = {}
        self.movement_counter = {}
        self.parent = None
        self.agent_ctr = -1  # start from 0
        self.current_agent_id = None
        self.cell_container = cell_container

    def f(self):
        return self.g + self.h

    def setParent(self, parent):
        self.parent = parent

    def constraint1(self, num_directions):
        moved = False
        # At least one agent needs to move
        if 99 in self.movements.values():
            # Initial Condition
            moved = True
        else:
            for movement in self.movements.values():
                if movement < num_directions:
                    moved = True
                    break
        if not moved:
            raise Exception('At least one agent needs to move')

    def constraint2(self):
        # Check distance between two agents
        max_distance = CONSTRAINTS['MAX_DISTANCE']
        if len(self.agent_cells.values()) == 1:
            return
        for id_l in self.agent_cells:
            one_agent_in_range = False
            for id_m in self.agent_cells:
                if id_l == id_m:
                    continue
                # pythogorims theorem
                x1, y1 = self.agent_cells[id_l].position
                x2, y2 = self.agent_cells[id_m].position
                xx = x1 - x2
                yy = y1 - y2
                dist = math.sqrt(xx * xx + yy * yy)
                if dist <= max_distance:
                    one_agent_in_range = True
                    break
            if not one_agent_in_range:
                raise Exception('Agents cannot leave each others sphere.')

    def constraint3(self):
        # check distance of any agent with controlStation
        max_distance = CONSTRAINTS['MAX_DISTANCE']
        control_station = CONSTRAINTS['CS_POSITION']
        num_agents = len(self.agent_cells.values())

        # At least one agent should be within reach.
        cpos = np.full((num_agents, 2), control_station[0])
        apos = np.zeros((num_agents, 2))

        for i, id_l in enumerate(self.agent_cells):
            apos[i] = self.agent_cells[id_l].position

        dist = np.sqrt(np.sum((cpos - apos) ** 2, axis=1))

        if np.min(dist) >= max_distance:
            raise Exception('At least one agent needs to be in control station range.')

    def constraint4(self, old_state):
        # two agents cannot swap position
        for id_n in self.agent_cells:
            for id_m in old_state.agent_cells:
                if id_n == id_m:
                    continue
                if self.agent_cells[id_n] == old_state.agent_cells[id_m] and \
                        self.agent_cells[id_m] == old_state.agent_cells[id_n]:
                    raise Exception('Agents cannot swap position.')

    def constraint5(self, old_state):
        # two agents should not intersect each others line
        # line equation for paths.
        num_agents = len(self.agent_cells.values())
        line_eqs = np.zeros((num_agents, 4))
        for i, id_a in enumerate(self.agent_cells):
            line_eqs[i, 0:2] = self.agent_cells[id_a].position
            line_eqs[i:, 2:4] = old_state.agent_cells[id_a].position

        for i in range(num_agents):
            eq = line_eqs[i, :]
            other_eqs = np.delete(line_eqs, [i], axis=0)
            if np.sum(is_intersecting(eq, other_eqs)) > 0:
                raise Exception('Agents cannot have intersecting paths.')

    def check_constraints(self, num_directions, old_state=None):
        self.constraint1(num_directions)
        self.constraint2()
        self.constraint3()

        if old_state is not None:
            self.constraint4(old_state)
            self.constraint5(old_state)

    def add_agent_action(self, agent_id, movement, new_cell, agent_cells=None, movement_counter=None):
        if agent_cells is not None:
            self.agent_cells = agent_cells.copy()

        if movement_counter is not None:
            self.movement_counter = movement_counter.copy()

        if new_cell in self.agent_cells.values():
            raise Exception('Invalid move, two agents cannot occupy the same node')
        self.movements[agent_id] = movement
        self.agent_cells[agent_id] = new_cell
        if agent_id not in self.movement_counter:
            self.movement_counter[agent_id] = 0
        self.movement_counter[agent_id] += 1

        self.current_agent_id = agent_id

    def set_agent_ctr(self, agent_ctr):
        self.agent_ctr = agent_ctr

    def update_visited_cells(self):
        agent_id = self.current_agent_id
        if self.visited_cells[self.agent_cells[agent_id].index] == 0:
            self.visited_cells[self.agent_cells[agent_id].index] = 1
        else:
            self.visited_cells[self.agent_cells[agent_id].index] *= 2

    def calculate_G(self, num_directions):
        agent_id = self.current_agent_id
        # stay move is numDirection, so has to be <=
        if 0 <= self.movements[agent_id] <= num_directions:
            if self.visited_cells[self.agent_cells[agent_id].index] == 0:
                self.g += self.visited_cells[self.agent_cells[agent_id].index] + 1
            else:
                self.g += self.visited_cells[self.agent_cells[agent_id].index] * 2

    def calculate_H(self, valid_cells):
        # call this after all add cells
        for cell in self.cell_container.valid_cells:
            if self.visited_cells[cell.index] == 0:
                self.h += 1
        return self.h

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
