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
        _index = state.agent_cells[agent_id].get_np_index()
        index = state.cell_container.cell_index[_index]
        movement = state.movements[agent_id]
        diag_cost = 1.44
        if state.cell_cost[index] == 0:
            state.cell_cost[index] = 1 if movement <= state.cell_container.MOVE['DOWN'] else diag_cost
        else:
            state.cell_cost[index] *= 3
            if movement >= state.cell_container.MOVE['DOWN']:
                state.cell_cost[index] += diag_cost

    @staticmethod
    def calculate_G(state, num_directions):
        state.g = np.sum(state.cell_cost)

    @staticmethod
    def calculate_H(state):
        empty_cells = np.equal(state.cell_cost, [0.])
        num_agents = len(state.agent_cells.values())
        empty_ones = state.cell_container.cell_data[empty_cells, 0:2]
        # Number of free cells
        state.h = empty_ones.shape[0]
        if state.h == 0:
            return
        # Process now
        empty_ones = np.tile(empty_ones, (num_agents,1))
        curr_pos = None
        for i in range(num_agents):
            _index = state.agent_cells.values()[i].get_np_index()
            if curr_pos is None:
                curr_pos = np.array([_index, ] * state.h)
            else:
                curr_pos = np.append(curr_pos, np.array([_index, ] * state.h), axis=0)
        dist = np.sqrt(np.sum((empty_ones - curr_pos) ** 2, axis=1))
        heuristic = np.median(dist)
        state.h = float(state.h)
        if heuristic > 1.:
            state.h += heuristic - 1
        # state.h += heuristic
        return state.h
