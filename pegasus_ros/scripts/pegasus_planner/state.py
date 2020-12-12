import numpy as np
from bisect import bisect_right
from state_helper import StateHelper


class StateKeyWrapper:
    def __init__(self, iterable, key):
        self.it = iterable
        self.key = key

    def __getitem__(self, item):
        return self.key(self.it[item])

    def __len__(self):
        return len(self.it)

    @staticmethod
    def insert_sorted(state_list, state):
        bsrindex = bisect_right(StateKeyWrapper(state_list, key=lambda s: StateHelper.f(s)), StateHelper.f(state))
        state_list.insert(bsrindex, state)


class State(object):
    def __init__(self, prev_cell_cost, cell_container):
        self.g = 0
        self.h = 0
        # cell cost is linked with cell_container.cell_matrix
        self.cell_cost = np.copy(prev_cell_cost)
        self.agent_cell_index = {}
        self.agent_cells = {}
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
