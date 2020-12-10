import math
import numpy as np
from cell import Cell


class CellContainer(object):
    MOVE = {
        'RIGHT': 0,
        'LEFT': 1,
        'UP': 2,
        'DOWN': 3,
        'RIGHT-UP': 4,
        'LEFT-UP': 5,
        'LEFT-DOWN': 6,
        'RIGHT-DOWN': 7,
        'STAY': 8,
    }

    MOVEMENT = np.array([
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1],
        [1, 1],
        [-1 , 1],
        [-1, -1],
        [1, -1],
        [0, 0]
    ], dtype=int)

    def __init__(self, bounding_box, grid_cells, cell_size, num_directions=9, agents_hover_height=10):
        self.i_max, self.j_max = grid_cells.min_max
        self.bounding_box = bounding_box
        self.grid_cells = grid_cells
        self.cell_size = cell_size
        self.valid_cells = None
        self.agents_hover_height = agents_hover_height
        self.NUM_DIRECTIONS = num_directions
        self.cells = [None] * self.i_max
        self.cell_index = np.full((self.i_max, self.j_max), -1, dtype=int)
        self.cell_positions = None
        for i in range(self.i_max):
            self.cells[i] = [None] * self.j_max
        self.valid_cells = []
        self.fill_cells()
        print (self.cell_index)
        print (self.cell_positions)

    def fill_cells(self):
        for k in range(self.i_max * self.j_max):
            ith, jth = self.grid_cells.get_index(self.i_max, self.j_max, k)
            self.cells[ith][jth] = Cell((ith, jth),
                                        self.grid_cells.cells_center[k],
                                        self.grid_cells.valid[k])
            if self.grid_cells.valid[k]:
                if self.cell_positions is None:
                    self.cell_positions = np.array([self.cells[ith][jth].data[2:4]], dtype=float)
                else:
                    self.cell_positions = np.append(self.cell_positions, [self.cells[ith][jth].data[2:4]], axis=0)
                self.cell_index[ith, jth] = self.cell_positions.shape[0] - 1
                self.valid_cells.append(self.cells[ith][jth])

    def check_validity(self, index):
        try:
            result = self.cell_index[index[0], index[1]]
        except Exception as e:
            raise e
        else:
            if result == -1:
                raise Exception('Invalid grid range %s.' % str(index))

    def move(self, direction, current_index):
        new_index = np.add(current_index, self.MOVEMENT)
        index = new_index[direction * 2 if self.NUM_DIRECTIONS == 4 else direction]
        self.check_validity(index)
        return index

    def move_and_get_cell(self, direction, current_cell=None, current_index=None):
        if current_cell is None and current_index is None:
            raise Exception('Need either cell or currentIndex')
        if current_cell is not None:
            current_index = current_cell.index
        i, j = self.move(direction, current_index)
        return self.cells[int(i)][int(j)]

    def position_to_index(self, position):
        x, y = position
        i = int(math.floor((x - self.bounding_box[0, 0]) / self.cell_size))
        j = int(math.floor((y - self.bounding_box[0, 1]) / self.cell_size))
        self.check_validity((i, j))
        return i, j

    def position_to_cell(self, position):
        i, j = self.position_to_index(position)
        return self.cells[i][j]
