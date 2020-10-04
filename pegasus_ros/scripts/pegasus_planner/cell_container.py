import math

from cell import Cell


class CellContainer(object):
    MOVE = {
        'RIGHT': 0,
        'RIGHT-UP': 1,
        'UP': 2,
        'LEFT-UP': 3,
        'LEFT': 4,
        'LEFT-DOWN': 5,
        'DOWN': 6,
        'RIGHT-DOWN': 7,
        'STAY': 8,
    }

    def __init__(self, bounding_box, grid_cells, cell_size, num_directions=4, agents_hover_height=10):
        self.i_max, self.j_max = grid_cells.min_max
        self.bounding_box = bounding_box
        self.grid_cells = grid_cells
        self.cell_size = cell_size
        self.valid_cells = None
        self.agents_hover_height = agents_hover_height
        self.NUM_DIRECTIONS = num_directions
        self.cells = [None] * self.i_max
        for i in range(self.i_max):
            self.cells[i] = [None] * self.j_max
        self.valid_cells = []
        self.fill_cells()

    def fill_cells(self):
        for k in range(self.i_max * self.j_max):
            ith, jth = self.grid_cells.get_index(self.i_max, self.j_max, k)
            self.cells[ith][jth] = Cell((ith, jth),
                                        self.grid_cells.cells_center[k],
                                        self.grid_cells.valid[k])
            if self.grid_cells.valid[k]:
                self.valid_cells.append(self.cells[ith][jth])

    def check_valid_cell(self, index):
        valid = False
        for cell in self.valid_cells:
            if cell.index[0] == index[0] and cell.index[1] == index[1]:
                valid = True
        if not valid:
            raise Exception('Invalid grid range %s.' % str(index))

    def check_valid_index(self, index):
        if (self.i_max <= index[0] or index[0] < 0
                or self.j_max <= index[1] or index[1] < 0):
            raise Exception('Out of bounds grid range %s.' % str(index))

    def move(self, direction, current_index):
        i, j = current_index
        new_index_list = [
            (i + 1, j),  # RIGHT
            (i + 1, j + 1),  # RIGHT-UP
            (i, j + 1),  # UP
            (i - 1, j + 1),  # LEFT-UP
            (i - 1, j),  # LEFT
            (i - 1, j - 1),  # LEFT-DOWN
            (i, j - 1),  # DOWN
            (i + 1, j - 1),  # RIGHT-DOWN
            (i, j),  # STAY
        ]

        new_index = new_index_list[direction * 2 if self.NUM_DIRECTIONS == 4 else direction]
        self.check_valid_index(new_index)
        self.check_valid_cell(new_index)
        return new_index

    def move_and_get_cell(self, direction, current_cell=None, current_index=None):
        if current_cell is None and current_index is None:
            raise Exception('Need either cell or currentIndex')

        if current_cell is not None:
            current_index = current_cell.index
        i, j = self.move(direction, current_index)
        return self.cells[i][j]

    def position_to_index(self, position):
        x, y = position
        i = int(math.floor((x - self.bounding_box[0, 0]) / self.cell_size))
        j = int(math.floor((y - self.bounding_box[0, 1]) / self.cell_size))
        self.check_valid_index((i, j))
        self.check_valid_cell((i, j))
        return i, j

    def position_to_cell(self, position):
        i, j = self.position_to_index(position)
        return self.cells[i][j]
