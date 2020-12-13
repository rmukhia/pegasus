import numpy as np


class Cell(object):
    """
    state.data: [0:2] index, [2:4] position, [4] valid, index is (y, x)
    """
    def __init__(self, index, position, valid):
        self.index = index
        self.position = position
        self.valid = valid
        self.data = np.concatenate((index[::-1], position, valid), axis=None)

    def get_np_index(self):
        # y, x
        return int(self.data[0]), int(self.data[1])

    def __str__(self):
        return 'Index %s: Position %s: Valid: %s ' % (self.index, self.position, self.valid)

    def __repr__(self):
        return self.__str__()
