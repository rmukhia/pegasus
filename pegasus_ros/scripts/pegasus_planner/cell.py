import numpy as np


class Cell(object):
    """
    state.data: [0:2] index, [2:4] position, [4] valid
    """
    def __init__(self, index, position, valid):
        self.index = index
        self.position = position
        self.valid = valid
        self.data = np.concatenate((index, position, valid), axis=None)

    def __str__(self):
        return 'Index %s: Position %s: Valid: %s ' % (self.index, self.position, self.valid)

    def __repr__(self):
        return self.__str__()
