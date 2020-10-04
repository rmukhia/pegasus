class Cell(object):
    def __init__(self, index, position, valid):
        self.index = index
        self.position = position
        self.valid = valid

    def __str__(self):
        return 'Index %s: Position %s: Valid: %s ' % (self.index, self.position, self.valid)

    def __repr__(self):
        return self.__str__()
