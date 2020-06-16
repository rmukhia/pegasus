import math
from pegasus_cell import Cell

class CellContainer(object):
  MOVE = {
      'RIGHT'     : 0,
      'RIGHT-UP'  : 1,
      'UP'        : 2,
      'LEFT-UP'   : 3,
      'LEFT'      : 4,
      'LEFT-DOWN' : 5,
      'DOWN'      : 6,
      'RIGHT-DOWN': 7,
      'STAY'      : 8,
  }

  def __init__(self, boundingBox, gridCells, cellSize, numDirections = 4, agentsHoverHeight = 10):
    self.iMax, self.jMax = gridCells.minMax
    self.boundingBox = boundingBox
    self.gridCells = gridCells
    self.cellSize = cellSize
    self.validCells = None
    self.agentsHoverHeight = agentsHoverHeight
    self.fillCells()
    self.NUM_DIRECTIONS = numDirections
    
  def fillCells(self):
    self.cells = [None] * self.iMax;
    self.validCells = []
    for i in range(self.iMax):
      self.cells[i]  = [None] * self.jMax
    for k in range(self.iMax * self.jMax):
      ith, jth = self.gridCells.getIndex(self.iMax, self.jMax, k)
      self.cells[ith][jth] = Cell((ith, jth), 
                                  self.gridCells.cellsCenter[k],
                                  self.gridCells.valid[k])
      if self.gridCells.valid[k]:
        self.validCells.append(self.cells[ith][jth])
    

  def checkValidCell(self, index):
    valid = False
    for cell in self.validCells:
      if cell.index[0] == index[0] and cell.index[1] == index[1]:
        valid = True
    if not valid:
      raise Exception('Invalid grid range %s.' % str(index))

  def checkValidIndex(self, index):
    if (self.iMax <= index[0] or index[0] < 0
        or self.jMax <= index[1] or index[1] < 0):
      raise Exception('Out of bounds grid range %s.' % str(index))
      
  def move(self, direction, currentIndex):
    i, j = currentIndex
    newIndexList = [
                (i + 1, j    ),   # RIGHT
                (i + 1, j + 1),   # RIGHT-UP
                (i    , j + 1),   # UP
                (i - 1, j + 1),   # LEFT-UP
                (i - 1, j    ),   # LEFT
                (i - 1, j - 1),   # LEFT-DOWN
                (i    , j - 1),   # DOWN
                (i + 1, j - 1),   # RIGHT-DOWN
                (i    , j    ),   # STAY
    ]

    newIndex = newIndexList[direction * 2 if self.NUM_DIRECTIONS == 4 else direction]
    self.checkValidIndex(newIndex)
    self.checkValidCell(newIndex)
    return newIndex

  def moveAndGetCell(self, direction, currentCell = None, currentIndex = None):
    if currentCell is None and currentIndex is None:
      raise Exception('Need either cell or currentIndex')

    if currentCell is not None:
      currentIndex = currentCell.index
    i , j = self.move(direction, currentIndex)
    return self.cells[i][j]

  def positionToIndex(self, position):
    x, y = position
    i = int(math.floor((x - self.boundingBox[0, 0]) / self.cellSize)) 
    j = int(math.floor((y - self.boundingBox[0, 1]) / self.cellSize))
    self.checkValidIndex((i, j))
    self.checkValidCell((i, j))
    return (i, j)

  def positionToCell(self, position):
    i, j = self.positionToIndex(position)
    return self.cells[i][j]
