import numpy as np
import math


def _getMinMaxVertices(polygon):
  (xMax, yMax) = np.max(polygon, axis=0)
  (xMin, yMin) = np.min(polygon, axis = 0)
  return (xMin, yMin, xMax, yMax) 

def _getBoundingBoxVertices(polygon):
  (xMin, yMin, xMax, yMax) = _getMinMaxVertices(polygon)
  return np.array(((xMin, yMin), (xMax, yMin), (xMax, yMax), (xMin, yMax)))

def _getGridRange(xMin, yMin, xMax, yMax, cellSize):
  iMax = math.ceil((xMax - xMin)/cellSize)
  jMax = math.ceil((yMax - yMin)/cellSize)
  return (int(iMax), int(jMax))

def _getGridCells(iMax, jMax, xMin, yMin, cellSize):
  totalGrids = iMax * jMax
  grids =  np.zeros((totalGrids, 2))
  ctr = 0
  for j in range (jMax):
    for i in range(iMax):
      grids[ctr, 0] = xMin + i * cellSize
      grids[ctr, 1] = yMin + j * cellSize
      ctr += 1
  return grids

def _getGridCellIndex(iMax, jMax, k):
  j = math.floor(k/iMax)
  i = k - j * iMax
  return (int(i), int(j))

def _getLineSegments(polygon):
  n = np.shape(polygon)[0]
  lines = np.zeros((n, 4))
  for s in range(n):
    if s != (n - 1):
        lines[s, 0] = polygon[s, 0]
        lines[s, 1] = polygon[s, 1]
        lines[s, 2] = polygon[s + 1, 0]
        lines[s, 3] = polygon[s + 1, 1]
    else:
        lines[s, 0] = polygon[s, 0]
        lines[s, 1] = polygon[s, 1]
        lines[s, 2] = polygon[0, 0]
        lines[s, 3] = polygon[0, 1]

  return lines


def _getGridRays(cellsCenter, xMax):
  k = np.shape(cellsCenter)[0]
  gridRays = np.zeros((k, 4))
  gridRays[:, 0:2] = cellsCenter
  gridRays[:, 2] = xMax + 1
  gridRays[:, 3] = cellsCenter[:,1]
  return gridRays


def isIntersecting(lineSegment, otherLineSegments):
  n = np.shape(otherLineSegments)[0]
  lineSegments = np.full((n, 4), lineSegment)

  f = lineSegments[:, 2:4] - lineSegments[:, 0:2]
  g = otherLineSegments[:, 2:4] - otherLineSegments[:, 0:2]
  qmp = otherLineSegments[:, 0:2] - lineSegments[:, 0:2]

  fxg = np.cross(f, g)
  qpxf = np.cross(qmp, f)

  mantessa = 1e-10

  # collinear but disjoint
  isCollinear = np.logical_and(
          np.abs(fxg) <= mantessa,
          np.abs(qpxf) <= mantessa
          )

  f = f[np.logical_not(isCollinear)]
  g = g[np.logical_not(isCollinear)]
  qmp = qmp[np.logical_not(isCollinear)]
  fxg = fxg[np.logical_not(isCollinear)]
  qpxf = qpxf[np.logical_not(isCollinear)]

  # parallel and non intersecting
  isParallel = np.logical_and(
          np.abs(fxg) <= mantessa,
          np.abs(qpxf) > mantessa
          )

  f = f[np.logical_not(isParallel)]
  g = g[np.logical_not(isParallel)]
  qmp = qmp[np.logical_not(isParallel)]
  fxg = fxg[np.logical_not(isParallel)]
  qpxf = qpxf[np.logical_not(isParallel)]


  t = np.cross(qmp, g) / fxg
  u = np.cross(qmp, f) / fxg

  return np.logical_and(
          np.abs(fxg) >= mantessa,
          np.logical_and(
              np.logical_and(0 <= t, t <= 1),
              np.logical_and(0 <= u, u <= 1)
              )
          )

def _checkLieInside(ray, polygonLineSegments):
  numIntersections = np.sum(isIntersecting(ray, polygonLineSegments))

  if numIntersections == 0 or numIntersections % 2 == 0:
    return False
  else:
    return True

def _getInsideGridIndex(rays, polygonLineSegments):
  k = np.shape(rays)[0]
  flag = np.zeros((k), dtype=bool)
  for i in range(k):
    flag[i] = _checkLieInside(rays[i], polygonLineSegments)
  return flag

class GridCells(object):
  def __init__(self):
    pass

def getGridCells(polygon, c):
    # namespace to hold grid related methods and variables
  gridCells = GridCells()

  minMax = _getMinMaxVertices(polygon)
  gridCells.minMax = _getGridRange(minMax[0], minMax[1], minMax[2], minMax[3], c)

  gridCells.boundingBox = _getBoundingBoxVertices(polygon)

  gridCells.cells = _getGridCells(gridCells.minMax[0], gridCells.minMax[1],
      minMax[0], minMax[1],  c)

  gridCells.cellsCenter = gridCells.cells + c/2 

  gridCells.getIndex = _getGridCellIndex

  gridCells.valid = _getInsideGridIndex(
          _getGridRays(gridCells.cellsCenter, _getMinMaxVertices(polygon)[2]),
          _getLineSegments(polygon))

  return gridCells
