import numpy as np
import math
from pegasus_grid_cells import isIntersecting
from pegasus_constraints import CONSTRAINTS

class State(object):
  def __init__(self, prevG, prevVisitedCells, cellContainer):
    self.g = prevG
    self.h = 0
    self.visitedCells = np.copy(prevVisitedCells)
    self.agentCells = {}
    self.movements = {}
    self.movementCounter = {}
    self.parent = None
    self.agentCtr = -1  # start from 0
    self.currentAgentId = None
    self.cellContainer = cellContainer

  def f(self):
    return self.g + self.h

  def setParent(self, parent):
    self.parent = parent

  def constriant1(self, numDirections):
    moved = False
    # At least one agent needs to move
    if 99 in self.movements.values():
      # Initial Condition
      moved = True
    else:
      for movement in self.movements.values():
        if movement < numDirections:
          moved = True
          break 
    if not moved:
      raise Exception('At least one agent needs to move')
      
  def constriant2(self):
    # Check distance between two agents
    maxDistance = CONSTRAINTS['MAX_DISTANCE']
    if len(self.agentCells.values()) == 1:
      return
    for id_l in self.agentCells:
      oneAgentInRange = False
      for id_m in self.agentCells:
        if id_l == id_m:
          continue
        # pythogorims theorem
        x1, y1 = self.agentCells[id_l].position
        x2, y2 = self.agentCells[id_m].position
        xx = x1 - x2
        yy = y1 - y2
        dist = math.sqrt(xx * xx + yy *yy)
        if dist <= maxDistance:
          oneAgentInRange = True
          break
      if not oneAgentInRange:
        raise Exception('Agents cannot leave each others sphere.')
  
  def constriant3(self):
    #check distance of any agent with controlStation
    maxDistance = CONSTRAINTS['MAX_DISTANCE']
    controlStation = CONSTRAINTS['CS_POSITION']
    numAgents = len(self.agentCells.values())

    # At least one agent should be within reach.
    cpos = np.full((numAgents, 2), controlStation[0])
    apos = np.zeros((numAgents, 2))
    
    for i, id_l in enumerate(self.agentCells):
      apos[i] = self.agentCells[id_l].position
      
    dist = np.sqrt(np.sum((cpos-apos) ** 2, axis=1))
    
    if np.min(dist) >= maxDistance:
      raise Exception('At least one agent needs to be in control station range.')


  def constriant4(self, oldState):
    # two agents cannot swap position
    for id_n in self.agentCells:
      for id_m in oldState.agentCells:
        if id_n == id_m:
          continue
        if self.agentCells[id_n] == oldState.agentCells[id_m] and self.agentCells[id_m] == oldState.agentCells[id_n]:
          raise Exception('Agents cannot swap position.')

  def constriant5(self, oldState):
    # two agents should not intersect each others line
    # line equation for paths.
    numAgents = len(self.agentCells.values())
    lineEqs = np.zeros((numAgents, 4))
    for i, id_a in enumerate(self.agentCells):
      lineEqs[i, 0:2] = self.agentCells[id_a].position
      lineEqs[i:,2:4] = oldState.agentCells[id_a].position
      
    for i in range(numAgents):
      eq = lineEqs[i,:]
      otherEqs = np.delete(lineEqs, [i], axis = 0)
      if np.sum(isIntersecting(eq, otherEqs)) > 0:
        raise Exception('Agents cannot have intersecting paths.')
    
  def checkConstraints(self, numDirections, oldState = None):
    self.constriant1(numDirections)
    self.constriant2()
    self.constriant3()
    
    if oldState is not None:
      self.constriant4(oldState)
      self.constriant5(oldState)

  def addAgentAction(self, agentId, movement, newCell, agentCells = None, movementCounter = None):
    if agentCells is not None:
        self.agentCells = agentCells.copy()

    if movementCounter is not None:
        self.movementCounter = movementCounter.copy()

    if newCell in self.agentCells.values():
      raise Exception('Invalid move, two agents cannot occupy the same node')
    self.movements[agentId] = movement
    self.agentCells[agentId] = newCell
    if agentId not in self.movementCounter:
        self.movementCounter[agentId] = 0
    self.movementCounter[agentId] += 1

    self.currentAgentId = agentId

  def setAgentCtr(self, agentCtr):
    self.agentCtr = agentCtr

  def updateVisitedCells(self):
    agentId = self.currentAgentId 
    if self.visitedCells[self.agentCells[agentId].index] == 0:
      self.visitedCells[self.agentCells[agentId].index] = 1
    else:
      self.visitedCells[self.agentCells[agentId].index] *= 2

  def calculateG(self, numDirections):
    agentId = self.currentAgentId
    # stay move is numDirection, so has to be <=
    if 0 <= self.movements[agentId] and self.movements[agentId] <= numDirections:
      if self.visitedCells[self.agentCells[agentId].index] == 0:
        self.g += self.visitedCells[self.agentCells[agentId].index] + 1
      else:
        self.g += self.visitedCells[self.agentCells[agentId].index] * 2

  def calculateH(self, validCells):
    # call this after all add cells
    for cell in self.cellContainer.validCells:
      if self.visitedCells[cell.index] == 0:
        self.h += 1
    return self.h

  def __eq__(self, otherState):
    # if the agents occupy the same position, then the state is the same
    for agentId in self.agentCells:
      if self.agentCells[agentId] != otherState.agentCells[agentId]:
        return False
    return True

  def __str__(self):
    return str(self.agentCells) + ' g: %s, h: %s %s' % (self.g, self.h, str(self.movements))

  def __repr__(self):
    self.__str__()
