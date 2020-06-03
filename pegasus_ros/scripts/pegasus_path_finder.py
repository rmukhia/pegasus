import math
import time
import random
import copy
import numpy as np
import rospy
from pegasus_state import State 

class PathFinder(object):
  def __init__(self, agents, cellContainer):
    self.agents = agents
    self.cellContainer = cellContainer

  def getMovement(self, conf):
    pass

  def getInitialState(self):
    visitedCells = np.zeros((self.cellContainer.iMax, self.cellContainer.jMax), dtype=float)
    state = State(0, visitedCells, self.cellContainer)

    for agent in self.agents:
      cell = self.cellContainer.positionToCell(agent.initialPosition)
      state.addAgentAction(agent.id, 99, cell)
      state.checkConstraints(self.cellContainer.NUM_DIRECTIONS)
      state.updateVisitedCells()
    state.calculateG(self.cellContainer.NUM_DIRECTIONS)
    state.calculateH(self.cellContainer.validCells)
    return state

  def getNeighbour(self, state, movement):
    agentCtr = self.getNextAgent(state.agentCtr)
    agent = self.agents[agentCtr]
    newState = State(state.g, state.visitedCells, self.cellContainer)
    cell = self.cellContainer.moveAndGetCell(movement, state.agentCells[agent.id])
    newState.addAgentAction(agent.id, movement, cell, state.agentCells, state.movementCounter)
    newState.checkConstraints(self.cellContainer.NUM_DIRECTIONS, state)
    newState.updateVisitedCells()
    newState.calculateG(self.cellContainer.NUM_DIRECTIONS)
    newState.calculateH(self.cellContainer.validCells)
    newState.setAgentCtr(agentCtr)
    return newState

  def getNextAgent(self, agentCtr):
    return (agentCtr + 1) % len(self.agents)

  def printParents(self, state):
    current = state
    print ('--------child start------------')
    while current is not None:
      print (current)
      current = current.parent
    print ('--------parent end------------')

  def getDepth(self, state):
    i = 0;
    current = state
    while current is not None:
      i += 1
      current = current.parent
    return i

  def getBaseParent(self, state):
    current = state
    while current.parent is not None:
      current = current.parent
    return current

  def concatinateGoals(self, goals):
    goalLast = None
    for g in goals:
      base = self.getBaseParent(g)
      if (goalLast is not None and goalLast.parent is not None):
        base.setParent(goalLast.parent)
      goalLast = g
    return goalLast
  
  def trimGoals(self, goal):
    current = goal
    h = current.h
    while current.parent is not None:
      if current.parent.h > h:
        return current
      current = current.parent
    return goa

  def search(self, depthExit = 0, epochStop = 0, previousGoal = None):
    numConf = self.cellContainer.NUM_DIRECTIONS
    openl = []
    closedl = []

    earlyExit = None

    initialState = None

    if (previousGoal):
      previousGoal.setParent(None)
      initialState = previousGoal
    else:
      try:
        initialState = self.getInitialState()
      except Exception as e:
        # print (e)
        return ('not-found', None)

    openl.append(initialState)

    if epochStop != 0:
      # If heuristics does not increase for certain epochs..then stop
      earlyExit = {
          'h': len(self.cellContainer.validCells),
          'epochs': 0,
          'minCostState': initialState
        }

    while True:
      if (len(openl) == 0):
        return ('not-found', None)

      # current - lowest f() in open list
      current = openl[0]
      for state in openl:
        if current.f() > state.f():
          current = state

      openl.remove(current)
      
      if epochStop != 0:
        if current.h < earlyExit['h']:
          earlyExit['h'] = current.h
          earlyExit['epochs'] = 0
          earlyExit['minCostState'] = current
        else:
          earlyExit['epochs'] += 1
          if earlyExit['minCostState'].f() > current.f():
            earlyExit['minCostState'] = current

        # early exit
        if earlyExit['epochs'] > epochStop:
          return ('early-exit', earlyExit['minCostState'])


      if (depthExit != 0 and self.getDepth(current) > depthExit):
        return ('depth-exit', current.parent)

      if (current.h == 0):
        return ('found', current)

      confgs = [x for x in range(numConf)]

      while len(confgs) > 0:
        successor = None
        successorCost = current.g + 1
        # print (successorCost)
        try:
          cnf = confgs.pop(random.randrange(len(confgs)))
          successor = self.getNeighbour(current, cnf)
        except Exception as e:
          # print (e, current.h)
          continue
        if successor in openl:
          if successor.g <= successorCost:
            continue
        elif successor in closedl:
          if successor.g <= successorCost:
            continue
          openl.append(successor)
          closedl.remove(successor)
        else:
          openl.append(successor)
        successor.g = successorCost
        successor.setParent(current)
      
      closedl.append(current)


  def find(self, depthExit, sigmaT):
    startTime = time.time()
    goal = None
    goals = []
    prev_h = len(self.cellContainer.validCells)
    sigma = 0
    while True:
      ret, goal = self.search(epochStop=2048, depthExit=depthExit, previousGoal=copy.deepcopy(goal))
      rospy.loginfo (ret)
      goals.append(goal)
      h = goal.h
      rospy.loginfo ('h= %s' %(h,)) 
      if (prev_h <= h):
        # Does not converge
        sigma += 1
        if sigma >= sigmaT:
          break
      elif h ==0:
        # converged
        break
      else:
        sigma = 0
      prev_h = h
    rospy.loginfo ("Total Time Taken: %s seconds." % (time.time() - startTime, ))
    finalGoal = self.concatinateGoals(goals)
    finalGoal = self.trimGoals(finalGoal)
    rospy.loginfo (finalGoal.visitedCells.T)
    rospy.loginfo ("Steps : %s " % (self.getDepth(finalGoal)))
    return finalGoal
  
  def getMovementPlanFromGoal(self, finalGoal):
    numAgents = len(finalGoal.agentCells.values())

    agentpose = []
    for i in range(numAgents):
      agentpose.append({ 'x': [], 'y': [], 'steps': [] })

    cur = finalGoal
    while cur is not None:
      for i, agentId in enumerate(cur.agentCells): 
        # truncate all steps which are redundant
        if (len(agentpose[i]['steps']) > 0 and cur.movementCounter[agentId] >= agentpose[i]['steps'][-1]):
            continue
        agentpose[i]['x'].append(cur.agentCells[agentId].position[0])
        agentpose[i]['y'].append(cur.agentCells[agentId].position[1])
        agentpose[i]['steps'].append(cur.movementCounter[agentId])
      cur = cur.parent
    for i in range(numAgents):
      agentpose[i]['x'].reverse()
      agentpose[i]['y'].reverse()

    return agentpose
