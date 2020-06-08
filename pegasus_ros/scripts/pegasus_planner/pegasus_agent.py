class Agent(object):
  def __init__(self, id, initialPosition):
    self.initialPosition = initialPosition
    self.id = id

'''
  def getInitialState(self):
    cell = self.cellContainer.positionToCell(self.initialPosition)
    agentState = AgentState(agent)
    agentState.addAction(99, cell)
    return agentState

  def initializeSearch(self, depthExit =0, epochStop = 0, previousGoal = None):
    self.numConf = self.cellContainer.NUM_DIRECTIONS
    self.openlist = []
    self.closedlist = []

    self.earlyExit = None

    initialState = None

    if (previousGoal):
      previousGoal.setParent(None)
      initialState = previousGoal
    else:
      # This will throw an exception if the state is invalid
      initialState = self.getInitialState(agent)

    self.openList.append(initialState)

    if epochStop != 0:
      self.earlyExit = {
          'h': len(self.cellContainer.validCells),
          'epochs': 0,
          'minCostState': initialState
          }

  def search(self):
    if (len(self.openlist) == 0):
      return ('not-found', None)

    current = self.openlist[0]
    for state in openlist:
      if current.f() > state.f():
        current = state

    self.openlist.remove(current)

    if (current.h == 0):
      return ('found', current)

    confgs = [x for x in range(numConf)]

    for conf in range(numConf):
      successor = None
      successorCost = current.g + 1
      try:
        cnf = confgs.pop(random.randrange(len(confgs)))
        successor = self.getNeighbour(current, cnf)
      except Exception as e:
        # print (e)
        continue
      
      if successor in self.openlist:
        if successor.g <= successorCost:
          continue
      elif successor in self.closedlist:
        if sucessor.g <= succesorCost:
          continue
        self.openlist.append(successor)
        self.closedl.remove(successor)
      else:
        self.openlist.append(successor)
      successor.g = successorCost
      successor.setParent(current)

    self.closedlist.append(current)

    return ('next', None)
'''
