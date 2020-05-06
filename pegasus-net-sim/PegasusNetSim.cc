#include "PegasusNetSim.h"

PegasusNetSim * PegasusNetSim::instance = NULL;

PegasusNetSim *PegasusNetSim::Instance()
{
  if (instance == NULL) {
    instance = new PegasusNetSim();
    instance->gazeboNode = new GazeboNode();
    instance->ns3Runner = new NS3Runner();
  }

  return instance;
}


PegasusNetSim::~PegasusNetSim()
{
  delete instance->gazeboNode;
  delete instance->ns3Runner;
}
