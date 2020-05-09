#ifndef _PEGASUSVARIABLES_H
#define _PEGASUSVARIABLES_H


#include <unordered_map>
#include <string>
#include <vector>

#include "ns3/system-mutex.h"
#include "ns3/vector.h"
#include "ns3/node-container.h"

using namespace ns3;

class NS3PegasusDroneApp;
class PegasusSocket;

class PegasusVariables {
  public:
    //Store the main simulator context to make ns3 simulation thread safe.
    uint32_t m_simulatorContext;

    std::unordered_map<std::string, Vector> m_poseMap;

    SystemMutex m_poseMapMutex;

    std::vector<std::string> m_modelsName;

    //All nodes
    NodeContainer m_nodes;

    NodeContainer m_controlStationNode;

    NodeContainer m_droneNodes;

    std::vector<Ptr<NS3PegasusDroneApp>> m_ns3PegasusDroneApps;

    std::vector<PegasusSocket *> m_pegasusSockets;

};
#endif
