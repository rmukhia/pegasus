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
class NS3PegasusControlStationApp;
class PegasusSocket;

class PegasusVariables {
  public:
    std::unordered_map<std::string, Vector> m_poseMap;

    SystemMutex m_poseMapMutex;

    std::vector<std::string> m_modelsName;

    //All nodes
    NodeContainer m_nodes;

    NodeContainer m_controlStationNode;

    NodeContainer m_droneNodes;

    std::vector<Ptr<NS3PegasusDroneApp>> m_ns3PegasusDroneApps;

    Ptr<NS3PegasusControlStationApp> m_ns3PegasusControlStationApp;

    std::vector<PegasusSocket *> m_pegasusSockets;

};
#endif
