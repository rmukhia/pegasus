#ifndef _PEGASUSVARIABLES_H
#define _PEGASUSVARIABLES_H


#include <unordered_map>
#include <string>
#include <vector>

#include "ns3/system-mutex.h"
#include "ns3/vector.h"

using namespace ns3;

class PegasusVariables {
  public:
    std::unordered_map<std::string, Vector> m_poseMap;

    SystemMutex m_poseMapMutex;

    std::vector<std::string> m_modelsName;

};
#endif
