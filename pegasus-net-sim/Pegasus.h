#ifndef _PEGASUS_H
#define _PEGASUS_H


#include "PegasusVariables.h"
#include "NS3Runner.h"
#include "GazeboNode.h"
#include "PegasusSocketRunner.h"
#include <vector>
#include <string>


using namespace ns3;

class PegasusSocket;
class NS3PegasusDroneApp;

class Pegasus {
  private:
    static PegasusVariables m_pegasusVars;

    static Pegasus * sm_instance;

    Pegasus();


  public:
    NS3Runner m_ns3Runner;

    GazeboNode m_gazeboNode;

    PegasusSocketRunner m_pegasusSocketRunner;

    std::vector<PegasusSocket> m_pegasusSockets;

    ~Pegasus();

    static Pegasus* GetInstance();

    static void ChangeDronesPosition();

    void Run(int argc, char** argv, const std::vector<std::string> & droneNames);

};
#endif