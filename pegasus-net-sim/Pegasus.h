#ifndef _PEGASUS_H
#define _PEGASUS_H


#include "PegasusVariables.h"
#include "NS3Runner.h"
#include "GazeboNode.h"
#include "PegasusSocketRunner.h"
#include "PegasusConfig.h"


using namespace ns3;

class NS3PegasusDroneApp;
class PegasusUDPSocket;

class Pegasus {
  private:
    static PegasusVariables m_pegasusVars;

    static Pegasus * sm_instance;

    Pegasus();

    void SetupProxy();


  public:
    NS3Runner m_ns3Runner;

    GazeboNode m_gazeboNode;

    PegasusSocketRunner m_pegasusSocketRunner;

    ~Pegasus();

    static Pegasus* GetInstance();

    static void ChangePosition();

    void Run(int argc, char** argv);

};
#endif