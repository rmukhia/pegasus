#ifndef _PEGASUS_H
#define _PEGASUS_H


#include "PegasusVariables.h"
#include <vector>
#include "PegasusUDPSocket.h"
#include "NS3Runner.h"
#include "GazeboNode.h"
#include "PegasusSocketRunner.h"
#include "PegasusConfig.h"


using namespace ns3;

class PegasusTrace;
class NS3PegasusDroneApp;

class Pegasus {
  private:
    static PegasusVariables m_pegasusVars;

    bool m_running;

    Ptr<SystemThread> m_stStatus;
    static Pegasus * sm_instance;

    std::vector<PegasusTrace> m_pegasusTraces;

    PegasusUDPSocket m_statusPegasusSocket;

    Pegasus();

    void SetupProxy();

    void SetupTrace();

    void RunStatusThread();


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