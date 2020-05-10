#ifndef _PEGASUSSOCKETRUNNER_H
#define _PEGASUSSOCKETRUNNER_H


#include "ns3/system-thread.h"

using namespace ns3;

class PegasusVariables;
class PegasusSocket;
class PegasusPacket;
class NS3PegasusDroneApp;
class PegasusConfig;

class PegasusSocketRunner {
  private:
    static PegasusVariables * m_pegasusVars;

    //systemThread
    Ptr<SystemThread> m_st;

    bool m_running;

    void handleRead(int maxFd);


  public:
    void handleWrite(int maxFd);


  private:
    void SendSimulation(PegasusSocket* pegasusSocket, const char* buffer, const size_t & len);


  public:
    static void Set_m_pegasusVars(PegasusVariables * value);

    PegasusSocketRunner();

    ~PegasusSocketRunner();

    void ExecutionLoop();

    void Start();

    void Stop();

};
#endif
