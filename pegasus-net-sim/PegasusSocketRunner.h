#ifndef _PEGASUSSOCKETRUNNER_H
#define _PEGASUSSOCKETRUNNER_H


#include "ns3/system-thread.h"

using namespace ns3;

class PegasusVariables;
class PegasusSocket;
class NS3PegasusDroneApp;

class PegasusSocketRunner {
  private:
    static PegasusVariables * m_pegasusVars;

    //systemThread
    Ptr<SystemThread> m_st;

    bool m_running;

    void SendUDPSimulation(const PegasusSocket* pegasusSocket, const char* buffer, const size_t & len);

    void SendTCPSimulation(const PegasusSocket* pegasusSocket, const char* buffer, const size_t & len);


  public:
    PegasusSocketRunner();

    ~PegasusSocketRunner();

    static void Set_m_pegasusVars(PegasusVariables * value);

    void Read();

    void SendSimulation(const PegasusSocket* pegasusSocket, const char* buffer, const size_t & len);

    void Start();

    void Stop();

};
#endif
