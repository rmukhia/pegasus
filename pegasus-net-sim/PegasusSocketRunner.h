#ifndef _PEGASUSSOCKETRUNNER_H
#define _PEGASUSSOCKETRUNNER_H


#include "ns3/system-thread.h"

using namespace ns3;

class PegasusSocket;

class PegasusSocketRunner {
  private:
    //systemThread
    Ptr<SystemThread> m_st;

    void SendUDPSimulation(const PegasusSocket & pegasusSocket, const char * & buffer, const size_t & len);

    void SendTCPSimulation(const PegasusSocket & pegasusSocket, const char * & buffer, const size_t & len);


  public:
    PegasusSocketRunner();

    ~PegasusSocketRunner();

    void Read();

    void SendSimulation(const PegasusSocket & pegasusSocket, char buffer, const size_t & len);

};
#endif
