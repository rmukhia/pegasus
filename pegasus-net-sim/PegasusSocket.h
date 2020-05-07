#ifndef _PEGASUSSOCKET_H
#define _PEGASUSSOCKET_H


#include <ns3/node.h>

using namespace ns3;

class PegasusSocket {
  private:
    int m_sd;

    int m_port;

    Ptr<Node> m_simNode;

    unsigned int m_simSockIndex;

    Ptr<Node> m_simDstNode;

    Ptr<Node> m_simDstPort;

    int m_peerPort;


  public:
    PegasusSocket();

    ~PegasusSocket();

    virtual void Send() = 0;

    virtual void Create() = 0;

};
#endif
