#ifndef _PEGASUSSOCKET_H
#define _PEGASUSSOCKET_H


#include <deque>
#include "ns3/node.h"
#include "netinet/in.h"
#include "ns3/system-mutex.h"
#include "PegasusConfig.h"

using namespace ns3;

class PegasusPortConfig;
class PegasusPacket;

class PegasusSocket {
  public:
  enum PegasusSocketType {
    PEGASUS_SOCKET_UDP,
    PEGASUS_SOCKET_TCP
  };


  protected:
    //TCP is not implemented.
    PegasusSocketType m_type;

    int m_sd;

    PegasusPortConfig * m_portConfig;

    //The virtual device corrosponding to this socket.
    Ptr<Node> m_node;

    struct sockaddr_in m_peerAddr;

    PegasusSocket();


  public:
    SystemMutex m_rxMutex;

    SystemMutex m_txMutex;

    std::deque<PegasusPacket *> m_packetTxQueue;

    std::deque<PegasusPacket *> m_packetRxQueue;

    virtual ~PegasusSocket();

    inline const PegasusSocketType Get_m_type() const;

    inline const int Get_m_sd() const;

    inline const PegasusPortConfig * Get_m_portConfig() const;

    inline const Ptr<Node> & Get_m_node() const;

    virtual void Create() = 0;

    virtual void Bind() = 0;

    virtual void Send(const char* buffer, const unsigned int & len) = 0;

    void SetAttributes(PegasusPortConfig * portConfig, const Ptr<Node> & node);

};
inline const PegasusSocket::PegasusSocketType PegasusSocket::Get_m_type() const {
  return m_type;
}

inline const int PegasusSocket::Get_m_sd() const {
  return m_sd;
}

inline const PegasusPortConfig * PegasusSocket::Get_m_portConfig() const {
  return m_portConfig;
}

inline const Ptr<Node> & PegasusSocket::Get_m_node() const {
  return m_node;
}

#endif
