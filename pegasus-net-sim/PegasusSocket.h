#ifndef _PEGASUSSOCKET_H
#define _PEGASUSSOCKET_H


#include <deque>
#include "ns3/node.h"
#include "netinet/in.h"
#include "ns3/system-mutex.h"

using namespace ns3;

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

    //Real port this socket is bound to. NS3 also shares the same port number.
    int m_port;

    //Real port this socket will send to.
    int m_peerPort;

    //NS3 port that this socket will send to.
    int m_virtualPeerPort;

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

    inline const int Get_m_port() const;

    inline const int Get_m_peerPort() const;

    inline const int Get_m_virtualPeerPort() const;

    inline const Ptr<Node> & Get_m_node() const;

    virtual void Create() = 0;

    virtual void Bind() = 0;

    virtual void Send(const char* buffer, const unsigned int & len) = 0;

    void SetAttributes(int port, int peerPort, int virtualPeerPort, const Ptr<Node> & node);

};
inline const PegasusSocket::PegasusSocketType PegasusSocket::Get_m_type() const {
  return m_type;
}

inline const int PegasusSocket::Get_m_sd() const {
  return m_sd;
}

inline const int PegasusSocket::Get_m_port() const {
  return m_port;
}

inline const int PegasusSocket::Get_m_peerPort() const {
  return m_peerPort;
}

inline const int PegasusSocket::Get_m_virtualPeerPort() const {
  return m_virtualPeerPort;
}

inline const Ptr<Node> & PegasusSocket::Get_m_node() const {
  return m_node;
}

#endif
