#ifndef _PEGASUSSOCKET_H
#define _PEGASUSSOCKET_H


#include "ns3/node.h"
#include "netinet/in.h"

using namespace ns3;

class PegasusSocket {
  protected:
    int m_sd;

    //Port this socket is bound to.
    int m_port;

    //Port this socket will send to.
    int m_peerPort;

    //The virtual device corrosponding to this socket.
    Ptr<Node> m_node;

    struct sockaddr_in m_peerAddr;

    PegasusSocket();


  public:
    virtual ~PegasusSocket();

    virtual void Create() = 0;

    virtual void Bind() = 0;

    virtual void Send(const char* buffer, const unsigned int & len) = 0;

    void SetAttributes(int port, int peerPort, const Ptr<Node> & node);

    inline const int Get_m_sd() const;

    inline const int Get_m_port() const;

    inline const int Get_m_peerPort() const;

    inline const Ptr<Node> & Get_m_node() const;

};
inline const int PegasusSocket::Get_m_sd() const {
  return m_sd;
}

inline const int PegasusSocket::Get_m_port() const {
  return m_port;
}

inline const int PegasusSocket::Get_m_peerPort() const {
  return m_peerPort;
}

inline const Ptr<Node> & PegasusSocket::Get_m_node() const {
  return m_node;
}

#endif
