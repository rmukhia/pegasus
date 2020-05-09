#ifndef _PEGASUSUDPSOCKET_H
#define _PEGASUSUDPSOCKET_H


#include "PegasusSocket.h"

using namespace ns3;

class PegasusUDPSocket : public PegasusSocket {
  public:
    PegasusUDPSocket();

    ~PegasusUDPSocket();

    void Create();

    void Bind();

    virtual void Send(const char* buffer, const unsigned int & len);

};
#endif
