#ifndef _PEGASUSCLIENTSOCKET_H
#define _PEGASUSCLIENTSOCKET_H


#include "PegasusSocket.h"

class PegasusClientSocket : public PegasusSocket {
  public:
    PegasusClientSocket();

    ~PegasusClientSocket();

    virtual void Create();

    virtual void Send();

};
#endif
