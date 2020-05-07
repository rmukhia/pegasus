#ifndef _PEGASUSSERVERSOCKET_H
#define _PEGASUSSERVERSOCKET_H


#include "PegasusSocket.h"

class PegasusServerSocket : public PegasusSocket {
  public:
    PegasusServerSocket();

    ~PegasusServerSocket();

    virtual void Bind();

    virtual void Create();

    virtual void Send();

};
#endif
