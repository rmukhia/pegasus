#ifndef _NS3PEGASUSAPP_H
#define _NS3PEGASUSAPP_H


#include <vector>

#include <ns3/socket.h>
#include <ns3/application.h>

using namespace ns3;

class PegasusSocket;

//Inherits from ns3::Application Module.
class NS3PegasusApp {
  private:
    std::vector<Ptr<Socket>> m_socket;

    PegasusSocket& FindPegasusSocket(const Ptr<Node> & node, const unsigned int & simSockIndex);


  public:
    void ScheduleSend();

    void Send();

    void HandleRead();

};
#endif
