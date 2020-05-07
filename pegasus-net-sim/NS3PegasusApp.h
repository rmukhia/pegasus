#ifndef _NS3PEGASUSAPP_H
#define _NS3PEGASUSAPP_H


#include <vector>

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/address.h"
#include "ns3/socket.h"
#include "ns3/traced-callback.h"

using namespace ns3;

class PegasusSocket;

//Inherits from ns3::Application Module.
class NS3PegasusApp: public Application {
  private:
    std::vector<Ptr<Socket>> m_socket;

    TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_rxTraceWithAddresses;

    TracedCallback<Ptr<const Packet> > m_rxTrace;

    PegasusSocket& FindPegasusSocket(const Ptr<Node> & node, const unsigned int & simSockIndex);

    virtual void StartApplication(void );

    virtual void StopApplication(void );


  protected:
    virtual void DoDispose(void );


  public:
    static TypeId GetTypeId(void );

    NS3PegasusApp();

    virtual ~NS3PegasusApp();

    void ScheduleSend();

    void Send();

    void HandleRead();

};
#endif
