#ifndef _NS3PEGASUSCONTROLSTATIONAPP_H
#define _NS3PEGASUSCONTROLSTATIONAPP_H


#include "ns3/application.h"

#include <vector>

#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/address.h"
#include "ns3/socket.h"
#include "ns3/traced-callback.h"

using namespace ns3;

//Inherits from ns3::Application Module.
class NS3PegasusControlStationApp : public Application{
  private:
    std::vector<Ptr<Socket>> m_socket;

    TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_rxTraceWithAddresses;

    TracedCallback<Ptr<const Packet> > m_rxTrace;

    virtual void StartApplication(void );

    virtual void StopApplication(void );


  protected:
    virtual void DoInitialize();

    virtual void DoDispose(void );


  public:
    static TypeId GetTypeId(void );

    NS3PegasusControlStationApp();

    virtual ~NS3PegasusControlStationApp();

    void ScheduleSend();

    void Send();

    void HandleRead();

};
#endif
