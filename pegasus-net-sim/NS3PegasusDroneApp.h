#ifndef _NS3PEGASUSDRONEAPP_H
#define _NS3PEGASUSDRONEAPP_H


#include "ns3/application.h"

#include <map>

#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/address.h"
#include "ns3/socket.h"
#include "ns3/traced-callback.h"

using namespace ns3;

class PegasusVariables;
class PegasusSocket;

//Inherits from ns3::Application Module.
class NS3PegasusDroneApp : public Application{
  private:
    static PegasusVariables * m_pegasusVars;

    TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_rxTraceWithAddresses;

    TracedCallback<Ptr<const Packet> > m_rxTrace;

    virtual void StartApplication(void );

    virtual void StopApplication(void );

    Address GetAddressFromRealDstPort(int realDstPort);

    PegasusSocket* FindPegasusSocket(int port);


  protected:
    virtual void DoInitialize();

    virtual void DoDispose(void );


  public:
    //Real port corrosponding to this node. 
    std::map<int, Ptr<Socket>> m_realDstPortMapVirtualSocket;

    static TypeId GetTypeId(void );

    NS3PegasusDroneApp();

    virtual ~NS3PegasusDroneApp();

    static void Set_m_pegasusVars(PegasusVariables * value);

    Ptr<Socket> CreateVirtualSocket(int virtPort, const Ptr<NS3PegasusDroneApp> & app);

    void ScheduleSend();

    void Send(int realDstPort, int peerRealDstPort, const char * buffer, const unsigned int len);

    void HandleRead(Ptr<Socket> socket);

};
#endif
