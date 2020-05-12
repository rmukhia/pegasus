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
class PegasusPacket;

//Inherits from ns3::Application Module.
class NS3PegasusDroneApp : public Application{
  private:
    static PegasusVariables * m_pegasusVars;

    TracedCallback<Ptr<const Packet>, const Address &, const Address &> m_rxTraceWithAddresses;

    TracedCallback<Ptr<const Packet> > m_rxTrace;

    bool m_running;

    virtual void StartApplication(void );

    virtual void StopApplication(void );

    Address GetAddressFromVirtualPort(int port);

    PegasusSocket* FindPegasusSocket(int port);

    void IntoTheMatrix();


  protected:
    virtual void DoInitialize();

    virtual void DoDispose(void );


  public:
    //Real and virutal port corrosponding to this node.  A node shares the same real and ns3 port number.
    std::map<int, Ptr<Socket>> m_portMapVirtualSocket;

    static TypeId GetTypeId(void );

    NS3PegasusDroneApp();

    virtual ~NS3PegasusDroneApp();

    static void Set_m_pegasusVars(PegasusVariables * value);

    Ptr<Socket> CreateVirtualSocket(int virtPort);

    void ScheduleSend(int port, int peerPort, const char * buffer, const unsigned int & len);

    void Send(const Ptr<Socket> & socket, const Ptr<Packet> & packet, int flags, const InetSocketAddress & addr);

    void HandleRead(Ptr<Socket> socket);

};
#endif
