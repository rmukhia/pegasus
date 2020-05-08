
#include "NS3PegasusDroneApp.h"
#include "PegasusVariables.h"
#include "PegasusSocket.h"

#include "ns3/ipv4.h"

# include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNS3DroneApp");
NS_OBJECT_ENSURE_REGISTERED (NS3PegasusDroneApp);

PegasusVariables * NS3PegasusDroneApp::m_pegasusVars;

void NS3PegasusDroneApp::StartApplication(void ) {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusDroneApp::StopApplication(void ) {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusDroneApp::DoInitialize() {
  NS_LOG_FUNCTION(this);
  Application::DoInitialize();
}

void NS3PegasusDroneApp::DoDispose(void ) {
  NS_LOG_FUNCTION(this);
  Application::DoDispose();
}

TypeId NS3PegasusDroneApp::GetTypeId(void )
{
   static TypeId tid = TypeId ("NS3PegasusDroneApp")
     .SetParent<Application> ()
     .SetGroupName("NS3PegasusDroneApp")
     .AddConstructor<NS3PegasusDroneApp> ()
     .AddTraceSource ("Rx", "A packet has been received",
                      MakeTraceSourceAccessor (&NS3PegasusDroneApp::m_rxTrace),
                      "ns3::Packet::TracedCallback")
     .AddTraceSource ("RxWithAddresses", "A packet has been received",
                      MakeTraceSourceAccessor (&NS3PegasusDroneApp::m_rxTraceWithAddresses),
                      "ns3::Packet::TwoAddressTracedCallback");
   return tid;
}

NS3PegasusDroneApp::NS3PegasusDroneApp() {
  NS_LOG_FUNCTION(this);
}

NS3PegasusDroneApp::~NS3PegasusDroneApp() {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusDroneApp::Set_m_pegasusVars(PegasusVariables * value)
{
  m_pegasusVars = value;
}

Ptr<Socket> NS3PegasusDroneApp::CreateVirtualSocket(int virtPort, const Ptr<NS3PegasusDroneApp> &app) {
  NS_LOG_FUNCTION(this);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> socket = Socket::CreateSocket (app->GetNode (), tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (),
      virtPort);
  if (socket->Bind (local) == -1)
    NS_FATAL_ERROR ("Failed to bind socket");

  socket->SetRecvCallback (MakeCallback (&NS3PegasusDroneApp::HandleRead, app));

  return socket;
}

void NS3PegasusDroneApp::ScheduleSend() {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusDroneApp::Send(int realDstPort, const Ptr<Packet> & packet, int peerRealDstPort) {
  NS_LOG_FUNCTION(this);
  Ptr<Socket> socket = m_realDstPortMapVirtualSocket[realDstPort];
  Ptr<Node> node = NULL;

  for (unsigned int i = 0; i < m_pegasusVars->m_ns3PegasusDroneApps.size(); i++) {
    Ptr<NS3PegasusDroneApp> app = m_pegasusVars->m_ns3PegasusDroneApps[i];
    if (app->m_realDstPortMapVirtualSocket.find(peerRealDstPort) !=
        app->m_realDstPortMapVirtualSocket.end()) {
      node = app->GetNode();
      break;
    }
  }

  if (!node) 
    NS_FATAL_ERROR ("Failed to find destination node address.");

  NS_LOG_INFO(node);
  Ptr<NetDevice> dev = node->GetDevice(0);
  NS_LOG_INFO(dev << dev->GetInstanceTypeId().GetName());
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();

  int ipv4_idx = ipv4->GetInterfaceForDevice(dev);
  NS_LOG_INFO(ipv4->GetAddress(ipv4_idx, 0).GetLocal());

  Address address = dev->GetAddress();

  NS_LOG_INFO(address);






}

void NS3PegasusDroneApp::HandleRead(Ptr<Socket> socket) {
  NS_LOG_FUNCTION(this);
}

