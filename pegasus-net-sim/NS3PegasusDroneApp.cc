
#include "NS3PegasusDroneApp.h"
#include "PegasusVariables.h"
#include "PegasusSocket.h"

#include <algorithm>
#include "ns3/log.h"
#include "ns3/ipv4.h"
#include "ns3/names.h"
#include "ns3/simulator.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNS3DroneApp");
NS_OBJECT_ENSURE_REGISTERED (NS3PegasusDroneApp);

PegasusVariables * NS3PegasusDroneApp::m_pegasusVars;

void NS3PegasusDroneApp::StartApplication(void ) {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusDroneApp::StopApplication(void ) {
  NS_LOG_FUNCTION(this);
}

Address NS3PegasusDroneApp::GetAddressFromRealDstPort(int realDstPort) {
  Ptr<Node> node = NULL;

  for (unsigned int i = 0; i < m_pegasusVars->m_ns3PegasusDroneApps.size(); i++) {
    Ptr<NS3PegasusDroneApp> app = m_pegasusVars->m_ns3PegasusDroneApps[i];
    if (app->m_realDstPortMapVirtualSocket.find(realDstPort) !=
        app->m_realDstPortMapVirtualSocket.end()) {
      node = app->GetNode();
      break;
    }
  }

  if (!node) 
    NS_FATAL_ERROR ("Failed to find destination node address.");

  Ptr<NetDevice> dev = node->GetDevice(0);
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
  int ipv4_idx = ipv4->GetInterfaceForDevice(dev);

  return ipv4->GetAddress(ipv4_idx, 0).GetLocal();
}

PegasusSocket* NS3PegasusDroneApp::FindPegasusSocket(int port) {
  std::vector<PegasusSocket *>::iterator res = std::find_if(
      m_pegasusVars->m_pegasusSockets.begin(),
      m_pegasusVars->m_pegasusSockets.end(),
      [port] (PegasusSocket *psoc) {
        return psoc->Get_m_port() == port;
      });
  if (res == m_pegasusVars->m_pegasusSockets.end())
    return NULL;

  if ((*res)->Get_m_node() != GetNode())
    NS_FATAL_ERROR(port << " has invalid node " << Names::FindName(GetNode()) << " reference");
  return *res;
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

void NS3PegasusDroneApp::ScheduleSend(int port, int peerPort, const char * buffer, const unsigned int & len) {
  NS_LOG_FUNCTION(this);
  Ptr<Socket> socket = m_realDstPortMapVirtualSocket[port];
  Ptr<Node> node = NULL;
  Address addr = GetAddressFromRealDstPort(peerPort);
  InetSocketAddress s_addr = InetSocketAddress (Ipv4Address::ConvertFrom(addr), peerPort);
  
  Ptr<Packet> p = Create<Packet>((uint8_t*)buffer, len);

  Simulator::ScheduleWithContext(m_pegasusVars->m_simulatorContext,
      Seconds(0), &NS3PegasusDroneApp::Send, this, socket, p, 0, s_addr);
  //int ret = socket->SendTo(p, 0, s_addr);
  //NS_LOG_DEBUG(ret);
}

void NS3PegasusDroneApp::Send(const Ptr<Socket> & socket, const Ptr<Packet> & packet, int flags, const InetSocketAddress & addr) {
  NS_LOG_FUNCTION(this);
  int ret = socket->SendTo(packet, 0, addr);
  NS_LOG_DEBUG("Packet of size " << ret << " goes inside the matrix.");
}

void NS3PegasusDroneApp::HandleRead(Ptr<Socket> socket) {
  NS_LOG_FUNCTION(this);
  Ptr<Packet> packet;
  Address from;
  Address localAddress;
  char buffer[9000]; // Huge packets
  while ((packet = socket->RecvFrom (from)))
  {
    socket->GetSockName (localAddress);
    int port = InetSocketAddress::ConvertFrom (localAddress).GetPort();

    PegasusSocket * psock = FindPegasusSocket(port);
    if (!psock)
      NS_LOG_INFO("Packet received on non bound port " << port);
    else {
      unsigned int size = packet->CopyData((uint8_t *)buffer, 9000); 
      if (size > 0)
        psock->Send(buffer, size);
    }

    m_rxTrace (packet);
    m_rxTraceWithAddresses (packet, from, localAddress);
    if (packet->GetSize () > 0)
    {
        NS_LOG_INFO ("TraceDelay: RX " << packet->GetSize () <<
            " bytes from "<< InetSocketAddress::ConvertFrom (from).GetIpv4 () <<
            " Uid: " << packet->GetUid ());
      }
  }
}

