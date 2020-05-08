
#include "NS3PegasusControlStationApp.h"

# include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNS3ControlStationApp");
NS_OBJECT_ENSURE_REGISTERED (NS3PegasusControlStationApp);

void NS3PegasusControlStationApp::StartApplication(void ) {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusControlStationApp::StopApplication(void ) {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusControlStationApp::DoInitialize() {
  NS_LOG_FUNCTION(this);
  Application::DoInitialize();
}

void NS3PegasusControlStationApp::DoDispose(void ) {
  NS_LOG_FUNCTION(this);
  Application::DoDispose();
}

TypeId NS3PegasusControlStationApp::GetTypeId(void )
{
   static TypeId tid = TypeId ("NS3PegasusControlStationApp")
     .SetParent<Application> ()
     .SetGroupName("NS3PegasusControlStationApp")
     .AddConstructor<NS3PegasusControlStationApp> ()
     .AddTraceSource ("Rx", "A packet has been received",
                      MakeTraceSourceAccessor (&NS3PegasusControlStationApp::m_rxTrace),
                      "ns3::Packet::TracedCallback")
     .AddTraceSource ("RxWithAddresses", "A packet has been received",
                      MakeTraceSourceAccessor (&NS3PegasusControlStationApp::m_rxTraceWithAddresses),
                      "ns3::Packet::TwoAddressTracedCallback");
   return tid;
}

NS3PegasusControlStationApp::NS3PegasusControlStationApp() {
  NS_LOG_FUNCTION(this);
}

NS3PegasusControlStationApp::~NS3PegasusControlStationApp() {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusControlStationApp::ScheduleSend() {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusControlStationApp::Send() {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusControlStationApp::HandleRead() {
  NS_LOG_FUNCTION(this);
}

