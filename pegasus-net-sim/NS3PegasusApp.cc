
#include "NS3PegasusApp.h"
#include "PegasusSocket.h"
#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("NS3PegasusApp");
NS_OBJECT_ENSURE_REGISTERED (NS3PegasusApp);

void NS3PegasusApp::StartApplication(void ) {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusApp::StopApplication(void ) {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusApp::DoDispose(void ) {
  NS_LOG_FUNCTION(this);
  Application::DoDispose();
}

TypeId NS3PegasusApp::GetTypeId(void )
{
   static TypeId tid = TypeId ("ns3::NS3PegasusApp")
     .SetParent<Application> ()
     .SetGroupName("Applications")
     .AddConstructor<NS3PegasusApp> ()
     .AddTraceSource ("Rx", "A packet has been received",
                      MakeTraceSourceAccessor (&NS3PegasusApp::m_rxTrace),
                      "ns3::Packet::TracedCallback")
     .AddTraceSource ("RxWithAddresses", "A packet has been received",
                      MakeTraceSourceAccessor (&NS3PegasusApp::m_rxTraceWithAddresses),
                      "ns3::Packet::TwoAddressTracedCallback")
   ;
   return tid;
}

NS3PegasusApp::NS3PegasusApp() {
  NS_LOG_FUNCTION(this);
}

NS3PegasusApp::~NS3PegasusApp() {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusApp::ScheduleSend() {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusApp::Send() {
  NS_LOG_FUNCTION(this);
}

void NS3PegasusApp::HandleRead() {
  NS_LOG_FUNCTION(this);
}
