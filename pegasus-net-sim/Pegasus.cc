
#include "Pegasus.h"
#include "PegasusTrace.h"
#include "NS3PegasusDroneApp.h"
#include "PegasusUDPSocket.h"

#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("Pegasus");

PegasusVariables Pegasus::m_pegasusVars;

Pegasus * Pegasus::sm_instance= NULL;

Pegasus::Pegasus(){
  NS_LOG_FUNCTION(this);
  m_running = true;
}

void Pegasus::SetupProxy() {
  NS_LOG_FUNCTION(this);

  auto pegasusSockets = &m_pegasusVars.m_pegasusSockets;

  for(auto &config: PegasusConfig::m_config) {
    auto name = config.first;
    auto portParamList = config.second;
    Ptr<Node> node;
    PegasusUDPSocket *psoc;

    if (name.compare(CONTROL_STATION_STR)) // drone
      node = Names::Find<Node>("/Pegasus/drones", name);
    else // control station
      node = Names::Find<Node>("/Pegasus", CONTROL_STATION_STR);

    if (!node) continue;

    for(auto i = 0u; i < portParamList.size(); i++) {
      // skip the iterator pointer management to get the actual static location
      auto portParam = &PegasusConfig::m_config[config.first][i];
      psoc = new PegasusUDPSocket();
      psoc->SetAttributes(portParam, node);
      pegasusSockets->push_back(psoc);
    }
  }

  for(auto const &psock: *pegasusSockets) {
    NS_LOG_INFO(psock->Get_m_portConfig());
      psock->Create();
      psock->Bind();
  };
}

void Pegasus::SetupTrace() {
  auto nodes = &m_pegasusVars.m_nodes;
  for (auto nodeItr = nodes->Begin(); nodeItr != nodes->End(); nodeItr++) {
    PegasusTrace pegasusTrace(*nodeItr);
    m_pegasusTraces.push_back(pegasusTrace);
  }

  for(auto i = 0u; i < m_pegasusTraces.size(); i++) {
    std::ostringstream oss;
    auto trace = &m_pegasusTraces[i];

    oss << "/NodeList/"
    << trace->Get_m_node()->GetId ()
    << "/DeviceList/*"
    << "/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";

    std::cout << oss.str() << std::endl;

    Config::Connect (oss.str(), MakeCallback (&PegasusTrace::HandleWifiTrace, trace));
  }

  m_stStatus = Create<SystemThread> (MakeCallback (&Pegasus::RunStatusThread, this));
  m_stStatus->Start();

}

void Pegasus::RunStatusThread() {
  struct timespec sleep = { 0 , 100000000 };
  PegasusPortConfig config(1999, 2999, 0);
  NS_LOG_INFO("Status UDP bind port 1999, remote port 2999.");
  m_statusPegasusSocket.SetAttributes(&config, NULL);
  m_statusPegasusSocket.Create();
  m_statusPegasusSocket.Bind();

  while(m_running) {
    for(auto const trace: m_pegasusTraces) {
      std::ostringstream oss;
      oss << "/" << Names::FindName(trace.Get_m_node())
        << "/" << trace.Get_m_avgSignalDbm()
        << "/" << trace.Get_m_avgNoiseDbm();
      std::cout << oss.str() << std::endl;
      m_statusPegasusSocket.Send(oss.str().c_str(), oss.str().length());
    }
    nanosleep(&sleep, NULL);
  }

}

Pegasus::~Pegasus(){
  NS_LOG_FUNCTION(this);

  auto pegasusSockets = &m_pegasusVars.m_pegasusSockets;

  for(auto const &psock: *pegasusSockets) {
    delete psock;
  }

  m_running = false;
}

Pegasus* Pegasus::GetInstance() {
  if (!sm_instance) {
    sm_instance = new Pegasus();
    sm_instance->m_gazeboNode.Set_m_pegasusVars(&sm_instance->m_pegasusVars);
    sm_instance->m_ns3Runner.Set_m_pegasusVars(&sm_instance->m_pegasusVars);
    sm_instance->m_pegasusSocketRunner.Set_m_pegasusVars(&sm_instance->m_pegasusVars);
    NS3PegasusDroneApp::Set_m_pegasusVars(&sm_instance->m_pegasusVars);
  }

  return sm_instance;
}

void Pegasus::ChangePosition() {
  {
    CriticalSection(m_pegasusVars.m_poseMapMutex);

    for(auto const &element: m_pegasusVars.m_poseMap) {

      Ptr<Node> node;

      if (element.first.compare(CONTROL_STATION_STR) == 0)
        node = Names::Find<Node>("/Pegasus", CONTROL_STATION_STR);
      else
        node = Names::Find<Node>("/Pegasus/drones", element.first);
      
      if (!node) continue;

      auto mobilityModel = node->GetObject<ConstantPositionMobilityModel>();

      Simulator::ScheduleWithContext(
          m_pegasusVars.m_simulatorContext,
          Seconds(0),
          &ConstantPositionMobilityModel::SetPosition,
          mobilityModel, element.second
          );
    }
  }

  Simulator::ScheduleWithContext(
      m_pegasusVars.m_simulatorContext,
      MilliSeconds(100),
      &Pegasus::ChangePosition);
}

void Pegasus::Run(int argc, char** argv) {
  NS_LOG_FUNCTION(this);

  bool verbose = true;
  bool tracing = false;

  GlobalValue::Bind ("SimulatorImplementationType",
      StringValue ("ns3::RealtimeSimulatorImpl"));
  Time::SetResolution (Time::NS);

  CommandLine cmd;
  // cmd.AddValue ("nDrones", "Number of drones (max 18)", nDrones);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc,argv);

  if (verbose) {
    //LogComponentEnable ("PegasusNS3Runner", LOG_LEVEL_LOGIC);
    //LogComponentEnable ("PegasusGazeboNode", LOG_LEVEL_LOGIC);
    //LogComponentEnable ("PegasusNS3DroneApp", LOG_LEVEL_LOGIC);
    //LogComponentEnable ("PegasusUDPSocket", LOG_LEVEL_LOGIC);
    //LogComponentEnable ("PegasusSocket", LOG_LEVEL_LOGIC);
    //LogComponentEnable ("PegasusNS3SocketRunner", LOG_LEVEL_LOGIC);
    //LogComponentEnable ("Pegasus", LOG_LEVEL_LOGIC);
  }

  m_gazeboNode.Setup(argc, argv);
  m_gazeboNode.SetTopic("~/pose/info");

  m_ns3Runner.Create();

  SetupProxy();

  SetupTrace();

  m_pegasusVars.m_simulatorContext = Simulator::GetContext();

  m_gazeboNode.Subscribe();
  m_pegasusSocketRunner.Start();

  if (tracing == true)
    m_ns3Runner.EnableTracing();

  // Start a new subscriber thread

  Simulator::Stop (Seconds (1000));

  Simulator::Schedule(Seconds(1), &Pegasus::ChangePosition);
  Simulator::Run ();
  Simulator::Destroy ();

  m_gazeboNode.Destroy();
  m_pegasusSocketRunner.Stop();
}

