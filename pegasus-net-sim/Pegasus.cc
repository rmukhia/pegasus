
#include "Pegasus.h"
#include "NS3PegasusDroneApp.h"
#include "PegasusUDPSocket.h"

#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("Pegasus");

PegasusVariables Pegasus::m_pegasusVars;

Pegasus * Pegasus::sm_instance= NULL;

Pegasus::Pegasus(){
  NS_LOG_FUNCTION(this);
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

Pegasus::~Pegasus(){
  NS_LOG_FUNCTION(this);

  auto pegasusSockets = &m_pegasusVars.m_pegasusSockets;

  for(auto const &psock: *pegasusSockets) {
    delete psock;
  }
}

Pegasus* Pegasus::GetInstance()
{
  if (!sm_instance) {
    sm_instance = new Pegasus();
    sm_instance->m_gazeboNode.Set_m_pegasusVars(&sm_instance->m_pegasusVars);
    sm_instance->m_ns3Runner.Set_m_pegasusVars(&sm_instance->m_pegasusVars);
    sm_instance->m_pegasusSocketRunner.Set_m_pegasusVars(&sm_instance->m_pegasusVars);
    NS3PegasusDroneApp::Set_m_pegasusVars(&sm_instance->m_pegasusVars);
  }

  return sm_instance;
}

void Pegasus::ChangeDronesPosition() {
  {
    CriticalSection(m_pegasusVars.m_poseMapMutex);

    for(auto const &element: m_pegasusVars.m_poseMap) {
        auto drone = Names::Find<Node>("/Pegasus/drones", element.first);
        if (!drone) continue;
        auto droneMobilityModel = drone->GetObject<ConstantPositionMobilityModel>();

        Simulator::ScheduleWithContext(
            m_pegasusVars.m_simulatorContext,
            Seconds(0),
            &ConstantPositionMobilityModel::SetPosition,
            droneMobilityModel, element.second
            );
    }
  }

  Simulator::ScheduleWithContext(
      m_pegasusVars.m_simulatorContext,
      MilliSeconds(250),
      &Pegasus::ChangeDronesPosition);
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

  m_pegasusVars.m_simulatorContext = Simulator::GetContext();

  m_gazeboNode.Subscribe();
  m_pegasusSocketRunner.Start();

  if (tracing == true)
    m_ns3Runner.EnableTracing();

  // Start a new subscriber thread

  Simulator::Stop (Seconds (1000));

  Simulator::Schedule(Seconds(1), &Pegasus::ChangeDronesPosition);
  Simulator::Run ();
  Simulator::Destroy ();

  m_gazeboNode.Destroy();
  m_pegasusSocketRunner.Stop();
}

