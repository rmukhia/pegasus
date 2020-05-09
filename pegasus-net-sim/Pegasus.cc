
#include "Pegasus.h"
#include "NS3PegasusDroneApp.h"
#include "PegasusUDPSocket.h"

#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("Pegasus");

PegasusVariables Pegasus::m_pegasusVars;

Pegasus * Pegasus::sm_instance= NULL;

Pegasus::Pegasus(){
}

void Pegasus::SetupProxy() {
  PegasusUDPSocket *psoc;

  psoc = new PegasusUDPSocket();
  // Bound to 5550, transmits to 5440, transmits in ns3 to to 5551
  psoc->SetAttributes(5550, 5440, 5551, m_pegasusVars.m_nodes.Get(0));
  m_pegasusVars.m_pegasusSockets.push_back(psoc);

  psoc = new PegasusUDPSocket();
  // Bound to 5551, transmits to 5510, transmits in ns3 to to 5550
  psoc->SetAttributes(5551, 5510, 5550, m_pegasusVars.m_nodes.Get(1));
  m_pegasusVars.m_pegasusSockets.push_back(psoc);

  std::for_each(m_pegasusVars.m_pegasusSockets.begin(), m_pegasusVars.m_pegasusSockets.end(),
      [] (PegasusSocket *pegasusSock) {
      pegasusSock->Create();
      pegasusSock->Bind();
  });
}

Pegasus::~Pegasus(){
  std::for_each(m_pegasusVars.m_pegasusSockets.begin(), m_pegasusVars.m_pegasusSockets.end(),
      [] (PegasusSocket *pegasusSock) {
      delete pegasusSock;
  });
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
    std::for_each(m_pegasusVars.m_poseMap.begin(), m_pegasusVars.m_poseMap.end(),
        [] (std::pair<std::string, Vector> element) {
        // first is key, second is value
        Ptr<Node> drone =  Names::Find<Node>(
            "/Pegasus/drones",
            element.first
            );

        if (!drone) return;
        Ptr<ConstantPositionMobilityModel> droneMobilityModel
        = drone->GetObject<ConstantPositionMobilityModel>();

        Simulator::ScheduleNow(
            &ConstantPositionMobilityModel::SetPosition,
            droneMobilityModel, element.second
            );

        }
    );
  }
  Simulator::Schedule(MilliSeconds(250), &Pegasus::ChangeDronesPosition);
}

void Pegasus::Run(int argc, char** argv, const std::vector<std::string> & droneNames) {
  bool verbose = true;
  bool tracing = false;

  m_pegasusVars.m_modelsName = droneNames;

  CommandLine cmd;
  // cmd.AddValue ("nDrones", "Number of drones (max 18)", nDrones);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc,argv);

  if (verbose) {
    LogComponentEnable ("PegasusNS3Runner", LOG_LEVEL_INFO);
    LogComponentEnable ("PegasusGazeboNode", LOG_LEVEL_INFO);
    LogComponentEnable ("PegasusNS3DroneApp", LOG_LEVEL_INFO);
    LogComponentEnable ("PegasusUDPSocket", LOG_LEVEL_INFO);
    LogComponentEnable ("PegasusSocket", LOG_LEVEL_INFO);
    LogComponentEnable ("PegasusNS3SocketRunner", LOG_LEVEL_INFO);
    LogComponentEnable ("Pegasus", LOG_LEVEL_INFO);
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

