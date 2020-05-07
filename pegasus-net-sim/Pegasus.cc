
#include "Pegasus.h"
#include "PegasusVariables.h"
#include "PegasusSocket.h"
#include "NS3PegasusApp.h"

PegasusVariables Pegasus::m_pegasusVars;

Pegasus * Pegasus::sm_instance= NULL;

Pegasus::Pegasus(){
}

Pegasus::~Pegasus(){
}

Pegasus* Pegasus::GetInstance()
{
  if (!sm_instance) {
    sm_instance = new Pegasus();
    sm_instance->m_gazeboNode.Set_m_pegasusVars(&sm_instance->m_pegasusVars);
    sm_instance->m_ns3Runner.Set_m_pegasusVars(&sm_instance->m_pegasusVars);
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
    LogComponentEnable ("PegasusNS3Runner", LOG_LEVEL_ALL);
    LogComponentEnable ("PegasusGazeboNode", LOG_LEVEL_ALL);
  }

  m_gazeboNode.Setup(argc, argv);
  m_gazeboNode.SetTopic("~/pose/info");

  m_ns3Runner.Create();


  if (tracing == true)
    m_ns3Runner.EnableTracing();

  m_gazeboNode.Subscribe();

  Simulator::Stop (Seconds (3.0));

  Simulator::Schedule(Seconds(1), &Pegasus::ChangeDronesPosition);
  Simulator::Run ();
  Simulator::Destroy ();

  m_gazeboNode.Destroy();
}

