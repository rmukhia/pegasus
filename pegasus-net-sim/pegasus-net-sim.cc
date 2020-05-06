#include "PegasusNetSim.h"

/** Set this list to the name of the drones you are simulating **/

std::vector<std::string> droneNames = {
  "iris_0",
  "iris_1",
  "iris_2",
};


PegasusNetSim *pegasusNetSim;

void changePosition()
{
  std::for_each(GazeboNode::poseMap.begin(), GazeboNode::poseMap.end(),
    [] (std::pair<std::string, ns3::Vector> element) {
      // first is key, second is value
      ns3::Ptr<ns3::Node> drone =  ns3::Names::Find<ns3::Node>(
          "/Pegasus/drones",
          element.first
        );

      if (!drone) return;
      ns3::Ptr<ns3::ConstantPositionMobilityModel> droneMobilityModel
        = drone->GetObject<ns3::ConstantPositionMobilityModel>();

      ns3::Simulator::ScheduleNow(
            &ns3::ConstantPositionMobilityModel::SetPosition,
            droneMobilityModel, element.second
          );

    }
  );
  ns3::Simulator::Schedule(ns3::MilliSeconds(250), &changePosition);
}


int main (int argc, char *argv[])
{
  bool verbose = true;
  bool tracing = false;

  pegasusNetSim = PegasusNetSim::Instance();

  pegasusNetSim->gazeboNode->Setup(argc, argv);
  pegasusNetSim->gazeboNode->SetTopic("~/pose/info");
  pegasusNetSim->gazeboNode->SetModelsName(droneNames);

  ns3::CommandLine cmd;
  // cmd.AddValue ("nDrones", "Number of drones (max 18)", nDrones);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc,argv);

#if 0
  if (verbose) {
    ns3::LogComponentEnable ("UdpEchoClientApplication", ns3::LOG_LEVEL_INFO);
    ns3::LogComponentEnable ("UdpEchoServerApplication", ns3::LOG_LEVEL_INFO);
  }
#endif

  ns3::LogComponentEnable ("PegasusNS3Runner", ns3::LOG_LEVEL_ALL);
  ns3::LogComponentEnable ("PegasusGazeboNode", ns3::LOG_LEVEL_ALL);

  pegasusNetSim->ns3Runner->Create(droneNames);

  if (tracing == true)
    pegasusNetSim->ns3Runner->EnableTracing();

  pegasusNetSim->gazeboNode->Subscribe();

  ns3::Simulator::Stop (ns3::Seconds (3.0));

  ns3::Simulator::Schedule(ns3::Seconds(1), &changePosition);
  ns3::Simulator::Run ();
  ns3::Simulator::Destroy ();

  pegasusNetSim->gazeboNode->Destroy();

  delete pegasusNetSim;

  return 0;
}
