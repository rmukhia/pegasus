#include "NS3Runner.h"
#include "GazeboNode.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNetSim");

GazeboNode *gazeboNode;
NS3Runner *ns3runner;

void changePosition()
{
  ns3::Ptr<ns3::Node> drone = ns3runner->GetDroneNodes().Get(0);

  ns3::Ptr<ns3::ConstantPositionMobilityModel> droneMobilityModel
    = drone->GetObject<ns3::ConstantPositionMobilityModel>();

  ns3::Simulator::ScheduleNow(
      &ns3::ConstantPositionMobilityModel::SetPosition,
      droneMobilityModel, GazeboNode::poseMap["iris_0"]
      );

  ns3::Simulator::Schedule(ns3::MilliSeconds(100), &changePosition);
}


int main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nDrones = 1;
  bool tracing = false;

  gazeboNode = new GazeboNode();

  gazeboNode->Setup(argc, argv);
  gazeboNode->SetTopic("~/pose/info");
  gazeboNode->SetModelsName({ "iris_0", "iris_1" });

  ns3::CommandLine cmd;
  cmd.AddValue ("nDrones", "Number of drones (max 18)", nDrones);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc,argv);

  if (nDrones > 18) {
    std::cout << "nDrones should be 18 or less; otherwise grid layout exceeds the bounding box"
      << std::endl;
    return 1;
  }

  if (verbose) {
    ns3::LogComponentEnable ("UdpEchoClientApplication", ns3::LOG_LEVEL_INFO);
    ns3::LogComponentEnable ("UdpEchoServerApplication", ns3::LOG_LEVEL_INFO);
  }

  ns3runner = new NS3Runner();

  ns3runner->Create(nDrones);

  if (tracing == true)
    ns3runner->EnableTracing();

  gazeboNode->Subscribe();

  ns3::Simulator::Stop (ns3::Seconds (1000.0));

  ns3::Simulator::Schedule(ns3::Seconds(1), &changePosition);
  ns3::Simulator::Run ();
  ns3::Simulator::Destroy ();

  gazeboNode->Destroy();

  delete gazeboNode;
  delete ns3runner;
  return 0;
}
