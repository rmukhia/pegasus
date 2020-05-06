#include "NS3Runner.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNS3Runner");

NS3Runner::NS3Runner() {
  NS_LOG_FUNCTION(this);
  PacketMetadata::Enable ();
}

void NS3Runner::Create(std::vector<std::string> &droneNames)
{
  NS_LOG_FUNCTION(this);
  droneNames = droneNames;
  CreateNode();
  CreateWifiChnlPhy();
  CreateMeshNetwork();
  CreateNetDevices();
  CreateMobility();
  CreateRouting();
  CreateIpAddr();
}

void NS3Runner::CreateNode()
{
  NS_LOG_FUNCTION(this);
  droneNodes.Create (droneNames.size());
  for (unsigned int i = 0; i < droneNames.size(); i++) {
    Names::Add("/Pegasus/drones", droneNames[i], droneNodes.Get(i));
  }
  /* One Control Station */
  controlStationNode.Create(1);
  Names::Add("/Pegasus", "controlStation", controlStationNode.Get(0));
}

void NS3Runner::CreateWifiChnlPhy()
{
  NS_LOG_FUNCTION(this);
  channel = YansWifiChannelHelper::Default ();
  physical = YansWifiPhyHelper::Default ();
  physical.SetChannel (channel.Create ());
}

void NS3Runner::CreateMeshNetwork()
{
  NS_LOG_FUNCTION(this);
  mesh = MeshHelper::Default();
  mesh.SetStackInstaller("ns3::Dot11sStack");
}

void NS3Runner::CreateNetDevices()
{
  NS_LOG_FUNCTION(this);
  droneDevices = mesh.Install (physical, droneNodes);
  controlStationDevice = mesh.Install (physical, controlStationNode);
}

void NS3Runner::CreateMobility()
{
  NS_LOG_FUNCTION(this);
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
      "MinX", DoubleValue (0.0),
      "MinY", DoubleValue (0.0),
      "DeltaX", DoubleValue (0.0),
      "DeltaY", DoubleValue (0.0),
      "GridWidth", UintegerValue (3),
      "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (controlStationNode);
  mobility.Install (droneNodes);
}

void NS3Runner::CreateRouting()
{
  NS_LOG_FUNCTION(this);
  Ipv4StaticRoutingHelper staticRouting;
  /* The second parameter is the priority */
  list.Add(staticRouting, 0);
  list.Add (olsr, 10);
  stack.SetRoutingHelper (list);
  stack.Install (controlStationNode);
  stack.Install (droneNodes);
}


void NS3Runner::CreateIpAddr()
{
  NS_LOG_FUNCTION(this);
  address.SetBase ("10.1.3.0", "255.255.255.0");
  controlStationInterfaces = address.Assign (controlStationDevice);
  droneInterfaces = address.Assign (droneDevices);

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
}


void NS3Runner::EnableTracing()
{
  NS_LOG_FUNCTION(this);
  physical.EnablePcap ("pegasus", controlStationDevice.Get (0));
}
