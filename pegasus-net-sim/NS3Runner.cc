#include "NS3Runner.h"

NS3Runner::NS3Runner() {
  ns3::PacketMetadata::Enable ();
}

void NS3Runner::Create(std::vector<std::string> &droneNames)
{
  this->droneNames = droneNames;
  this->numDrones = droneNames.size();
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
  droneNodes.Create (numDrones);
  for (unsigned int i = 0; i < numDrones; i++) {
    ns3::Names::Add("/Pegasus/drones", droneNames[i], droneNodes.Get(i));
  }
  /* One Control Station */
  controlStationNode.Create(1);
  ns3::Names::Add("/Pegasus", "controlStation", controlStationNode.Get(0));
}

void NS3Runner::CreateWifiChnlPhy()
{
  channel = ns3::YansWifiChannelHelper::Default ();
  physical = ns3::YansWifiPhyHelper::Default ();
  physical.SetChannel (channel.Create ());
}

void NS3Runner::CreateMeshNetwork()
{
  mesh = ns3::MeshHelper::Default();
  mesh.SetStackInstaller("ns3::Dot11sStack");
}

void NS3Runner::CreateNetDevices()
{
  droneDevices = mesh.Install (physical, droneNodes);
  controlStationDevice = mesh.Install (physical, controlStationNode);
}

void NS3Runner::CreateMobility()
{

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
      "MinX", ns3::DoubleValue (0.0),
      "MinY", ns3::DoubleValue (0.0),
      "DeltaX", ns3::DoubleValue (0.0),
      "DeltaY", ns3::DoubleValue (0.0),
      "GridWidth", ns3::UintegerValue (3),
      "LayoutType", ns3::StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (controlStationNode);
  mobility.Install (droneNodes);
}

void NS3Runner::CreateRouting()
{
  /* The second parameter is the priority */
  list.Add (olsr, 4);
  stack.SetRoutingHelper (list);
  stack.Install (controlStationNode);
  stack.Install (droneNodes);
}


void NS3Runner::CreateIpAddr()
{
  address.SetBase ("10.1.3.0", "255.255.255.0");
  controlStationInterfaces = address.Assign (controlStationDevice);
  droneInterfaces = address.Assign (droneDevices);
}


void NS3Runner::EnableTracing()
{
  physical.EnablePcap ("third", controlStationDevice.Get (0));
}
