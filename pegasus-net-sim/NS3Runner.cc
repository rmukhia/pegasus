
#include "NS3Runner.h"
#include "PegasusVariables.h"
#include "PegasusConfig.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNS3Runner");
PegasusVariables * NS3Runner::m_pegasusVars;

void NS3Runner::CreateNode() {
  NS_LOG_FUNCTION(this);

  auto numNodes = PegasusConfig::m_config.size();
  // n - 1 drones 
  m_pegasusVars->m_droneNodes.Create(numNodes - 1);
  // 1 control station
  m_pegasusVars->m_controlStationNode.Create(1);

  int i = 0;
  for(auto const &config: PegasusConfig::m_config) {
    if (config.first.compare(CONTROL_STATION_STR)) // not equal
      Names::Add("/Pegasus/drones", config.first, m_pegasusVars->m_droneNodes.Get(i++));
  }

  /* One Control Station */
  Names::Add("/Pegasus", CONTROL_STATION_STR, m_pegasusVars->m_controlStationNode.Get(0));

  m_pegasusVars->m_nodes.Add(m_pegasusVars->m_droneNodes);
  m_pegasusVars->m_nodes.Add(m_pegasusVars->m_controlStationNode);
}

void NS3Runner::CreateWifiChnlPhy() {
  NS_LOG_FUNCTION(this);
  m_channel = YansWifiChannelHelper::Default ();
  m_physical = YansWifiPhyHelper::Default ();
  m_physical.SetChannel (m_channel.Create ());

}

void NS3Runner::CreateMeshNetwork() {
  NS_LOG_FUNCTION(this);
  m_mesh = MeshHelper::Default();
  m_mesh.SetStackInstaller("ns3::Dot11sStack");
}

void NS3Runner::CreateNetDevices() {
  NS_LOG_FUNCTION(this);
  m_droneDevices = m_mesh.Install (m_physical, m_pegasusVars->m_droneNodes);
  m_controlStationDevice = m_mesh.Install (m_physical, m_pegasusVars->m_controlStationNode);
}

void NS3Runner::CreateMobility() {
  NS_LOG_FUNCTION(this);
  m_mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
      "MinX", DoubleValue (0.0),
      "MinY", DoubleValue (0.0),
      "DeltaX", DoubleValue (10.0),
      "DeltaY", DoubleValue (10.0),
      "GridWidth", UintegerValue (3),
      "LayoutType", StringValue ("RowFirst"));

  m_mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  m_mobility.Install (m_pegasusVars->m_controlStationNode);
  m_mobility.Install (m_pegasusVars->m_droneNodes);
}

void NS3Runner::CreateRouting() {
  NS_LOG_FUNCTION(this);
  Ipv4StaticRoutingHelper staticRouting;
  /* The second parameter is the priority */
  m_list.Add(staticRouting, 0);
  m_list.Add (m_olsr, 10);
  m_stack.SetRoutingHelper (m_list);
  m_stack.Install (m_pegasusVars->m_controlStationNode);
  m_stack.Install (m_pegasusVars->m_droneNodes);
}

void NS3Runner::CreateIpAddr() {
  NS_LOG_FUNCTION(this);
  m_address.SetBase ("10.1.3.0", "255.255.255.0");
  m_controlStationInterfaces = m_address.Assign (m_controlStationDevice);
  m_droneInterfaces = m_address.Assign (m_droneDevices);

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
}

void NS3Runner::InstallApplications() {
  NS_LOG_FUNCTION(this);

  ApplicationContainer apps;
  auto ns3PegasusDroneApps = &m_pegasusVars->m_ns3PegasusDroneApps;

  for(auto const &config: PegasusConfig::m_config) {
    auto name = config.first;
    auto portParamList = config.second;
    Ptr<Node> node;

    if (name.compare(CONTROL_STATION_STR)) // drone
      node = Names::Find<Node>("/Pegasus/drones", name);
    else // control station
      node = Names::Find<Node>("/Pegasus", CONTROL_STATION_STR);

    if (!node) continue;

    for(auto const &portParam: portParamList) {
      auto app = CreateObject<NS3PegasusDroneApp>();

      node->AddApplication(app);
      apps.Add(app);
      app->m_portMapVirtualSocket[portParam.m_port] = app->CreateVirtualSocket(portParam.m_port);
      ns3PegasusDroneApps->push_back(app);
    }
  }

  apps.Start(Seconds(5));
  apps.Stop(Seconds(1000));
}

NS3Runner::NS3Runner(){
  NS_LOG_FUNCTION(this);
  PacketMetadata::Enable ();
}

NS3Runner::~NS3Runner(){

}

void NS3Runner::Create() {
  NS_LOG_FUNCTION(this);
  CreateNode();
  CreateWifiChnlPhy();
  CreateMeshNetwork();
  CreateNetDevices();
  CreateMobility();
  CreateRouting();
  CreateIpAddr();
  InstallApplications();
}

void NS3Runner::EnableTracing() {
  NS_LOG_FUNCTION(this);
  m_physical.EnablePcap ("pegasus", m_controlStationDevice.Get (0));
}

void NS3Runner::Set_m_pegasusVars(PegasusVariables * value)
{
  m_pegasusVars = value;
}

