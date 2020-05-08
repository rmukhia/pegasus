
#include "NS3Runner.h"
#include "PegasusVariables.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNS3Runner");
PegasusVariables * NS3Runner::m_pegasusVars;

void NS3Runner::CreateNode() {
  NS_LOG_FUNCTION(this);
  m_pegasusVars->m_droneNodes.Create (m_pegasusVars->m_modelsName.size());
  for (unsigned int i = 0; i < m_pegasusVars->m_modelsName.size(); i++) {
    Names::Add("/Pegasus/drones", m_pegasusVars->m_modelsName[i], m_pegasusVars->m_droneNodes.Get(i));
  }
  /* One Control Station */
  m_pegasusVars->m_controlStationNode.Create(1);
  Names::Add("/Pegasus", "controlStation", m_pegasusVars->m_controlStationNode.Get(0));

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
  Ptr<Node> node = m_pegasusVars->m_controlStationNode.Get(0);
  m_pegasusVars->m_ns3PegasusControlStationApp = CreateObject<NS3PegasusControlStationApp>();
  node->AddApplication(m_pegasusVars->m_ns3PegasusControlStationApp);
  apps.Add(m_pegasusVars->m_ns3PegasusControlStationApp);

  int i = 5550;
  std::for_each(m_pegasusVars->m_droneNodes.Begin(), m_pegasusVars->m_droneNodes.End(),
      [this, &apps, &i] (const Ptr<Node> droneNode) {
        m_pegasusVars->m_ns3PegasusDroneApps.push_back(CreateObject<NS3PegasusDroneApp>());
        Ptr<NS3PegasusDroneApp> app = m_pegasusVars->m_ns3PegasusDroneApps[
          m_pegasusVars->m_ns3PegasusDroneApps.size() -1
        ];

        droneNode->AddApplication(app);
        apps.Add(app);

        app->m_realDstPortMapVirtualSocket[i] = app->CreateVirtualSocket(i, app);

        Simulator::Schedule(Seconds(5), &NS3PegasusDroneApp::Send, app, i, (Ptr<Packet>) NULL, i);
        i++;
      }
    );

  apps.Start(Seconds(1));
  apps.Stop(Seconds(60));
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

