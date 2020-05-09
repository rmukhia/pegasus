#ifndef _NS3RUNNER_H
#define _NS3RUNNER_H


#include "NS3PegasusDroneApp.h"

#include "ns3/core-module.h"
#include "ns3/log.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/minstrel-wifi-manager.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-helper.h"

using namespace ns3;

class PegasusVariables;

class NS3Runner {
  private:
    static PegasusVariables * m_pegasusVars;

    YansWifiChannelHelper m_channel;

    YansWifiPhyHelper m_physical;

    MeshHelper m_mesh;

    NetDeviceContainer m_controlStationDevice;

    NetDeviceContainer m_droneDevices;

    MobilityHelper m_mobility;

    OlsrHelper m_olsr;

    Ipv4ListRoutingHelper m_list;

    InternetStackHelper m_stack;

    Ipv4AddressHelper m_address;

    Ipv4InterfaceContainer m_controlStationInterfaces;

    Ipv4InterfaceContainer m_droneInterfaces;

    void CreateNode();

    void CreateWifiChnlPhy();

    void CreateMeshNetwork();

    void CreateNetDevices();

    void CreateMobility();

    void CreateRouting();

    void CreateIpAddr();

    void InstallApplications();


  public:
    NS3Runner();

    ~NS3Runner();

    void Create();

    void EnableTracing();

    static void Set_m_pegasusVars(PegasusVariables * value);

    void CreateProxyRoute(int srcPort, int proxyDstPort, int proxySrcPort, int dstPort);

};
#endif
