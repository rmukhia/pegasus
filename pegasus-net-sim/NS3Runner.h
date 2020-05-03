//
// Created by rmukhia on 2/5/63.
//

#ifndef PEGASUS_SIM_NS3RUNNER_H
#define PEGASUS_SIM_NS3RUNNER_H

#include "ns3/core-module.h"
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

class NS3Runner {
  private:
    int numDrones;
    ns3::NodeContainer controlStationNode;
    ns3::NodeContainer droneNodes;
    ns3::YansWifiChannelHelper channel;
    ns3::YansWifiPhyHelper physical;
    ns3::MeshHelper mesh;
    ns3::NetDeviceContainer controlStationDevice;
    ns3::NetDeviceContainer droneDevices;
    ns3::MobilityHelper mobility;
    ns3::OlsrHelper olsr;
    ns3::Ipv4ListRoutingHelper list;
    ns3::InternetStackHelper stack;
    ns3::Ipv4AddressHelper address;
    ns3::Ipv4InterfaceContainer controlStationInterfaces;
    ns3::Ipv4InterfaceContainer droneInterfaces;

  public:
    NS3Runner();
    void Create(int numDrones);
    void CreateNode(int numDrones);
    void CreateWifiChnlPhy();
    void CreateMeshNetwork();
    void CreateNetDevices();
    void CreateMobility();
    void CreateRouting();
    void CreateIpAddr();
    void EnableTracing();
    

    const ns3::NodeContainer& GetDroneNodes() const { return droneNodes; };
};


#endif //PEGASUS_SIM_NS3RUNNER_H
