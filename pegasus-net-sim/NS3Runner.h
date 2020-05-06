//
// Created by rmukhia on 2/5/63.
//

#ifndef PEGASUS_SIM_NS3RUNNER_H
#define PEGASUS_SIM_NS3RUNNER_H

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

class NS3Runner {
  private:
    std::vector<std::string> droneNames;
    unsigned int numDrones;
    NodeContainer controlStationNode;
    NodeContainer droneNodes;
    YansWifiChannelHelper channel;
    YansWifiPhyHelper physical;
    MeshHelper mesh;
    NetDeviceContainer controlStationDevice;
    NetDeviceContainer droneDevices;
    MobilityHelper mobility;
    OlsrHelper olsr;
    Ipv4ListRoutingHelper list;
    InternetStackHelper stack;
    Ipv4AddressHelper address;
    Ipv4InterfaceContainer controlStationInterfaces;
    Ipv4InterfaceContainer droneInterfaces;

  public:
    NS3Runner();
    void Create(std::vector<std::string> &droneNames);
    void CreateNode();
    void CreateWifiChnlPhy();
    void CreateMeshNetwork();
    void CreateNetDevices();
    void CreateMobility();
    void CreateRouting();
    void CreateIpAddr();
    void EnableTracing();
    
    const NodeContainer& GetDroneNodes() const { return droneNodes; };
};


#endif //PEGASUS_SIM_NS3RUNNER_H
