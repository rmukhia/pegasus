/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mesh-helper.h"
#include "ns3/minstrel-wifi-manager.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-helper.h"

#include "GazeboNode.h"
// Default Network Topology
//
//   Wifi 10.1.3.0
//                 AP
//  *    *    *    *
//  |    |    |    |    10.1.1.0
// n5   n6   n7   n0 -------------- n1   n2   n3   n4
//                   point-to-point  |    |    |    |
//                                   ================
//                                     LAN 10.1.2.0

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("PegasusNetSim");

int 
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nWifi = 18;
  bool tracing = false;

  GazeboNode gazeboNode;

  gazeboNode.setup(argc, argv);
  gazeboNode.setTopic("~/pose/info");
  gazeboNode.subscribe();

  CommandLine cmd;
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc,argv);

  // The underlying restriction of 18 is due to the grid position
  // allocator's configuration; the grid layout will exceed the
  // bounding box if more than 18 nodes are provided.
  if (nWifi > 18)
    {
      std::cout << "nWifi should be 18 or less; otherwise grid layout exceeds the bounding box" << std::endl;
      return 1;
    }

  if (verbose)
    {
      LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
    }


  PacketMetadata::Enable ();
  /* Create mesh clients */
  NodeContainer meshClientNodes;
  meshClientNodes.Create (nWifi);

  /* Create mesh gateway router */
  NodeContainer meshRouterNode;
  meshRouterNode.Create(1);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  MeshHelper mesh = MeshHelper::Default();

  mesh.SetStackInstaller("ns3::Dot11sStack");


  NetDeviceContainer clientDevices;
  clientDevices = mesh.Install (phy, meshClientNodes);


  NetDeviceContainer routerDevices;
  routerDevices = mesh.Install (phy, meshRouterNode);

  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5.0),
                                 "DeltaY", DoubleValue (10.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));

  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (-300, 300, -300, 300)));
  mobility.Install (meshClientNodes);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (meshRouterNode);

  OlsrHelper olsr;
  Ipv4ListRoutingHelper list;
  list.Add (olsr, 10);

  InternetStackHelper stack;
  stack.SetRoutingHelper (list);
  stack.Install (meshRouterNode);
  stack.Install (meshClientNodes);

  Ipv4AddressHelper address;

  Ipv4InterfaceContainer clientInterfaces;

  address.SetBase ("10.1.3.0", "255.255.255.0");
  address.Assign (routerDevices);
  clientInterfaces = address.Assign (clientDevices);

  Simulator::Stop (Seconds (10.0));

  if (tracing == true)
    {
      phy.EnablePcap ("third", routerDevices.Get (0));
    }

  Simulator::Run ();
  Simulator::Destroy ();
  gazeboNode.destroy();
  return 0;
}
