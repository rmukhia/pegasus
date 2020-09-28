#ifndef _PEGASUSCONFIG_H
#define _PEGASUSCONFIG_H


#include <map>
#include <string>
#include <vector>


#define CONTROL_STATION_STR "control_station"  //name of the control station  
#define NS3_SEND_EVENT_INTERVAL 5000           // time interval between send events in ns3, the lower the lesser latecy but higher cpu usage.

//A node have a 'port' which a udp socket is bound to. A node expects udp packet to arrive at 'port' from 'peerPort'. A node transmits to 'peerPort'.
//A node simulation has udp socket bound to 'port' and ' virtualPeerPort' is the simulation 'port' the simulated udp socket transmits data to.
class PegasusPortConfig {
  public:
    int m_port;

    int m_peerPort;

    int m_virtualPeerPort;

    PegasusPortConfig(int port, int peerPort, int virtualPeerPort);

    inline const int Get_m_port() const;

    inline const int Get_m_peerPort() const;

    inline const int Get_m_virtualPeerPort() const;

};
inline const int PegasusPortConfig::Get_m_port() const {
  return m_port;
}

inline const int PegasusPortConfig::Get_m_peerPort() const {
  return m_peerPort;
}

inline const int PegasusPortConfig::Get_m_virtualPeerPort() const {
  return m_virtualPeerPort;
}

class PegasusConfig {
  public:
    static std::map<std::string, std::vector<PegasusPortConfig>> m_config;

};
#endif
