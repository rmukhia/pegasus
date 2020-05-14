#ifndef _PEGASUSTRACE_H
#define _PEGASUSTRACE_H

#include "ns3/node.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/wifi-phy.h"

using namespace ns3;

#include <string>

class PegasusTrace {
  private:
    Ptr<Node> m_node;

    Time m_timestamp;

    double m_avgNoiseDbm;

    double m_avgSignalDbm;

    uint32_t m_samples;


  public:
    PegasusTrace(const Ptr<Node> & node);

    void HandleWifiTrace(std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise);

    inline const Ptr<Node> Get_m_node() const;

    inline const double Get_m_avgNoiseDbm() const;

    inline const double Get_m_avgSignalDbm() const;

};
inline const Ptr<Node> PegasusTrace::Get_m_node() const {
  return m_node;
}

inline const double PegasusTrace::Get_m_avgNoiseDbm() const {
  return m_avgNoiseDbm;
}

inline const double PegasusTrace::Get_m_avgSignalDbm() const {
  return m_avgSignalDbm;
}

#endif
