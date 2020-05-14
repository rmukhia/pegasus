
#include "PegasusTrace.h"

#include "ns3/wifi-module.h"
#include "ns3/simulator.h"

using namespace ns3;


#define TRACE_AVERAGE_SMOOTHING_TIME_MS 100

PegasusTrace::PegasusTrace(const Ptr<Node> & node) {
  m_node = node;
  m_samples = m_avgSignalDbm = m_avgNoiseDbm = 0;
  m_timestamp = Simulator::Now();
}

void PegasusTrace::HandleWifiTrace(std::string context, Ptr<const Packet> packet,
    uint16_t channelFreqMhz, WifiTxVector txVector,
    MpduInfo aMpdu, SignalNoiseDbm signalNoise) {
  if (m_timestamp + MilliSeconds(TRACE_AVERAGE_SMOOTHING_TIME_MS) < Simulator::Now()) {
    // reset
    m_samples = m_avgSignalDbm = m_avgNoiseDbm = 0;
    m_timestamp = Simulator::Now();
  }

  m_samples++;

  m_avgNoiseDbm += (signalNoise.noise - m_avgNoiseDbm) / m_samples;
  m_avgSignalDbm += (signalNoise.signal - m_avgSignalDbm) / m_samples;
}

