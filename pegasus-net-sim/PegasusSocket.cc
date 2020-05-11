
#include "PegasusSocket.h"
#include "PegasusConfig.h"
#include "PegasusPacket.h"

#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("PegasusSocket");

PegasusSocket::PegasusSocket() {
  NS_LOG_FUNCTION(this);
}

PegasusSocket::~PegasusSocket() {
  NS_LOG_FUNCTION(this);
}

void PegasusSocket::SetAttributes(PegasusPortConfig * portConfig, const Ptr<Node> & node) {
  NS_LOG_FUNCTION(this);
  m_portConfig = portConfig;
  m_node = node;
}
