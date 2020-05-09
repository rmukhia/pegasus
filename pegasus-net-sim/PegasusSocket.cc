
#include "PegasusSocket.h"

#include "ns3/log.h"

NS_LOG_COMPONENT_DEFINE ("PegasusSocket");

PegasusSocket::PegasusSocket() {
  NS_LOG_FUNCTION(this);
}

PegasusSocket::~PegasusSocket() {
  NS_LOG_FUNCTION(this);
}

void PegasusSocket::SetAttributes(int port, int peerPort, int virtualPeerPort, const Ptr<Node> & node) {
  NS_LOG_FUNCTION(this);
  m_port = port;
  m_peerPort = peerPort;
  m_virtualPeerPort = virtualPeerPort;
  m_node = node;
}
