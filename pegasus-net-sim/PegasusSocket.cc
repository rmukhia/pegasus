
#include "PegasusSocket.h"

PegasusSocket::PegasusSocket() {
}

PegasusSocket::~PegasusSocket() {
}

void PegasusSocket::SetAttributes(int port, int peerPort, const Ptr<Node> & node) {
  m_port = port;
  m_peerPort = peerPort;
  m_node = node;
}
