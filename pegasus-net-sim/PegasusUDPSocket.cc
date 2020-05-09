
#include "PegasusUDPSocket.h"

#include "ns3/log.h"
#include "ns3/names.h"
#include <sys/socket.h>
#include <netinet/in.h>

NS_LOG_COMPONENT_DEFINE ("PegasusUDPSocket");

PegasusUDPSocket::PegasusUDPSocket(){
  NS_LOG_FUNCTION(this);
}

PegasusUDPSocket::~PegasusUDPSocket(){
  NS_LOG_FUNCTION(this);
}

void PegasusUDPSocket::Create() {
  NS_LOG_FUNCTION(this);
  m_sd = socket(AF_INET, SOCK_DGRAM, 0);

  int enable = 1;
  if(setsockopt(m_sd, SOL_SOCKET, SO_REUSEADDR,  &enable, sizeof(int)) < 0)
    NS_FATAL_ERROR("setsockopt(SO_REUSEADDR) failed");

  NS_LOG_INFO(Names::FindName(m_node) << ": Created proxy socket " << m_sd);
}

void PegasusUDPSocket::Bind() {
  NS_LOG_FUNCTION(this);
  struct sockaddr_in serv;
  serv.sin_family = AF_INET;
  serv.sin_addr.s_addr = htonl(INADDR_ANY);
  serv.sin_port = htons(m_port);

  if (bind(m_sd, (struct sockaddr *)&serv, sizeof(serv)) < 0)
    NS_FATAL_ERROR("Bind Failed.");

  NS_LOG_INFO(Names::FindName(m_node) << ": Bound proxy socket " << m_sd
      << " to port " << m_port << " with peer port " << m_peerPort);
}

void PegasusUDPSocket::Send(const char* buffer, const unsigned int & len) {
  NS_LOG_FUNCTION(this);
  NS_LOG_INFO(Names::FindName(m_node) << ": Sending real packet of size " << len);
}

