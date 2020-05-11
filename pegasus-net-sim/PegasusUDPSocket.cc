
#include "PegasusUDPSocket.h"

#include "ns3/log.h"
#include "ns3/names.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

NS_LOG_COMPONENT_DEFINE ("PegasusUDPSocket");

PegasusUDPSocket::PegasusUDPSocket(){
  NS_LOG_FUNCTION(this);
  m_type = PEGASUS_SOCKET_UDP;
}

PegasusUDPSocket::~PegasusUDPSocket(){
  NS_LOG_FUNCTION(this);
  close(m_sd);
}

void PegasusUDPSocket::Create() {
  NS_LOG_FUNCTION(this);
  m_sd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);

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
  serv.sin_port = htons(m_portConfig->Get_m_port());

  if (bind(m_sd, (struct sockaddr *)&serv, sizeof(serv)) < 0)
    NS_FATAL_ERROR("Bind Failed.");

  // Setup peer port
  memset(&m_peerAddr, 0, sizeof(m_peerAddr)); 
  m_peerAddr.sin_family = AF_INET;
  m_peerAddr.sin_port = htons(m_portConfig->Get_m_peerPort());
  // Fix the following for distrbuted simulation support
  m_peerAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

  NS_LOG_INFO(Names::FindName(m_node) << ": Bound proxy socket " << m_sd
      << " to port " << m_portConfig->Get_m_port() << " with peer port " << m_portConfig->Get_m_peerPort());
}

void PegasusUDPSocket::Send(const char* buffer, const unsigned int & len) {
  NS_LOG_FUNCTION(this);
  size_t ret = sendto(m_sd, buffer, len, 0, (sockaddr*)&m_peerAddr, sizeof(m_peerAddr));

  if (ret < 0)
    NS_LOG_ERROR(Names::FindName(m_node) << ": Cannot send real packet." << errno );
  else 
    NS_LOG_INFO(Names::FindName(m_node) << ": Sent real packet of size " << len << " in port " << m_portConfig->Get_m_peerPort());
}

