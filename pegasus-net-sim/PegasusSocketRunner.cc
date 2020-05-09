
#include "PegasusSocketRunner.h"
#include "PegasusVariables.h"
#include "PegasusSocket.h"
#include "NS3PegasusDroneApp.h"

#include <algorithm>
#include "ns3/log.h"
#include "ns3/names.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNS3SocketRunner");

PegasusVariables * PegasusSocketRunner::m_pegasusVars;

void PegasusSocketRunner::SendUDPSimulation(const PegasusSocket* pegasusSocket, const char* buffer, const size_t & len) {
  NS_LOG_FUNCTION(this);
  Ptr<Node> node = pegasusSocket->Get_m_node();
  auto ns3PegasusDroneApps = &m_pegasusVars->m_ns3PegasusDroneApps;
  auto appItr = std::find_if(ns3PegasusDroneApps->begin(), ns3PegasusDroneApps->end(),
      [&node] (Ptr<NS3PegasusDroneApp> app) {
        return app->GetNode() == node;
      });

  if (appItr == ns3PegasusDroneApps->end()) {
    NS_LOG_ERROR("Invalid app for pegasusSocket " << pegasusSocket->Get_m_port()
        << "--" << pegasusSocket->Get_m_virtualPeerPort());
  } else {
    (*appItr)->ScheduleSend(pegasusSocket->Get_m_port(), pegasusSocket->Get_m_virtualPeerPort(), buffer, len);
    NS_LOG_INFO(Names::FindName(node) << ": Reveived real packet of size " << len
        << " in port " << pegasusSocket->Get_m_port() << " and relayed to virtual peer port "
        << pegasusSocket->Get_m_virtualPeerPort());
  }
}

void PegasusSocketRunner::SendTCPSimulation(const PegasusSocket* pegasusSocket, const char* buffer, const size_t & len) {
  NS_LOG_FUNCTION(this);
}

PegasusSocketRunner::PegasusSocketRunner(){
  NS_LOG_FUNCTION(this);
}

PegasusSocketRunner::~PegasusSocketRunner(){
  NS_LOG_FUNCTION(this);
}

void PegasusSocketRunner::Read() {
  NS_LOG_FUNCTION(this);
  fd_set rset;
  auto pegasusSockets = &m_pegasusVars->m_pegasusSockets;

  // Get max fd + 1
  int maxFd = (*std::max_element(pegasusSockets->begin(), pegasusSockets->end(),
      [] (PegasusSocket* a, PegasusSocket* b) {
        return a->Get_m_sd() < b->Get_m_sd();
  }))->Get_m_sd() + 1;


  while(m_running) {
    FD_ZERO(&rset);
    // Set the fd_set
    std::for_each(pegasusSockets->begin(), pegasusSockets->end(),
        [&rset] (PegasusSocket *psock) {
          FD_SET(psock->Get_m_sd(), &rset);
        });


    NS_LOG_DEBUG("Waiting to read sockets...");

    int nready = select(maxFd, &rset, NULL, NULL, NULL);

    NS_LOG_DEBUG("Sockets ready for read: " << nready);

    std::for_each(pegasusSockets->begin(), pegasusSockets->end(), 
        [this, &rset] (PegasusSocket *psock) {
          if (FD_ISSET(psock->Get_m_sd(), &rset)) {
            char buffer[9000];
            struct sockaddr_in cliAddr;
            socklen_t addrLen;
            unsigned int len = recvfrom(psock->Get_m_sd(), &buffer, sizeof(buffer), 0, 
                         (struct sockaddr*)&cliAddr, &addrLen); 
            SendSimulation(psock, buffer, len);
          }
        });
  }
}

void PegasusSocketRunner::SendSimulation(const PegasusSocket* pegasusSocket, const char* buffer, const size_t & len)
{
  NS_LOG_FUNCTION(this);
  if (pegasusSocket->Get_m_type() == PegasusSocket::PEGASUS_SOCKET_UDP) {
    SendUDPSimulation(pegasusSocket, buffer, len);
  }
  //... else  PEGASUS_SOCKET_TCP....
}

void PegasusSocketRunner::Set_m_pegasusVars(PegasusVariables * value)
{
  m_pegasusVars = value;
}

void PegasusSocketRunner::Start() {
  NS_LOG_FUNCTION(this);
  m_running = true;
  m_st = Create<SystemThread> (MakeCallback (&PegasusSocketRunner::Read, this));
  m_st->Start();
}

void PegasusSocketRunner::Stop() {
  NS_LOG_FUNCTION(this);
  m_running = false;
}
