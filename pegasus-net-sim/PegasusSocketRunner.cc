
#include "PegasusSocketRunner.h"
#include "PegasusVariables.h"
#include "PegasusSocket.h"
#include "NS3PegasusDroneApp.h"
#include "PegasusPacket.h"

#include <algorithm>
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/system-mutex.h"

NS_LOG_COMPONENT_DEFINE ("PegasusNS3SocketRunner");

PegasusVariables * PegasusSocketRunner::m_pegasusVars;

void PegasusSocketRunner::handleRead(int maxFd) {
  NS_LOG_FUNCTION(this);

  fd_set rset;
  struct timeval timeout = { 0 , 0 };
  auto pegasusSockets = &m_pegasusVars->m_pegasusSockets;

  FD_ZERO(&rset);
  // Set the fd_set

  for (auto const &psock: *pegasusSockets) {
    FD_SET(psock->Get_m_sd(), &rset);
  }

  NS_LOG_DEBUG("Waiting to read sockets...");

  auto nready = select(maxFd, &rset, NULL, NULL, &timeout);

  NS_LOG_DEBUG("Sockets ready for read: " << nready);

  for (auto const &psock: *pegasusSockets) {
    if (FD_ISSET(psock->Get_m_sd(), &rset)) {
      char buffer[MAX_PACKET_SIZE];
      struct sockaddr_in cliAddr;
      socklen_t addrLen;
      auto len = recvfrom(psock->Get_m_sd(), &buffer, sizeof(buffer), 0, 
          (struct sockaddr*)&cliAddr, &addrLen); 
      if (len > 0)
        SendSimulation(psock, buffer, len);
    }
  }
}

void PegasusSocketRunner::handleWrite(int maxFd) {
  NS_LOG_FUNCTION(this);

  auto pegasusSockets = &m_pegasusVars->m_pegasusSockets;

  for (auto const &psock: *pegasusSockets) {
    // The size of the deque should not increase, so the other thread better wait.
    PegasusPacket * packet = NULL;

    {
      CriticalSection(psock->m_txMutex); 
      if (!psock->m_packetTxQueue.empty()) {
        packet = psock->m_packetTxQueue.front();
        psock->m_packetTxQueue.pop_front();
      }
    }

    if (packet) {
      psock->Send(packet->Get_m_buffer(), packet->Get_m_len());
      delete packet;
    }
  }
}

void PegasusSocketRunner::SendSimulation(PegasusSocket* pegasusSocket, const char* buffer, const size_t & len)
{
  NS_LOG_FUNCTION(this);

  auto packet = new PegasusPacket(buffer, len);
  {
    CriticalSection(pegasusSocket->m_rxMutex);
    pegasusSocket->m_packetRxQueue.push_back(packet);
  }
}

PegasusSocketRunner::PegasusSocketRunner(){
  NS_LOG_FUNCTION(this);
}

PegasusSocketRunner::~PegasusSocketRunner(){
  NS_LOG_FUNCTION(this);
}

void PegasusSocketRunner::Set_m_pegasusVars(PegasusVariables * value)
{
  m_pegasusVars = value;
}

void PegasusSocketRunner::ExecutionLoop() {
  NS_LOG_FUNCTION(this);
  auto pegasusSockets = &m_pegasusVars->m_pegasusSockets;

  // Get max fd + 1
  auto maxFd = (*std::max_element(pegasusSockets->begin(), pegasusSockets->end(),
      [] (PegasusSocket* a, PegasusSocket* b) {
        return a->Get_m_sd() < b->Get_m_sd();
  }))->Get_m_sd() + 1;
  
  while(m_running) {
    int itr = 16;
    while(itr-- > 0)
      handleWrite(maxFd);
    
    itr = 1;
    while(itr-- > 0)
      handleRead(maxFd);
  }

}

void PegasusSocketRunner::Start() {
  NS_LOG_FUNCTION(this);
  m_running = true;
  m_st = Create<SystemThread> (MakeCallback (&PegasusSocketRunner::ExecutionLoop, this));
  m_st->Start();
}

void PegasusSocketRunner::Stop() {
  NS_LOG_FUNCTION(this);
  m_running = false;
}
