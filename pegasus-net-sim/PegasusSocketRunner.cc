
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
  //struct timeval timeout = { 0 , 0 };
  auto pegasusSockets = &m_pegasusVars->m_pegasusSockets;

  FD_ZERO(&rset);
  // Set the fd_set

  for (auto const &psock: *pegasusSockets) {
    FD_SET(psock->Get_m_sd(), &rset);
  }

  NS_LOG_DEBUG("Waiting to read sockets...");

  //auto nready = select(maxFd, &rset, NULL, NULL, &timeout);
  auto nready = select(maxFd, &rset, NULL, NULL, NULL);

  if(nready <=0) return;

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
    PegasusPacket packet;
    if (psock->m_packetTxQueue.pop(packet))
      psock->Send(packet.Get_m_buffer(), packet.Get_m_len());
  }
}

void PegasusSocketRunner::SendSimulation(PegasusSocket* pegasusSocket, const char* buffer, const size_t & len)
{
  NS_LOG_FUNCTION(this);
  static uint32_t errorPackets = 0;

  PegasusPacket packet(buffer, len);
  if(!pegasusSocket->m_packetRxQueue.push(packet)) {
    // Rate limited error
    if (++errorPackets > 1024) {
      NS_LOG_INFO("RXQUEUE full.");
      errorPackets = 0;
    }
  }
}

void PegasusSocketRunner::Read() {
  NS_LOG_FUNCTION(this);
  auto pegasusSockets = &m_pegasusVars->m_pegasusSockets;

  // Get max fd + 1
  auto maxFd = (*std::max_element(pegasusSockets->begin(), pegasusSockets->end(),
      [] (PegasusSocket* a, PegasusSocket* b) {
        return a->Get_m_sd() < b->Get_m_sd();
  }))->Get_m_sd() + 1;

  struct timespec req = { 0, 10 };
  while(m_running) {
      handleRead(maxFd);
      nanosleep(&req, NULL);
  }
}

void PegasusSocketRunner::Write() {
  NS_LOG_FUNCTION(this);
  struct timespec req = { 0, 10 };
  while(m_running) {
    handleWrite(0);
    nanosleep(&req, NULL);
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
}

void PegasusSocketRunner::Start() {
  NS_LOG_FUNCTION(this);
  m_running = true;
  m_stRead = Create<SystemThread> (MakeCallback (&PegasusSocketRunner::Read, this));
  m_stRead->Start();

  m_stWrite = Create<SystemThread> (MakeCallback (&PegasusSocketRunner::Write, this));
  m_stWrite->Start();
}

void PegasusSocketRunner::Stop() {
  NS_LOG_FUNCTION(this);
  m_running = false;
}
