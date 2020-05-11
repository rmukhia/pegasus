
#include "PegasusConfig.h"

PegasusPortConfig::PegasusPortConfig(int port, int peerPort, int virtualPeerPort) {
  m_port = port;
  m_peerPort = peerPort;
  m_virtualPeerPort = virtualPeerPort;
}

std::map<std::string, std::vector<PegasusPortConfig>> PegasusConfig::m_config {
  {
    "iris", {
      {14540, 14580, 14581},
      {14550, 18570, 18751},
    }
  },
    /*
  {
    "iris_2", {
      {5560, 4450, 5550},
      {5561, 4451, 5570},
    }
  },
  */
  {
    CONTROL_STATION_STR , {
      {14581, 14541, 14540},
      {18751, 14551, 14550},
    }
  },
};

