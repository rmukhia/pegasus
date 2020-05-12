
#include "PegasusConfig.h"

PegasusPortConfig::PegasusPortConfig(int port, int peerPort, int virtualPeerPort) {
  m_port = port;
  m_peerPort = peerPort;
  m_virtualPeerPort = virtualPeerPort;
}

std::map<std::string, std::vector<PegasusPortConfig>> PegasusConfig::m_config {
  {
    "iris_0", {
      {14540, 14580, 14680},
    }
  },
  {
    "iris_1", {
      {14541, 14581, 14681},
    }
  },
  {
    "iris_2", {
      {14542, 14582, 14682},
    }
  },
  {
    CONTROL_STATION_STR , {
      {14680, 14640, 14540},
      {14681, 14641, 14541},
      {14682, 14642, 14542},
    }
  },
};

