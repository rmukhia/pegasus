
#include "PegasusConfig.h"

PegasusPortConfig::PegasusPortConfig(int port, int peerPort, int virtualPeerPort) {
  m_port = port;
  m_peerPort = peerPort;
  m_virtualPeerPort = virtualPeerPort;
}

/* Define the test case here */
#define ONE_DRONE


/* Pick up one of the test cases from below */
#ifdef ONE_DRONE
std::map<std::string, std::vector<PegasusPortConfig>> PegasusConfig::m_config {
  {
    "iris", {
      {5444, 4444, 3444},
      {7300, 7400, 7200},
    }
  },
  {
    CONTROL_STATION_STR , {
      {3444, 6444, 5444},
      {7200, 8200, 7300},
    }
  },
};

#endif
#ifdef TWO_DRONE
std::map<std::string, std::vector<PegasusPortConfig>> PegasusConfig::m_config {
  {
    "iris_0", {
      {5444, 4444, 3444},
      {7300, 7400, 7200},
    }
  },
  {
    "iris_1", {
      {5445, 4445, 3445},
      {7301, 7401, 7201},
    }
  },
  {
    CONTROL_STATION_STR , {
      {3444, 6444, 5444},
      {7200, 8200, 7300},
      {3445, 6445, 5445},
      {7201, 8201, 7301},
    }
  },
};

#endif
#ifdef THREE_DRONE
std::map<std::string, std::vector<PegasusPortConfig>> PegasusConfig::m_config {
  {
    "iris_0", {
      {5444, 4444, 3444},
      {7300, 7400, 7200},
    }
  },
  {
    "iris_1", {
      {5445, 4445, 3445},
      {7301, 7401, 7201},
    }
  },
  {
    "iris_2", {
      {5446, 4446, 3446},
      {7302, 7402, 7202},
    }
  },
  {
    CONTROL_STATION_STR , {
      {3444, 6444, 5444},
      {7200, 8200, 7300},
      {3445, 6445, 5445},
      {7201, 8201, 7301},
      {3446, 6446, 5446},
      {7202, 8202, 7302},
    }
  },
};

#endif
