#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <map>
#include <stdexcept>
#include <cstdlib>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Float64.h"

#define MAX_PACKET_SIZE 64

typedef std::map<std::string, ros::Publisher> PublisherMap;

int CreateStatusSocket(const int& port) {
  int sd;
  int enable = 1;
  struct sockaddr_in serv;
  struct timeval tv = { 1, 0 };

  if ((sd = socket(AF_INET, SOCK_DGRAM, 0)) < 1) {
    ROS_FATAL("failed to create socket.");
    return -1;
  }


  if(setsockopt(sd, SOL_SOCKET, SO_REUSEADDR,  &enable, sizeof(int)) < 0) {
    ROS_FATAL("setsockopt(SO_REUSEADDR) failed");
    return -1;
  }

  if(setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO,  (const char *)&tv, sizeof(tv)) < 0) {
    ROS_FATAL("setsockopt(SO_RCVTIMEO) failed");
    return -1;
  }
  serv.sin_family = AF_INET;
  serv.sin_addr.s_addr = htonl(INADDR_ANY);
  serv.sin_port = htons(port);
  if (bind(sd, (struct sockaddr *)&serv, sizeof(serv)) < 0) {
    ROS_FATAL("bind failed.");
    return -1;
  }

  return sd;
}

ros::Publisher GetPublisher(const std::string& device, ros::NodeHandle& n, PublisherMap& pubs) {
  if (pubs.find(device) == pubs.end()) {
    std::ostringstream oss;
    oss << "/pegasus_status/snr/" << device;
    pubs[device] = n.advertise<std_msgs::Float64>(oss.str(), 1000, false);
  }

  return pubs[device];
}

std::vector<std::string> ParseStatus(const std::string& str) {
  std::vector<std::string> tokens;
  std::size_t cur, prev = 0;

  while((cur = str.find('/', prev)) != std::string::npos) {
    tokens.push_back(str.substr(prev, cur - prev));
    prev = cur + 1;
  }

  tokens.push_back(str.substr(prev, cur - prev));
  if (tokens.size() > 0)
    tokens.erase(tokens.begin());

  return tokens;
}

void ProcessStatus(const std::string& status, ros::NodeHandle& n, PublisherMap& pubs) {
  std::vector<std::string> tokens = ParseStatus(status);
  std::ostringstream oss;

  oss << "Invalid status message " << status;

  if (tokens.size() != 3) {
    ROS_ERROR_STREAM(oss.str());
    throw std::invalid_argument(oss.str());
  }

  std::string device =  tokens[0];
  char *ptr;

  double signal = std::strtod(tokens[1].c_str(), &ptr);
  if (!ptr)
    throw std::invalid_argument(oss.str());

  double noise = std::strtod(tokens[2].c_str(), &ptr);
  if (!ptr)
    throw std::invalid_argument(oss.str());

  double snr = signal - noise;

  ros::Publisher pub = GetPublisher(device, n, pubs);
  std_msgs::Float64 msg;
  msg.data = snr;

  pub.publish(msg);
}



int main(int argc, char **argv) {
  PublisherMap pubs;
  int sd, len;
  char c_buf[MAX_PACKET_SIZE];

  ros::init(argc, argv, "pegasus_network_status");
  ros::NodeHandle n;

  if ((sd = CreateStatusSocket(2999)) < 1) {
    ROS_FATAL("CreateStatusSocket failed.");
    ros::shutdown();
    exit(1);
  }

  while(ros::ok())
  {
    std::string buf;
    struct sockaddr_in cliAddr;
    socklen_t addrLen;
    if ((len = recv(sd, &c_buf, sizeof(c_buf), 0)) > 0) {
        buf.append(c_buf, len);
        try {
          ProcessStatus(buf, n, pubs);
        }
        catch(std::exception &e) {
          // ROS_ERROR(e.what());
        }
    }

    ros::spinOnce();
  }

  return 0;
}
