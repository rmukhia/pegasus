//
// Created by rmukhia on 2/5/63.
//

#ifndef PEGASUS_SIM_GAZEBONODE_H
#define PEGASUS_SIM_GAZEBONODE_H

#include <string>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>


class GazeboNode {
  private:
    std::string topic;
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr sub;
  public:
    void setup(int argc, char **argv);
    void setTopic(const std::string& topic);
    void subscribe();
    void destroy();
};


#endif //PEGASUS_SIM_GAZEBONODE_H
