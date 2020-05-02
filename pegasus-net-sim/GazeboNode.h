//
// Created by rmukhia on 2/5/63.
//

#ifndef PEGASUS_SIM_GAZEBONODE_H
#define PEGASUS_SIM_GAZEBONODE_H

#include <string>
#include <vector>
#include <unordered_map>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "ns3/system-thread.h"

#include "Drone.h"

class GazeboNode {
  private:
    std::string topic;
    ns3::Ptr<ns3::SystemThread> st;
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr sub;
    static std::vector<std::string> modelsName;
    void _subscribe();
    static void _parsePoseStampedMsg(const gazebo::msgs::Pose& pose);
  public:
    GazeboNode();
    static void MsgHandlerCb(ConstPosesStampedPtr& _msg);
    void SetModelsName(const std::vector<std::string>& models);
    void Setup(int argc, char **argv);
    void SetTopic(const std::string& topic);
    void Subscribe();
    void Destroy();
    static std::unordered_map<std::string, Drone> poseMap;
};


#endif //PEGASUS_SIM_GAZEBONODE_H
