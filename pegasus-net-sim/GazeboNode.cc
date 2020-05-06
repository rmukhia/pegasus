//
// Created by rmukhia on 2/5/63.
//

#include "GazeboNode.h"
#include <cstring>
#include "ns3/simulator.h"


NS_LOG_COMPONENT_DEFINE ("PegasusGazeboNode");

std::vector<std::string> GazeboNode::modelsName;
std::unordered_map<std::string, Vector> GazeboNode::poseMap;

void GazeboNode::_subscribe()
{
  NS_LOG_FUNCTION(this);
  sub = node->Subscribe(this->topic, this->MsgHandlerCb);
}

GazeboNode::GazeboNode()
{
  NS_LOG_FUNCTION(this);
  //poseMap = new std::unordered_map<std::string, int>();
}

void GazeboNode::MsgHandlerCb(ConstPosesStampedPtr& _msg)
{
  NS_LOG_FUNCTION(&GazeboNode::MsgHandlerCb);
  // Dump the message contents to stdout.
  // std::cout << _msg->DebugString();
  std::for_each(_msg->pose().begin(), _msg->pose().end(), 
    [] (const gazebo::msgs::Pose& pose) {
      if (std::find(modelsName.begin(), modelsName.end(), pose.name())
          != modelsName.end()) {
#if 0
        std::cout << "[" << pose.name() << "]" <<
        " x:" << pose.position().x() <<
        " y:" << pose.position().y() <<
        " z:" << pose.position().z() << std::endl;
#endif
        poseMap[pose.name()] = Vector(
            pose.position().x(),
            pose.position().y(),
            pose.position().z()
          );
      }
    });
}

void GazeboNode::SetModelsName(const std::vector<std::string>& models)
{
  NS_LOG_FUNCTION(this);
  modelsName = models;
}

void GazeboNode::Setup(int argc, char **argv)
{
  NS_LOG_FUNCTION(this);
  gazebo::client::setup(argc, argv);
  node =  gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
}

void GazeboNode::SetTopic(const std::string &topic)
{
  NS_LOG_FUNCTION(this);
  this->topic.assign(topic);
}

void GazeboNode::Subscribe()
{
  NS_LOG_FUNCTION(this);
  st = Create<SystemThread>(MakeCallback(&GazeboNode::_subscribe, this));
  st->Start();
}

void GazeboNode::Destroy()
{
  NS_LOG_FUNCTION(this);
  gazebo::client::shutdown();
}
