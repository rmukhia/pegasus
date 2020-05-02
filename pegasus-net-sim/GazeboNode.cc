//
// Created by rmukhia on 2/5/63.
//

#include "GazeboNode.h"
#include "ns3/simulator.h"

#include <cstring>


std::vector<std::string> GazeboNode::modelsName;
std::unordered_map<std::string, ns3::Vector> GazeboNode::poseMap;

void GazeboNode::_subscribe()
{
  this->sub = node->Subscribe(this->topic, this->MsgHandlerCb);
}

void GazeboNode::_parsePoseStampedMsg(const gazebo::msgs::Pose& pose)
{

  if (std::find(modelsName.begin(), modelsName.end(), pose.name())
        != modelsName.end()) {
    std::cout << "[" << pose.name() << "]" <<
      " x:" << pose.position().x() <<
      " y:" << pose.position().y() <<
      " z:" << pose.position().z() << std::endl;
    poseMap[pose.name()] = ns3::Vector(pose.position().x(), pose.position().y(), pose.position().z());
    // callback(pose.name(), poseMap[pose.name()]);
  }
}

GazeboNode::GazeboNode()
{
  //poseMap = new std::unordered_map<std::string, int>();
}

void GazeboNode::MsgHandlerCb(ConstPosesStampedPtr& _msg)
{
  // Dump the message contents to stdout.
  // std::cout << _msg->DebugString();
  for_each(_msg->pose().begin(), _msg->pose().end(), _parsePoseStampedMsg);
}

void GazeboNode::SetModelsName(const std::vector<std::string>& models)
{
  modelsName = models;
}

void GazeboNode::Setup(int argc, char **argv)
{
  gazebo::client::setup(argc, argv);
  this->node =  gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
}

void GazeboNode::SetTopic(const std::string &topic)
{
  this->topic.assign(topic);
}

void GazeboNode::Subscribe()
{
  this->st = ns3::Create<ns3::SystemThread>(ns3::MakeCallback(&GazeboNode::_subscribe, this));
  this->st->Start();
}

void GazeboNode::Destroy()
{
  gazebo::client::shutdown();
}
