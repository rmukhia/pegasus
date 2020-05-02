//
// Created by rmukhia on 2/5/63.
//

#include "GazeboNode.h"

void cb(ConstPosesStampedPtr &_msg)
{
  // Dump the message contents to stdout.
  std::cout << _msg->DebugString();
}


void GazeboNode::setup(int argc, char **argv)
{
  gazebo::client::setup(argc, argv);
  this->node =  gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node->Init();
}

void GazeboNode::setTopic(const std::string &topic)
{
  this->topic.assign(topic);
}

void GazeboNode::subscribe()
{
  this->sub = node->Subscribe(this->topic, cb);
}

void GazeboNode::destroy()
{
  gazebo::client::shutdown();
}
