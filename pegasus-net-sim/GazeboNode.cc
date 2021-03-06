
#include "GazeboNode.h"
#include "PegasusVariables.h"
#include "PegasusConfig.h"

NS_LOG_COMPONENT_DEFINE ("PegasusGazeboNode");
PegasusVariables * GazeboNode::m_pegasusVars;

GazeboNode::GazeboNode(){
  NS_LOG_FUNCTION(this);
}

GazeboNode::~GazeboNode(){
  NS_LOG_FUNCTION(this);
}

void GazeboNode::MsgHandlerCb(const ConstPosesStampedPtr & msg)
{
  // Dump the message contents to stdout.
  // std::cout << _msg->DebugString();

  for(auto const &pose: msg->pose()) {
    if (PegasusConfig::m_config.find(pose.name()) != PegasusConfig::m_config.end()) {
#if 0
      std::cout << "[" << pose.name() << "]" <<
        " x:" << pose.position().x() <<
        " y:" << pose.position().y() <<
        " z:" << pose.position().z() << std::endl;
#endif
      {
        CriticalSection(m_pegasusVars->m_poseMapMutex);
        m_pegasusVars->m_poseMap[pose.name()] = Vector(
            pose.position().x(),
            pose.position().y(),
            pose.position().z()
            );
      }
    }
  }
}

void GazeboNode::Setup(int argc, char** argv) {
  NS_LOG_FUNCTION(this);
  gazebo::client::setup(argc, argv);
  m_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  m_node->Init();
}

void GazeboNode::SetTopic(const std::string & topic) {
  NS_LOG_FUNCTION(this);
  m_topic.assign(topic);
}

void GazeboNode::Subscribe() {
  NS_LOG_FUNCTION(this);
  m_sub = m_node->Subscribe(m_topic, MsgHandlerCb);
}

void GazeboNode::Destroy() {
  NS_LOG_FUNCTION(this);
  gazebo::client::shutdown();
}

void GazeboNode::Set_m_pegasusVars(PegasusVariables * value) {
  m_pegasusVars = value;
}
