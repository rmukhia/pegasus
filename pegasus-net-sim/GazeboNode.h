#ifndef _GAZEBONODE_H
#define _GAZEBONODE_H


#include <string>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "ns3/system-thread.h"
#include "ns3/log.h"
#include "ns3/vector.h"

using namespace ns3;

class PegasusVariables;
class PegasusConfig;

//Gazebo Subscriber starts a new thread, so we don't need to explicitly do it.
class GazeboNode {
  private:
    static PegasusVariables * m_pegasusVars;

    std::string m_topic;

    gazebo::transport::NodePtr m_node;

    gazebo::transport::SubscriberPtr m_sub;


  public:
    GazeboNode();

    ~GazeboNode();

    static void MsgHandlerCb(const ConstPosesStampedPtr & msg);

    void Setup(int argc, char** argv);

    void SetTopic(const std::string & topic);

    void Subscribe();

    void Destroy();

    static void Set_m_pegasusVars(PegasusVariables * value);

};
#endif
