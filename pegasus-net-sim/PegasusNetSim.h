#ifndef PEGASUS_SIM_PEGASUSNETSIM_H
#define PEGASUS_SIM_PEGASUSNETSIM_H

#include "NS3Runner.h"
#include "GazeboNode.h"

class PegasusNetSim {
  private:
    static PegasusNetSim *instance;
    PegasusNetSim(){}; 
  public:
    static PegasusNetSim* Instance();
    ~PegasusNetSim();
    // const NS3Runner& GetNS3Runner ()  const { return ns3Runner; }
    // const GazeboNode& GetGazeboNode ()  const { return gazeboNode; }
    NS3Runner *ns3Runner;
    GazeboNode *gazeboNode;
};


#endif //PEGASUS_SIM_PEGASUSNETSIM_H
