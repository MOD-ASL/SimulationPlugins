//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Filename:  ModelController.hh
// Author:  Edward Cui
// Contact: cyk1990995@gmail.com
// Last updated:  11/24/2013
// Description: 
// Commit info: git@github.com:princeedward/SimulationPlugins.git
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "ModelController.hh"

using namespace gazebo;
using namespace std;

typedef const boost::shared_ptr<const msgs::GzString> GzStringPtr;
typedef const boost::shared_ptr<const msgs::Pose> PosePtr;
typedef const boost::shared_ptr<const command_message::msgs::CommandMessage> CommandMessagePtr;



GZ_REGISTER_MODEL_PLUGIN(ModelController)