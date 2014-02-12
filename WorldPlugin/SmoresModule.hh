#ifndef _GAZEBO_SMORES_MODULE_HH_
#define _GAZEBO_SMORES_MODULE_HH_
// #include "SmoresEdge.hh"
#include "SmoresNode.hh"
#include "command_message.pb.h"
#include <string>
// #include <iostream>

using namespace std;
using namespace gazebo;

typedef boost::shared_ptr<command_message::msgs::CommandMessage> CommandPtr;
class ModuleCommands
{
public:
  ModuleCommands(SmoresModulePtr which_module);

public:
  SmoresModulePtr WhichModule;
  // The vector of command arrays
  vector<CommandPtr> CommandSquence;
  // The indicator of whether a command has been executing
  bool FinishedFlag;
  bool ReceivedFlag;
  int CurrentPriority;
};

typedef boost::shared_ptr<ModuleCommands> ModuleCommandsPtr;

class SmoresModule: public boost::enable_shared_from_this<SmoresModule>
{
public:
	SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, transport::PublisherPtr publisher, transport::SubscriberPtr subsciber, unsigned int num_ID);

	SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, transport::PublisherPtr publisher, transport::SubscriberPtr subsciber);

	SmoresModule(string mID, bool mtype, transport::PublisherPtr publisher, transport::SubscriberPtr subsciber, unsigned int num_ID);

	~SmoresModule();

	void ManuallyNodeInitial(SmoresModulePtr module_ptr);

	// Get the pointer to the node by node ID
	SmoresNodePtr GetNode(int node_id);

	int GetNodeAxis(int node_id);

	void SetModulePtr(physics::ModelPtr modelPtr);

	physics::LinkPtr GetLinkPtr(int node_id);


//++++++++++++++ Here comes model properties +++++++++++++++++++++
public: string ModuleID;
public: unsigned int ModuleNumID;	// Position in the vector
public: bool ModuleType; // Active module or Passive Module, true for active
public: physics::ModelPtr ModuleObject;	// A pointer to the real module
public: transport::PublisherPtr ModulePublisher;
public: transport::SubscriberPtr ModuleSubscriber;
public: ModuleCommandsPtr ModuleCommandContainer;
// Geometry information
public: math::Pose ModulePosition;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//-------------- Nodes -------------------------------------------
public:
	SmoresNode NodeUH; 	// The Ushape part
	SmoresNode NodeFW;	// The Front Wheel
	SmoresNode NodeLW;
	SmoresNode NodeRW;

	SmoresNodePtr NodeUHPtr;
	SmoresNodePtr NodeFWPtr;
	SmoresNodePtr NodeLWPtr;
	SmoresNodePtr NodeRWPtr;
//---------------------------------------------------------------- 
};

#endif