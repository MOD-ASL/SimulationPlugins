//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: This class is built for configuration representation.
//              This class combines with 'SmoresEdge' and 'SmoresNode'
//              can represent very complicated graph structure.
//              A module has four nodes, and there geometry relations
//              are predetermined. This class provides some utility
//              functions to access the nodes and edges in the configuration
//              easily.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_SMORES_MODULE_HH_
#define _GAZEBO_SMORES_MODULE_HH_

#include <string>
#include <iostream>

#include "SmoresEdge.hh"
#include "SmoresNode.hh"
#include "CommandManagement.hh"

namespace gazebo{

typedef boost::shared_ptr<ModuleCommands> ModuleCommandsPtr;

class SmoresModule
{
 public:
  SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, 
      transport::PublisherPtr publisher, transport::SubscriberPtr subsciber, 
      unsigned int num_ID);
  SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, 
      transport::PublisherPtr publisher, transport::SubscriberPtr subsciber);
  SmoresModule(string mID, bool mtype, transport::PublisherPtr publisher, 
      transport::SubscriberPtr subsciber, unsigned int num_ID);
  ~SmoresModule();
  void ManuallyNodeInitial(SmoresModulePtr module_ptr);
  /// Get the pointer to the node by node ID
  SmoresNodePtr GetNode(int node_id);
  /// Return node axis id
  int GetNodeAxis(int node_id);
  /// Set module pointer
  void SetModulePtr(physics::ModelPtr modelPtr);
  physics::LinkPtr GetLinkPtr(int node_id);
  string ModuleID;
  unsigned int ModuleNumID; // Position in the vector
  bool ModuleType; // Active module or Passive Module, true for active
  physics::ModelPtr ModuleObject; // A pointer to the real module
  transport::PublisherPtr ModulePublisher;
  transport::SubscriberPtr ModuleSubscriber;
  ModuleCommandsPtr moduleCommandContainer;
  /// TODO: Need to add geometry information
  math::Pose ModulePosition;
  SmoresNode NodeUH;  // The U-shape part
  SmoresNode NodeFW;  // The Front Wheel
  SmoresNode NodeLW;
  SmoresNode NodeRW;

  SmoresNodePtr NodeUHPtr;
  SmoresNodePtr NodeFWPtr;
  SmoresNodePtr NodeLWPtr;
  SmoresNodePtr NodeRWPtr;
}; // class SmoresModule
} // namespace gazebo
#endif