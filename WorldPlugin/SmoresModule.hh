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
//! \file Defines SmoresModule class used in WorldPlugin
#include <string>
#include <iostream>

#include "SmoresEdge.hh"
#include "SmoresNode.hh"
#include "CommandManagement.hh"

namespace gazebo{
/// Smart pointer to the ModuleCommands object
typedef boost::shared_ptr<ModuleCommands> ModuleCommandsPtr;
/// A class used to store a cluster of nodes which has rigid connectivity
/*!
  This is part of the graph representation of configuration that used in 
  multiple research in modlab. 
*/
class SmoresModule
{
 public:
  /// Constructor
  /*!
    \param mID String ID of the module
    \param mtype Type of the module, true for active module and false for passive
    \param modulePtr smart pointer to the gazebo model object
    \param publisher Command publisher of the managed model
    \param subsciber Command subscriber of the managed model
    \param num_ID Position in the management vector
  */
  SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, 
      transport::PublisherPtr publisher, transport::SubscriberPtr subsciber, 
      unsigned int num_ID);
  /// Constructor
  /*!
    \param mID String ID of the module
    \param mtype Type of the module, true for active module and false for passive
    \param modulePtr smart pointer to the gazebo model object
    \param publisher Command publisher of the managed model
    \param subsciber Command subscriber of the managed model
  */
  SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, 
      transport::PublisherPtr publisher, transport::SubscriberPtr subsciber);
  /// Constructor
  /*!
    \param mID String ID of the module
    \param mtype Type of the module, true for active module and false for passive
    \param publisher Command publisher of the managed model
    \param subsciber Command subscriber of the managed model
    \param num_ID Position in the management vector
  */
  SmoresModule(string mID, bool mtype, transport::PublisherPtr publisher, 
      transport::SubscriberPtr subsciber, unsigned int num_ID);
  /// Destructor
  ~SmoresModule();
  /// Manually pass the pointer to the current object to its nodes
  /*!
    \param module_ptr The smart pointer of the current SmoresModule object
  */
  void ManuallyNodeInitial(SmoresModulePtr module_ptr);
  /// Get the pointer to the node by node ID
  /*!
    \param node_id Number id of node, 
                    0 for front
                    1 for left
                    2 for right
                    3 for square
    \return A smart pointer of the node object 
  */
  SmoresNodePtr GetNode(int node_id);
  /// Get node axis id, the axis is perpendicular to the connected face
  /*!
    \param node_id Number id of node, see GetNode()
    \return A number indicate the axis in the face local frame
  */
  int GetNodeAxis(int node_id);
  /// Pass the pointer to the gazebo model object to current object
  /*!
    \param modelPtr Smart pointer to gazebo Model object
  */
  void SetModulePtr(physics::ModelPtr modelPtr);
  /// Get the pointer to the gazebo Link object
  /*!
    \param node_id Number ID of node, see GetNode()
    \return Smart pointer to the gazebo Link object
  */
  physics::LinkPtr GetLinkPtr(int node_id);
  /// Unique name for the current module
  string ModuleID;
  /// Position in the vector
  unsigned int ModuleNumID;
  /// Active module or Passive Module, true for active
  bool ModuleType;
  /// A pointer of the gazebo model object for fast access
  physics::ModelPtr ModuleObject;
  /// Pointer to the command publisher
  transport::PublisherPtr ModulePublisher;
  /// Pointer to the command subscriber
  transport::SubscriberPtr ModuleSubscriber;
  /// Poointer to the command management object of current module
  ModuleCommandsPtr moduleCommandContainer;
  /// Position of the current module [Not implemented]
  // TODO: Need to add geometry information
  math::Pose ModulePosition;
  /// Node of the part that has the square face [redundant]
  SmoresNode NodeUH;  // The U-shape part
  /// Node of the front wheel [redundant]
  SmoresNode NodeFW;  // The Front Wheel
  /// Node of the left wheel [redundant]
  SmoresNode NodeLW;
  /// Node of the right wheel [redundant]
  SmoresNode NodeRW;
  /// Smart pointer of the node of the part that has the square face
  SmoresNodePtr NodeUHPtr;
  /// Smart pointer of the node of the front wheel
  SmoresNodePtr NodeFWPtr;
  /// Smart pointer of the node of the left wheel
  SmoresNodePtr NodeLWPtr;
  /// Smart pointer of the node of the right wheel
  SmoresNodePtr NodeRWPtr;
}; // class SmoresModule
} // namespace gazebo
#endif