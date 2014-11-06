//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: This class is built for configuration representation.
//              An node in smores robot is a connectable face, and a 
//              minimal unit in graph structure representation.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_SMORES_NODE_HH_
#define _GAZEBO_SMORES_NODE_HH_
//! \file Define SmoresNode class used in WorldPlugin
#include "SmoresEdge.hh"

namespace gazebo{
class SmoresModule;
typedef boost::shared_ptr<SmoresEdge> SmoresEdgePtr;
typedef boost::shared_ptr<SmoresModule> SmoresModulePtr;
/// A class used to manage each node, in this case is the part has connectable face
class SmoresNode
{
 public:
  /// Constructor
  SmoresNode();
  /// Constructor
  /*!
    \param nodeID Id number for the current node
    \param jtype Joint type of the current node, relative to module
    \param jvalue Current joint angle
    \param parent Pointer to its parent SmoresModule object
    \param edge Pointer to the SmoresEdge object
  */
  SmoresNode(int nodeID, int jtype, int jvalue, SmoresModulePtr parent,
      SmoresEdgePtr edge);
  /// Constructor
  /*!
    \param nodeID Id number for the current node
    \param jtype Joint type of the current node, relative to module
    \param jvalue Current joint angle
    \param parent Pointer to its parent SmoresModule object
  */
  SmoresNode(int nodeID, int jtype, int jvalue, SmoresModulePtr parent);
  /// Constructor
  /*!
    \param nodeID Id number for the current node
    \param jtype Joint type of the current node, relative to module
    \param jvalue Current joint angle
  */
  SmoresNode(int nodeID, int jtype, int jvalue);
  /// Destructor
  ~SmoresNode();
  /// Node initilization functions
  /*!
    \param nodeID Id number for the current node
    \param jtype Joint type of the current node, relative to module
    \param jvalue Current joint angle
    \param parent Pointer to its parent SmoresModule object
    \param edge Pointer to the SmoresEdge object
  */
  void NodeInit(int nodeID, int jtype, int jvalue, SmoresModulePtr parent,
      SmoresEdgePtr edge);
  /// Node initilization functions
  /*!
    \param nodeID Id number for the current node
    \param jtype Joint type of the current node, relative to module
    \param jvalue Current joint angle
    \param parent Pointer to its parent SmoresModule object
  */
  void NodeInit(int nodeID, int jtype, int jvalue, SmoresModulePtr parent);
  /// Set the parent object
  /*!
    \param mparent Pointer to its parent, which is a SmoresModule object
  */
  void SetParent(SmoresModulePtr mparent);
  /// Get axis of the norm vector of connectable face
  /*!
    \return Axis id
  */
  int GetAxis(void);
  /// Connect the current node to one edge
  /*!
    \edge The edge that current node connect to
  */
  void ConnectOnEdge(SmoresEdgePtr edge);
  /// Disconnect the current node from any edge
  void Disconnect(void);
  /// Get the edge that current node connects to
  /*!
    \return SmoresEdgePtr The edge this node connected to or empty pointer
  */
  SmoresEdgePtr GetEdge(void);
  /// Node ID specify which node the current node is, 0 is the base node
  int NodeID;
  /// 0 for revolute or 1 for prismatic
  int JointType;
  /// Revolute joint: angle value; prismatic joint: distance value
  int JointValue;
  /// Position of the current node relative to base node
  math::Vector3 RelativePosition;
  /// A pointer to its parent
  SmoresModulePtr Parent;
  /// A pointer to the existing edge
  SmoresEdgePtr Edge;
};
}
#endif