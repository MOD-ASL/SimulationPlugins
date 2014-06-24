//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: This class is built for configuration representation.
//              An node in smores robot is a connectable face, and a 
//              minimal unit in graph structure representation.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_SMORES_NODE_HH_
#define _GAZEBO_SMORES_NODE_HH_

#include "SmoresEdge.hh"

namespace gazebo{
class SmoresModule;
typedef boost::shared_ptr<SmoresEdge> SmoresEdgePtr;
typedef boost::shared_ptr<SmoresModule> SmoresModulePtr;
class SmoresNode
{
 public:
  SmoresNode();
  SmoresNode(int nodeID, int jtype, int jvalue, SmoresModulePtr parent,
      SmoresEdgePtr edge);
  SmoresNode(int nodeID, int jtype, int jvalue, SmoresModulePtr parent);
  SmoresNode(int nodeID, int jtype, int jvalue);
  ~SmoresNode();
  /// Node initilization functions
  void NodeInit(int nodeID, int jtype, int jvalue, SmoresModulePtr parent,
      SmoresEdgePtr edge);
  void NodeInit(int nodeID, int jtype, int jvalue, SmoresModulePtr parent);
  void SetParent(SmoresModulePtr mparent);
  /// Utility functions
  int GetAxis(void);
  void ConnectOnEdge(SmoresEdgePtr edge);
  void Disconnect(void);
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