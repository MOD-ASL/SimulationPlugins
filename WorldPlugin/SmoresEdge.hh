//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: This class is built for configuration representation.
//              And edge connects two nodes in different modules, and also
//              it will store a pointer to the dynamically generated
//              joint.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_SMORES_EDGE_HH_
#define _GAZEBO_SMORES_EDGE_HH_

#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// #include "SmoresNode.hh"

namespace gazebo{
class  SmoresNode;
typedef boost::shared_ptr<SmoresNode> SmoresNodePtr;
class SmoresEdge
{
 public:
  SmoresEdge(SmoresNodePtr model_first, SmoresNodePtr model_second,
      double length, double angle,int axis1, int axis2);
  ~SmoresEdge();
  SmoresNodePtr FindMatchingNode(SmoresNodePtr node);
  /// Distance offset along the aligned axis
  double Distance;
  /// Angle offset in radian around the aligned axis
  double Angle;
  /// Axis of the model_1, 0 for x, 1 for y, 2 for z
  int Axis_1;
  /// Axis of the model_2, 0 for x, 1 for y, 2 for z
  int Axis_2;

  /// A pointer to the first node on the first module
  SmoresNodePtr model_1;
  /// A pointer to the second node on the second module
  SmoresNodePtr model_2;
  /// A pointer to the dynamically generated joint
  physics::JointPtr DynamicJointPtr;
}; // class SmoresEdge
}
#endif