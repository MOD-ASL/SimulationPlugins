//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: This class is built for configuration representation.
//              And edge connects two nodes in different modules, and also
//              it will store a pointer to the dynamically generated
//              joint.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_SMORES_EDGE_HH_
#define _GAZEBO_SMORES_EDGE_HH_
//! \file Defines SmoresEdge class used in WordPlugin
#include <boost/shared_ptr.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// #include "SmoresNode.hh"

namespace gazebo{
// Forward declaration
class  SmoresNode;
/// Smart pointer of SmoresNode object
typedef boost::shared_ptr<SmoresNode> SmoresNodePtr;
/// A class used to represent connectivities in simulation
/*!
  This is part of the graph representation of configuration that used in 
  multiple research in modlab. 
*/
class SmoresEdge
{
 public:
  /// Constructor
  /*!
    \param model_first Pointer to one of the connected modules
    \param model_second Pointer to the other connected modules
    \param length The offset distance of the two nodes
    \param angle The offset angle of the two nodes along the connected axis
    \param axis1 Axis id of the connected axis in the node local frame, 
                  0 for x, 1 for y, 2 for z
    \param axis2 Axis id of the connected axis in the node local frame, 
                  0 for x, 1 for y, 2 for z
  */
  SmoresEdge(SmoresNodePtr model_first, SmoresNodePtr model_second,
      double length, double angle,int axis1, int axis2);
  /// Destructor
  ~SmoresEdge();
  /// Find the connected node given one node
  /*!
    \param node The given node
    \return If there is a node connected to given node, then return that node
            otherwise return an empty pointer
  */
  SmoresNodePtr FindMatchingNode(SmoresNodePtr node);
  /// Distance offset along the aligned axis
  double Distance;
  /// Angle offset in radian around the aligned axis
  double Angle;
  /// Axis of the model_1, 0 for x, 1 for y, 2 for z
  int Axis_1;
  /// Axis of the model_2, 0 for x, 1 for y, 2 for z
  int Axis_2;

  /// A pointer to the connected node on the first module
  SmoresNodePtr model_1;
  /// A pointer to the connected node on the second module
  SmoresNodePtr model_2;
  /// A pointer to the dynamically generated joint
  physics::JointPtr DynamicJointPtr;
}; // class SmoresEdge
}
#endif