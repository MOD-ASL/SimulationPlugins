//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: Thie is a customized model plugin class, in which we 
//              implemented the low level controllers of each individual
//              model. Those controllers including joint position
//              control, joint speed control, model plane motion control,
//              model plane position and orientation control
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! \file Defines the modelplugin for low level control of modules
#ifndef _GAZEBO_GRIPPER_CONTROLLER_HH_
#define _GAZEBO_GRIPPER_CONTROLLER_HH_

#include "ModuleController.hh"

namespace gazebo
{
/// This class only used in competiotion to do the magic gripper trick
class GripperController : public ModuleController 
{
 public:
  /// Constructor
  GripperController();
  /// Destructor
  ~GripperController();
  /// Callback that decodes collision message and populate it to world plugin
  /*! Used by magnetic connection
    \param msg GzStringPtr object of the collision message
  */
  virtual void CollisionReceivingCallback(GzStringPtr &msg);
  /// Extra stuff taht should be set on load
  virtual void ExtraOnload(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
 private:
  /// Gripper connection dynamically generated joint
  physics::JointPtr dynamicJoint;
}; // class GripperController
} // namespace gazebo
#endif
