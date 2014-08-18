//! \file Defines smores joint struct that used in Module controller
#ifndef _GAZEBO_SMORES_JOINT_HH_
#define _GAZEBO_SMORES_JOINT_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo{
/// A struct that has richer features than gazebo joint object
struct SmoresJoint
{
  /// JointPtr object of the joint
	physics::JointPtr jointPtr;
  /// The current joint angle of the joint, for PID control
	math::Angle jointAngleNow;
  /// The desired angle for the joint, for PID control
	math::Angle jointAngleDesire;	// This member is unused
	/// Whether the angle of the joint needs to be control
	bool needToBeSet;
	/// Maximium torque of the current joint
	double maximiumForce;
  /// Maximium rotation rate of the current joint, in rad/s
	double maximiumRotRate;
	/// Angle difference between current joint and desired joint, for PID control
 	double jointErrorHis;
  /// Integration of the joint angle error, for PID control
 	double jointErrorAccu;
};
}

#endif