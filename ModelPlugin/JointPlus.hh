#ifndef _GAZEBO_JOINT_PLUS_HH_
#define _GAZEBO_JOINT_PLUS_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo{
struct JointPlus
{
	physics::JointPtr jointPtr;
	math::Angle jointAngleNow;
	math::Angle jointAngleDesire;	// This member is unused
	// Whether the angle of the joint needs to be set
	bool needToBeSet;
	// Joint Properties
	double maximiumForce;
	double maximiumRotRate;
	// Variables need to be use to control the joint
 	double jointErrorHis;
 	double jointErrorAccu;
};
}

#endif