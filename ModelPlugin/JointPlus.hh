#ifndef _GAZEBO_JOINT_PLUS_HH_
#define _GAZEBO_JOINT_PLUS_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using namespace gazebo;

struct JointPlus
	{
		physics::JointPtr JointX;
		math::Angle JointAngleNow;
		math::Angle JointAngleDesire;
		// Whether the angle of the joint needs to be set
		bool Need2BeSet;

		// Joint Properties
		double MaximiumForce;
		double MaximiumRotRate;

		// Variables need to be use to control the joint
	 	double JointErrorHis;
	 	double JointErrorAccu;
	};

	#endif