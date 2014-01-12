#ifndef _GAZEBO_SMORES_EDGE_HH_
#define _GAZEBO_SMORES_EDGE_HH_
// #include "SmoresNode.hh"

using namespace gazebo;

class  SmoresNode;

class SmoresEdge
{
public:
	SmoresEdge(SmoresNode *model_first, SmoresNode *model_second, double length, double angle,int axis1, int axis2)
	{
		this->model_1 = model_first;
		this->model_2 = model_second;
		this->Distance = length;
		this->Angle = angle;
		this->Axis_1 = axis1;
		this->Axis_2 = axis2;
	}
	~SmoresEdge();

//++++++++++++++++ Here comes edge propoerties +++++++++++++++++++
public: double Distance; 	// Distance along the aligned axis
public: double Angle; 		// Angle in radian around the aligned axis, relative to model_1's X when aligned along Y and Z or Y when aligned along X
public: int Axis_1; 	// Axis of the model_1, 0 for x, 1 for y, 2 for z
public: int Axis_2; 	// Axis of the model_2, 0 for x, 1 for y, 2 for z
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//---------------- Here goes pointers ----------------------------
	public: SmoresNode *model_1;	// A pointer to the first node on the first module
	public: SmoresNode *model_2;	// A pointer to the second node on the second module
//----------------------------------------------------------------
};
#endif