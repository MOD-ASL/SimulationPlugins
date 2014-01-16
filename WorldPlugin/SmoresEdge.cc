#include "SmoresEdge.hh"

SmoresEdge::SmoresEdge(SmoresNodePtr model_first, SmoresNodePtr model_second, double length, double angle,int axis1, int axis2)
{
	this->model_1 = model_first;
	this->model_2 = model_second;
	this->Distance = length;
	this->Angle = angle;
	this->Axis_1 = axis1;
	this->Axis_2 = axis2;
}
SmoresEdge::~SmoresEdge()
{
	this->model_1.reset();
	this->model_2.reset();
	this->DynamicJointPtr.reset();
}