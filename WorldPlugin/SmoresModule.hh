#ifndef _GAZEBO_SMORES_MODULE_HH_
#define _GAZEBO_SMORES_MODULE_HH_
// #include "SmoresEdge.hh"
#include "SmoresNode.hh"
#include <string>

using namespace std;
using namespace gazebo;

class SmoresModule
{
public:
	SmoresModule(string mID, bool mtype)
	{
		this->ModuleID = mID;
		this->ModuleType = mtype;
		NodeFW.NodeInit(0,0,0,this);
		NodeUH.NodeInit(3,0,0,this);
		NodeLW.NodeInit(1,0,0,this);
		NodeRW.NodeInit(2,0,0,this);
	}
	~SmoresModule();

//++++++++++++++ Here comes model properties +++++++++++++++++++++
public: string ModuleID;
public: bool ModuleType; // Active module or Passive Module
// Geometry information

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//-------------- Nodes -------------------------------------------
private:
	SmoresNode NodeUH; 	// The Ushape part
	SmoresNode NodeFW;	// The Front Wheel
	SmoresNode NodeLW;
	SmoresNode NodeRW;
//---------------------------------------------------------------- 
};

#endif