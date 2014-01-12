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
	// Get the pointer to the node by node ID
	SmoresNode * GetNode(int node_id)
	{
		switch(node_id)
		{
			case 0:return &NodeFW;break;
			case 1:return &NodeLW;break;
			case 2:return &NodeRW;break;
			case 3:return &NodeUH;break;
		} 
	}
	int GetNodeAxis(int node_id)
	{
		return GetNode(node_id)->GetAxis();
	}

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