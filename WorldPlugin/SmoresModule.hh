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
		SmoresModulePtr module_ptr(this);
		this->ModuleID = mID;
		this->ModuleType = mtype;
		NodeFW.NodeInit(0,0,0,module_ptr);
		NodeUH.NodeInit(3,0,0,module_ptr);
		NodeLW.NodeInit(1,0,0,module_ptr);
		NodeRW.NodeInit(2,0,0,module_ptr);
	}
	~SmoresModule();
	void ManuallyNodeInitial(SmoresModulePtr module_ptr)
	{
		NodeFW.NodeInit(0,0,0,module_ptr);
		NodeUH.NodeInit(3,0,0,module_ptr);
		NodeLW.NodeInit(1,0,0,module_ptr);
		NodeRW.NodeInit(2,0,0,module_ptr);
	}
	// Get the pointer to the node by node ID
	SmoresNodePtr GetNode(int node_id)
	{
		switch(node_id)
		{
			case 0:{SmoresNodePtr NodePoint2(&NodeFW);return NodePoint2;break;}
			case 1:{SmoresNodePtr NodePoint2(&NodeLW);return NodePoint2;break;}
			case 2:{SmoresNodePtr NodePoint2(&NodeRW);return NodePoint2;break;}
			case 3:{SmoresNodePtr NodePoint2(&NodeUH);return NodePoint2;break;}
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