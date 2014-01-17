#ifndef _GAZEBO_SMORES_MODULE_HH_
#define _GAZEBO_SMORES_MODULE_HH_
// #include "SmoresEdge.hh"
#include "SmoresNode.hh"
#include <string>
// #include <iostream>

using namespace std;
using namespace gazebo;

class SmoresModule: public boost::enable_shared_from_this<SmoresModule>
{
public:
	SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, unsigned int num_ID);

	SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr);

	SmoresModule(string mID, bool mtype, unsigned int num_ID);

	~SmoresModule();

	void ManuallyNodeInitial(SmoresModulePtr module_ptr);

	// Get the pointer to the node by node ID
	SmoresNodePtr GetNode(int node_id);

	int GetNodeAxis(int node_id);

	void SetModulePtr(physics::ModelPtr modelPtr);

	physics::LinkPtr GetLinkPtr(int node_id);


//++++++++++++++ Here comes model properties +++++++++++++++++++++
public: string ModuleID;
public: unsigned int ModuleNumID;	// Position in the vector
public: bool ModuleType; // Active module or Passive Module, true for active
public: physics::ModelPtr ModuleObject;	// A pointer to the real module
// Geometry information

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//-------------- Nodes -------------------------------------------
private:
	SmoresNode NodeUH; 	// The Ushape part
	SmoresNode NodeFW;	// The Front Wheel
	SmoresNode NodeLW;
	SmoresNode NodeRW;

	SmoresNodePtr NodeUHPtr;
	SmoresNodePtr NodeFWPtr;
	SmoresNodePtr NodeLWPtr;
	SmoresNodePtr NodeRWPtr;
//---------------------------------------------------------------- 
};

#endif