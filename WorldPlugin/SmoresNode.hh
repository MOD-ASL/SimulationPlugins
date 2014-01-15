#ifndef _GAZEBO_SMORES_NODE_HH_
#define _GAZEBO_SMORES_NODE_HH_
// #include "SmoresModule.hh"
#include "SmoresEdge.hh"

using namespace gazebo;

class SmoresModule;

typedef boost::shared_ptr<SmoresEdge> SmoresEdgePtr;
typedef boost::shared_ptr<SmoresModule> SmoresModulePtr;

class SmoresNode
{
public:
	SmoresNode();

	SmoresNode(int nodeID, int jtype, int jvalue, SmoresModulePtr parent, SmoresEdgePtr edge);

	SmoresNode(int nodeID, int jtype, int jvalue, SmoresModulePtr parent);

	SmoresNode(int nodeID, int jtype, int jvalue);

	~SmoresNode();

	void NodeInit(int nodeID, int jtype, int jvalue, SmoresModulePtr parent, SmoresEdgePtr edge);

	void NodeInit(int nodeID, int jtype, int jvalue, SmoresModulePtr parent);

	int GetAxis(void);

	void ConnectOnEdge(SmoresEdgePtr edge);

	void Disconnect(void);

	SmoresEdgePtr GetEdge(void);

//++++++++++++++++++ Node properties goes here +++++++++++++++++++++++++
public: int NodeID;			// Node ID specify which node the current node is, 0 is the base node
public: int JointType;	// 0 for revolute or 1 for prismatic
public: int JointValue;	// Revolute joint: angle value; prismatic joint: distance value
public: math::Vector3 RelativePosition;	// Position of the current node relative to base node
public: SmoresModulePtr Parent;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public: SmoresEdgePtr Edge;	// A pointer to the existing edge
};
#endif