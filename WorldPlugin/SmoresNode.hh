#ifndef _GAZEBO_SMORES_NODE_HH_
#define _GAZEBO_SMORES_NODE_HH_
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// #include "SmoresModule.hh"
#include "SmoresEdge.hh"

using namespace gazebo;

class SmoresModule;

class SmoresNode
{
public:
	SmoresNode();
	SmoresNode(int nodeID, int jtype, int jvalue, SmoresModule *parent, SmoresEdge *edge)
	{
		this->NodeID = nodeID;
		this->JointType = jtype;
		this->JointValue = jvalue;
		this->Parent = parent;
		this->Edge = edge;
		// Only consider the case of smores revolute joint, need to be changed in the future
		switch(nodeID){
			case 0:{this->RelativePosition.Set(0,0,0);break;} // Front wheel
			case 1:{this->RelativePosition.Set(0.05,0.05,0);break;}	// Left wheel
			case 2:{this->RelativePosition.Set(-0.05,0.05,0);break;}	// Right wheel
			case 3:{this->RelativePosition.Set(0,0.05+0.05*cos(jvalue),0.05*sin(jvalue));break;}
		}
	}
	SmoresNode(int nodeID, int jtype, int jvalue, SmoresModule *parent)
	{
		this->NodeID = nodeID;
		this->JointType = jtype;
		this->JointValue = jvalue;
		this->Parent = parent;
		this->Edge = 0;
		switch(nodeID){
			case 0:{this->RelativePosition.Set(0,0,0);break;} // Front wheel
			case 1:{this->RelativePosition.Set(0.05,0.05,0);break;}	// Left wheel
			case 2:{this->RelativePosition.Set(-0.05,0.05,0);break;}	// Right wheel
			case 3:{this->RelativePosition.Set(0,0.05+0.05*cos(jvalue),0.05*sin(jvalue));break;}
		}
	}
	~SmoresNode();
	void NodeInit(int nodeID, int jtype, int jvalue, SmoresModule *parent, SmoresEdge *edge)
	{
		this->NodeID = nodeID;
		this->JointType = jtype;
		this->JointValue = jvalue;
		this->Parent = parent;
		this->Edge = edge;
		// Only consider the case of smores revolute joint, need to be changed in the future
		switch(nodeID){
			case 0:{this->RelativePosition.Set(0,0,0);break;} // Front wheel
			case 1:{this->RelativePosition.Set(0.05,0.05,0);break;}	// Left wheel
			case 2:{this->RelativePosition.Set(-0.05,0.05,0);break;}	// Right wheel
			case 3:{this->RelativePosition.Set(0,0.05+0.05*cos(jvalue),0.05*sin(jvalue));break;}
		}
	}
	void NodeInit(int nodeID, int jtype, int jvalue, SmoresModule *parent)
	{
		this->NodeID = nodeID;
		this->JointType = jtype;
		this->JointValue = jvalue;
		this->Parent = parent;
		this->Edge = 0;
		switch(nodeID){
			case 0:{this->RelativePosition.Set(0,0,0);break;} // Front wheel
			case 1:{this->RelativePosition.Set(0.05,0.05,0);break;}	// Left wheel
			case 2:{this->RelativePosition.Set(-0.05,0.05,0);break;}	// Right wheel
			case 3:{this->RelativePosition.Set(0,0.05+0.05*cos(jvalue),0.05*sin(jvalue));break;}
		}
	}
	int GetAxis(void)
	{
		switch(this->NodeID)
		{
			case 0:return 1;break;
			case 1:return 0;break;
			case 2:return 0;break;
			case 3:return 1;break;
		}
	}
	void ConnectOnEdge(SmoresEdge *edge)
	{
		this->Edge = edge;
	}
	void Disconnect(void)
	{
		this->Edge = 0;
	}
	SmoresEdge * GetEdge(void)
	{
		return Edge;
	}

//++++++++++++++++++ Node properties goes here +++++++++++++++++++++++++
public: int NodeID;			// Node ID specify which node the current node is, 0 is the base node
public: int JointType;	// 0 for revolute or 1 for prismatic
public: int JointValue;	// Revolute joint: angle value; prismatic joint: distance value
public: math::Vector3 RelativePosition;	// Position of the current node relative to base node
public: SmoresModule *Parent;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
public: SmoresEdge *Edge;	// A pointer to the existing edge
};
#endif