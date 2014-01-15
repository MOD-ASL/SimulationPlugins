#include "SmoresNode.hh"

SmoresNode::SmoresNode(int nodeID, int jtype, int jvalue, SmoresModulePtr parent, SmoresEdgePtr edge)
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
SmoresNode::SmoresNode(int nodeID, int jtype, int jvalue, SmoresModulePtr parent)
{
	this->NodeID = nodeID;
	this->JointType = jtype;
	this->JointValue = jvalue;
	this->Parent = parent;
	this->Edge.reset();
	switch(nodeID){
		case 0:{this->RelativePosition.Set(0,0,0);break;} // Front wheel
		case 1:{this->RelativePosition.Set(0.05,0.05,0);break;}	// Left wheel
		case 2:{this->RelativePosition.Set(-0.05,0.05,0);break;}	// Right wheel
		case 3:{this->RelativePosition.Set(0,0.05+0.05*cos(jvalue),0.05*sin(jvalue));break;}
	}
}
SmoresNode::SmoresNode(int nodeID, int jtype, int jvalue)
{
	this->NodeID = nodeID;
	this->JointType = jtype;
	this->JointValue = jvalue;
	this->Edge.reset();
	switch(nodeID){
		case 0:{this->RelativePosition.Set(0,0,0);break;} // Front wheel
		case 1:{this->RelativePosition.Set(0.05,0.05,0);break;}	// Left wheel
		case 2:{this->RelativePosition.Set(-0.05,0.05,0);break;}	// Right wheel
		case 3:{this->RelativePosition.Set(0,0.05+0.05*cos(jvalue),0.05*sin(jvalue));break;}
	}
}
SmoresNode::~SmoresNode()
{
	this->Parent.reset();
	this->Edge.reset();
}
void SmoresNode::NodeInit(int nodeID, int jtype, int jvalue, SmoresModulePtr parent, SmoresEdgePtr edge)
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
void SmoresNode::NodeInit(int nodeID, int jtype, int jvalue, SmoresModulePtr parent)
{
	this->NodeID = nodeID;
	this->JointType = jtype;
	this->JointValue = jvalue;
	this->Parent = parent;
	this->Edge.reset();
	switch(nodeID){
		case 0:{this->RelativePosition.Set(0,0,0);break;} // Front wheel
		case 1:{this->RelativePosition.Set(0.05,0.05,0);break;}	// Left wheel
		case 2:{this->RelativePosition.Set(-0.05,0.05,0);break;}	// Right wheel
		case 3:{this->RelativePosition.Set(0,0.05+0.05*cos(jvalue),0.05*sin(jvalue));break;}
	}
}
int SmoresNode::GetAxis(void)
{
	int Axis_ID;
	switch(this->NodeID)
	{
		case 0:Axis_ID = 1;break;
		case 1:Axis_ID = 0;break;
		case 2:Axis_ID = 0;break;
		case 3:Axis_ID = 1;break;
	}
	return Axis_ID;
}
void SmoresNode::ConnectOnEdge(SmoresEdgePtr edge)
{
	this->Edge = edge;
}
void SmoresNode::Disconnect(void)
{
	Edge.reset();
}
SmoresEdgePtr SmoresNode::GetEdge(void)
{
	return Edge;
}