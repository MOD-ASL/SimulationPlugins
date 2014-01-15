#include "SmoresModule.hh"

SmoresModule::SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, unsigned int num_ID):
																						NodeUH(3,0,0),NodeFW(0,0,0),NodeLW(1,0,0),NodeRW(2,0,0)
{
	this->ModuleID = mID;
	this->ModuleType = mtype;
	this->ModuleObject = modulePtr;
	this->ModuleNumID = num_ID;
}
SmoresModule::SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr):
																						NodeUH(3,0,0),NodeFW(0,0,0),NodeLW(1,0,0),NodeRW(2,0,0)
{
	this->ModuleID = mID;
	this->ModuleType = mtype;
	this->ModuleObject = modulePtr;
}
SmoresModule::SmoresModule(string mID, bool mtype, unsigned int num_ID):
																						NodeUH(3,0,0),NodeFW(0,0,0),NodeLW(1,0,0),NodeRW(2,0,0)
{
	this->ModuleID = mID;
	this->ModuleType = mtype;
	this->ModuleNumID = num_ID;
}
SmoresModule::~SmoresModule()
{
	this->ModuleObject.reset();
}
// This will be a temporary solution before I come up with some new ideas
void SmoresModule::ManuallyNodeInitial(SmoresModulePtr module_ptr)
{
	NodeFW.NodeInit(0,0,0,module_ptr);
	NodeUH.NodeInit(3,0,0,module_ptr);
	NodeLW.NodeInit(1,0,0,module_ptr);
	NodeRW.NodeInit(2,0,0,module_ptr);
}
// Get the pointer to the node by node ID
SmoresNodePtr SmoresModule::GetNode(int node_id)
{
	switch(node_id)
	{
		case 0:{SmoresNodePtr NodePoint2(&NodeFW);return NodePoint2;break;}
		case 1:{SmoresNodePtr NodePoint2(&NodeLW);return NodePoint2;break;}
		case 2:{SmoresNodePtr NodePoint2(&NodeRW);return NodePoint2;break;}
		case 3:{SmoresNodePtr NodePoint2(&NodeUH);return NodePoint2;break;}
	}
	// return NodePoint2;
}
int SmoresModule::GetNodeAxis(int node_id)
{
	return GetNode(node_id)->GetAxis();
}
void SmoresModule::SetModulePtr(physics::ModelPtr modelPtr)
{
	this->ModuleObject = modelPtr;
}
physics::LinkPtr SmoresModule::GetLinkPtr(int node_id)
{
	physics::LinkPtr link_ptr;
	switch(node_id)
	{
		case 0: link_ptr = this->ModuleObject->GetLink("FrontWheel");break;
		case 1: link_ptr = this->ModuleObject->GetLink("LeftWheel");break;
		case 2: link_ptr = this->ModuleObject->GetLink("RightWheel");break;
		case 3: link_ptr = this->ModuleObject->GetLink("UHolderBody");break;
	}
	return link_ptr;
}