#include "SmoresModule.hh"

ModuleCommands::ModuleCommands(SmoresModulePtr which_module)
{
  this->WhichModule = which_module;
  FinishedFlag = false;
  ReceivedFlag = false;
}
  
SmoresModule::SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, transport::PublisherPtr publisher, transport::SubscriberPtr subsciber, unsigned int num_ID):
																						NodeUH(3,0,0),NodeFW(0,0,0),NodeLW(1,0,0),NodeRW(2,0,0),NodeUHPtr(&NodeUH),NodeFWPtr(&NodeFW),NodeLWPtr(&NodeLW),NodeRWPtr(&NodeRW)
{
	this->ModuleID = mID;
	this->ModuleType = mtype;
	this->ModuleObject = modulePtr;
	this->ModuleNumID = num_ID;
	this->ModulePublisher = publisher;
	this->ModuleSubscriber = subsciber;
}
SmoresModule::SmoresModule(string mID, bool mtype, physics::ModelPtr modulePtr, transport::PublisherPtr publisher, transport::SubscriberPtr subsciber):
																						NodeUH(3,0,0),NodeFW(0,0,0),NodeLW(1,0,0),NodeRW(2,0,0),NodeUHPtr(&NodeUH),NodeFWPtr(&NodeFW),NodeLWPtr(&NodeLW),NodeRWPtr(&NodeRW)
{
	this->ModuleID = mID;
	this->ModuleType = mtype;
	this->ModuleObject = modulePtr;
	this->ModulePublisher = publisher;
	this->ModuleSubscriber = subsciber;
}
SmoresModule::SmoresModule(string mID, bool mtype, transport::PublisherPtr publisher, transport::SubscriberPtr subsciber, unsigned int num_ID):
																						NodeUH(3,0,0),NodeFW(0,0,0),NodeLW(1,0,0),NodeRW(2,0,0),NodeUHPtr(&NodeUH),NodeFWPtr(&NodeFW),NodeLWPtr(&NodeLW),NodeRWPtr(&NodeRW)
{
	this->ModuleID = mID;
	this->ModuleType = mtype;
	this->ModuleNumID = num_ID;
	this->ModulePublisher = publisher;
	this->ModuleSubscriber = subsciber;
}
SmoresModule::~SmoresModule()
{
	this->ModuleObject.reset();
	this->NodeUHPtr.reset();
	this->NodeFWPtr.reset();
	this->NodeLWPtr.reset();
	this->NodeRWPtr.reset();
	this->ModulePublisher.reset();
}
// This will be a temporary solution before I come up with some new ideas
void SmoresModule::ManuallyNodeInitial(SmoresModulePtr module_ptr)
{
	NodeFW.SetParent(module_ptr);
	NodeUH.SetParent(module_ptr);
	NodeLW.SetParent(module_ptr);
	NodeRW.SetParent(module_ptr);
}
// Get the pointer to the node by node ID
SmoresNodePtr SmoresModule::GetNode(int node_id)
{
	SmoresNodePtr NodePoint2;
	switch(node_id)
	{
		case 0:{NodePoint2 = NodeFWPtr;break;}
		case 1:{NodePoint2 = NodeLWPtr;break;}
		case 2:{NodePoint2 = NodeRWPtr;break;}
		case 3:{NodePoint2 = NodeUHPtr;break;}
	}
	return NodePoint2;
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