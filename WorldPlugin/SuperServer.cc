#include <sdf/sdf.hh>
#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include <string>
#include <iostream>
#include <cmath>
// Libraries for messages needed to use to communicate between plugins
#include "collision_message_plus.pb.h"
#include "command_message.pb.h"
// Libraries for connectivity representation
#include "SmoresModule.hh"

#define PI 3.141593   // 3.1411593
#define VALIDCONNECTIONDISUPPER 0.110
#define VALIDCONNECTIONDISLOWER 0.098


using namespace std;

namespace gazebo
{
  typedef const boost::shared_ptr<const collision_message_plus::msgs::CollisionMessage> CollisionMessagePtr;
  typedef const boost::shared_ptr<const command_message::msgs::CommandMessage> CommandMessagePtr;
  string Int2String(int number) //
  {
     stringstream ss; //create a stringstream
     ss << number;    //add number to the stream
     return ss.str(); //return a string with the contents of the stream
  }
  class CollisionInformation
  {
  //++++++++++++++ Class Methods +++++++++++++++++++++++++++++++++
  public:
    CollisionInformation(string collision1, string collision2, string link_collision1, string link_collision2)
    {
      Model1 = collision1;
      Model2 = collision2;
      LinkOfModel1 = link_collision1;
      LinkofModel2 = link_collision2;
    }
    bool SameCollision(string collision1, string collision2, string link_collision1, string link_collision2)
    {
      bool same_collision = false;
      if (collision1.compare(Model1)==0 && collision2.compare(Model2)==0)
      {
        if (link_collision1.compare(LinkOfModel1)==0 && link_collision2.compare(LinkofModel2)==0)
        {
          same_collision = true;
        }
      }else{
        if (collision1.compare(Model2)==0 && collision2.compare(Model1)==0)
        {
          if (link_collision1.compare(LinkofModel2)==0 && link_collision2.compare(LinkOfModel1)==0)
          {
            same_collision = true;
          }
        }
      }
      return same_collision;
    }
  //++++++++++++++ Class Members +++++++++++++++++++++++++++++++++
  public:
    string Model1;
    string Model2;
    string LinkOfModel1;
    string LinkofModel2;
  };
  class ControlCenter : public WorldPlugin
  {
    public: ControlCenter()
    {
      numOfModules = 0;
      infoCounter = 0;
      NeedToSetPtr = false;
    }
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
      this->currentWorld = _parent;
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/physics topic
      this->statePub = node->Advertise<msgs::GzString>("~/Welcome");

      // Set the step time
      // stateMsg.set_max_step_size(0.01);

      // Change gravity
      // msgs::Set(physicsMsg.mutable_gravity(), math::Vector3(0.01, 0, 0.1));
      // statePub->Publish(stateMsg);
      this->addEntityConnection = event::Events::ConnectAddEntity(boost::bind(&ControlCenter::addEntity2World, this, _1));
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControlCenter::OnSystemRunning, this, _1));
    }
    private: void addEntity2World(std::string & _info)
    {
      // transport::NodePtr node(new transport::Node());
      // node->Init();
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Welcome message generation each time a new model has been added
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      msgs::GzString welcomeMsg;
      string CurrentMessage;

      int modelnumber = this->currentWorld->GetModelCount();
      numOfModules = modelnumber-1;
      cout<<"World: Number of models: "<<modelnumber<<endl;
      // cout<<"Default information: "<<_info<<endl;
      // this->statePub = node->Advertise<msgs::GzString>("~/Welcome");
      CurrentMessage = "Model"+Int2String(modelnumber);
      welcomeMsg.set_data(CurrentMessage);

      statePub->Publish(welcomeMsg);
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Dynamic publisher generation
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      transport::NodePtr node1(new transport::Node());
      node1->Init(_info);
      string NewPubName = "~/" + _info + "_world";
      transport::PublisherPtr newModulePub = node1->Advertise<command_message::msgs::CommandMessage>(NewPubName);
      NewPubName = "~/" + _info + "_model";
      transport::SubscriberPtr newModuleSub = node1->Subscribe(NewPubName,&ControlCenter::FeedBackMessageDecoding, this);
      // WorldPublisher.push_back(newWorldPub);

      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Store the pointers of model into vectors
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // modelGroup.push_back(currentWorld->GetModel(_info));
      unsigned int howManyModules = moduleList.size();
      SmoresModulePtr newModule(new SmoresModule(_info, true, newModulePub, newModuleSub, howManyModules));
      newModule->ManuallyNodeInitial(newModule);
      NeedToSetPtr = true;
      moduleList.push_back(newModule);
      // newModule->ManuallyNodeInitial(newModule);
      // moduleList.at(howManyModules)->SaySomthing();

      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Dynamic subscriber of collision topic
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      string NewSubName = "~/"+_info+"_Collision";
      transport::SubscriberPtr newWorldSub = node1->Subscribe(NewSubName,&ControlCenter::AutomaticMagneticConnectionManagement, this);
      WorldColSubscriber.push_back(newWorldSub);
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function will be called in the every iteration of the simulation
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void OnSystemRunning(const common::UpdateInfo & /*_info*/)
    {
      SetThePointerInSmoresModule();


      // common::Time world_sim_time = currentWorld->GetSimTime();
      // if (world_sim_time.sec >= 20 && world_sim_time.sec <= 21)
      // {
      //   Deconnection(ConnectionEdges.back());
      //   cout<<"World: The crush has nothing to do with disconnection"<<endl;
      // }
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function will be called everytime receive command information
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void FeedBackMessageDecoding(CommandMessagePtr &msg)
    {}
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function will be called everytime receive collision information
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void AutomaticMagneticConnectionManagement(CollisionMessagePtr &msg)
    {
      string ModelOfCollision1 = msg->collision1().substr(0,msg->collision1().find("::"));
      string ModelOfCollision2 = msg->collision2().substr(0,msg->collision2().find("::"));
      string LinkOfCollision1 = msg->collision1().substr(msg->collision1().find("::")+2,msg->collision1().rfind("::")-msg->collision1().find("::")-2);
      string LinkOfCollision2 = msg->collision2().substr(msg->collision2().find("::")+2,msg->collision2().rfind("::")-msg->collision2().find("::")-2);
      SmoresModulePtr Model1Ptr = GetModulePtrByName(ModelOfCollision1);
      SmoresModulePtr Model2Ptr = GetModulePtrByName(ModelOfCollision2);
      int NodeOfModel1 = GetNodeIDByName(LinkOfCollision1);
      int NodeOfModel2 = GetNodeIDByName(LinkOfCollision2);

      math::Pose ContactLinkPos = msgs::Convert(msg->positioncol1());
      //---------------------- Find the pending connection request ------------------------
      int FoundPendingOne = 0;
      for (unsigned int i = 0; i < PendingRequest.size(); ++i)
      {
        if (PendingRequest.at(i).SameCollision(ModelOfCollision1,ModelOfCollision2,LinkOfCollision1,LinkOfCollision2))
        {
          PendingRequest.erase(PendingRequest.begin()+i);
          FoundPendingOne = 1;
          break;
        }
      }
      if(NodeOfModel1<4 && NodeOfModel2<4)
      {
        if (FoundPendingOne==0)
        {
          //---------------------- Add new pending connection request ------------------------
          if ((!AlreadyConnected(Model1Ptr,NodeOfModel1)) && (!AlreadyConnected(Model2Ptr,NodeOfModel2)) && (!AlreadyConnected(Model1Ptr,Model2Ptr)))
          {
            // This part is used to check the distance between robots
            math::Vector3 CenterModel1 = currentWorld->GetModel(ModelOfCollision1)->GetWorldPose().pos;
            math::Vector3 CenterModel2 = currentWorld->GetModel(ModelOfCollision2)->GetWorldPose().pos;
            if((CenterModel1-CenterModel2).GetLength()<VALIDCONNECTIONDISUPPER && (CenterModel1-CenterModel2).GetLength()>VALIDCONNECTIONDISLOWER)
            {
              cout<<"World: Distance between centers: "<<(CenterModel1-CenterModel2).GetLength()<<endl;
              CollisionInformation NewConnectionRequest(ModelOfCollision1, ModelOfCollision2,LinkOfCollision1,LinkOfCollision2);
              PendingRequest.push_back(NewConnectionRequest);
              cout<<"World: An pending entry has been established: '"<< ModelOfCollision1+":"+LinkOfCollision1+"::"+ ModelOfCollision2 + ":"+LinkOfCollision2<<"'"<<endl;
            }
          }
        }else{
          // The lighter configuration connects to heavier configuration
          if (CountModules(Model1Ptr)<CountModules(Model2Ptr))
          {
            SmoresModulePtr tmp_module_ptr = Model1Ptr;
            Model1Ptr = Model2Ptr;
            Model2Ptr = tmp_module_ptr;
            int tmp_node = NodeOfModel1;
            NodeOfModel1 = NodeOfModel2;
            NodeOfModel2 = tmp_node;
          }
          //-------- Do the real connection (including generate dynamic joint) ----------------
          ActiveConnection(Model1Ptr,Model2Ptr,NodeOfModel1,NodeOfModel2);
        }
      }
    }

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // These function are used to physically connect different models by generating dynamic joint
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void ConnectAndDynamicJointGeneration(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, SmoresEdgePtr an_edge)
    {
      math::Pose ContactLinkPos = module_1->GetLinkPtr(node1_ID)->GetWorldPose();
      math::Pose PosOfTheOtherModel = module_2->GetLinkPtr(node2_ID)->GetWorldPose();
      cout<<"World: Post position of the module 1: ("<<ContactLinkPos.pos.x<<","<<ContactLinkPos.pos.y<<","<<ContactLinkPos.pos.z<<")"<<endl;
      cout<<"World: Post position of the module 2: ("<<PosOfTheOtherModel.pos.x<<","<<PosOfTheOtherModel.pos.y<<","<<PosOfTheOtherModel.pos.z<<")"<<endl;
      physics::LinkPtr Link1, Link2;
      math::Vector3 axis;
      math::Vector3 ZDirectionOffset(0,0,0.000);  //0.008
      // math::Vector3 newCenterPoint = 0.5*(ContactLinkPos.pos + PendingRequestPos.at(i).pos)+ZDirectionOffset;
      math::Vector3 newPositionOfLink1;
      math::Vector3 newPositionOfLink2;
      math::Quaternion newDirectionofLink1;
      math::Vector3 newZAxis;
      double AngleBetweenZAxes;
      math::Quaternion FirstRotationOfLink2;
      math::Quaternion SecondRotationOfLink2;
      math::Quaternion newDirectionofLink2;
      if (node1_ID == 0)
      {
        newPositionOfLink1 = ContactLinkPos.pos;
        newPositionOfLink2 = ContactLinkPos.pos - (0.1+an_edge->Distance)*ContactLinkPos.rot.GetYAxis();
        newDirectionofLink1 = ContactLinkPos.rot;
        newZAxis = PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetXAxis())*newDirectionofLink1.GetXAxis();
        newZAxis = newZAxis.Normalize();
        AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis())); // + an_edge->Angle;
        FirstRotationOfLink2.SetFromEuler(0,0,PI);
        double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
        if (DirectionReference>0)
        {
          SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
        }else{
          SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
        }
        if (node2_ID==3)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,0);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (node2_ID==1)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
          }
        }
        if (node2_ID==2)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
          }
        }
        newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
        axis.Set(0,1,0);
      }
      if (node1_ID == 1)
      {
        newPositionOfLink1 = ContactLinkPos.pos;
        newPositionOfLink2 = ContactLinkPos.pos + (0.1+an_edge->Distance)*ContactLinkPos.rot.GetXAxis();
        newDirectionofLink1 = ContactLinkPos.rot;
        newZAxis = PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetYAxis())*newDirectionofLink1.GetYAxis();
        newZAxis = newZAxis.Normalize();
        AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis())); // + an_edge->Angle;
        FirstRotationOfLink2.SetFromEuler(0,0,PI);
        double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
        if (DirectionReference>0)
        {
          SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
        }else{
          SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
        }
        if (node2_ID==0)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (node2_ID==3)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (node2_ID==2)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
          }
        }
        newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
        axis.Set(1,0,0);
      }
      if (node1_ID == 2)
      {
        newPositionOfLink1 = ContactLinkPos.pos;
        newPositionOfLink2 = ContactLinkPos.pos + (0.1+an_edge->Distance)*ContactLinkPos.rot.GetXAxis();
        newDirectionofLink1 = ContactLinkPos.rot;
        newZAxis = PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetYAxis())*newDirectionofLink1.GetYAxis();
        newZAxis = newZAxis.Normalize();
        AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis())); // + an_edge->Angle;
        FirstRotationOfLink2.SetFromEuler(0,0,PI);
        double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
        if (DirectionReference>0)
        {
          SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
        }else{
          SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
        }
        if (node2_ID==0)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (node2_ID==3)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (node2_ID==1)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
          }
        }
        newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
        axis.Set(1,0,0);
      }
      if (node1_ID == 3)
      {
        newPositionOfLink1 = ContactLinkPos.pos;
        newPositionOfLink2 = ContactLinkPos.pos + (0.1+an_edge->Distance)*ContactLinkPos.rot.GetYAxis();
        newDirectionofLink1 = ContactLinkPos.rot;
        newZAxis = PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetXAxis())*newDirectionofLink1.GetXAxis();
        newZAxis = newZAxis.Normalize();
        AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis())); // + an_edge->Angle;
        FirstRotationOfLink2.SetFromEuler(0,0,PI);
        double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
        if (DirectionReference>0)
        {
          SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
        }else{
          SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
        }
        if (node2_ID==0)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,0);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
          }else{
            SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
          }
        }
        if (node2_ID==1)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
          }
        }
        if (node2_ID==2)
        {
          FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
          DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
          if (DirectionReference>0)
          {
            SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
          }else{
            SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
          }
        }
        newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
        axis.Set(0,1,0);
      }
      //+++++++++++++++++ This part of the code set the correct position of the models +++++++++++++++++
      Link1 = module_1->GetLinkPtr(node1_ID);
      Link2 = module_2->GetLinkPtr(node2_ID);

      module_1->ModuleObject->SetLinkWorldPose(math::Pose(newPositionOfLink1,newDirectionofLink1),Link1);
      module_2->ModuleObject->SetLinkWorldPose(math::Pose(newPositionOfLink2,newDirectionofLink2),Link2);

      //++++++++++++++++++ This part of the code generate the dynamic joint ++++++++++++++++++++++++++++
      physics::JointPtr DynamicJoint;
      DynamicJoint = currentWorld->GetPhysicsEngine()->CreateJoint("revolute",  module_1->ModuleObject);
      DynamicJoint->Attach(Link1, Link2);
      DynamicJoint->Load(Link1, Link2, math::Pose(math::Vector3(0,-0.00,0),math::Quaternion()));
      DynamicJoint->SetAxis(0, axis);
      module_1->ModuleObject->GetJointController()->AddJoint(DynamicJoint);
      DynamicJoint->SetAngle(0,math::Angle(0));
      DynamicJoint->SetHighStop(0,math::Angle(0.01));
      DynamicJoint->SetLowStop(0,math::Angle(-0.01));
      DynamicConnections.push_back(DynamicJoint);
      // This is necessary for easy access of the dynamic joint
      // an_edge->DynamicJointPtr = DynamicJoint;
      an_edge->DynamicJointPtr = DynamicConnections.back();
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function is used to physically destroy the connection between different modules, which is dynamic joint here
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void DynamicJointDestroy(SmoresEdgePtr aEdge)
    {
      aEdge->DynamicJointPtr->Detach();
      // cout<<"World: The crush is not because of dynamic joint detach"<<endl;
      for (unsigned int i = 0; i < DynamicConnections.size(); ++i)
      {
        if (aEdge->DynamicJointPtr==DynamicConnections.at(i))
        {
          // cout<<"World: The crush is before reset"<<endl;
          DynamicConnections.at(i).reset();
          // cout<<"World: The crush is after reset"<<endl;
          DynamicConnections.erase(DynamicConnections.begin()+i);
          break;
        }
      }
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // These functions are used to connect or deconnect modules
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Connect two modules by pointers and node_ID
    public: void PassiveConnection(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, double node_angle = 0, double node_distance = 0)
    {
      if (!AlreadyConnected(module_1, module_2, node1_ID, node2_ID))
      {
        SmoresEdgePtr new_connection(new SmoresEdge(module_1->GetNode(node1_ID),module_2->GetNode(node2_ID),node_distance,node_angle,module_1->GetNodeAxis(node1_ID),module_2->GetNodeAxis(node2_ID)));
        ConnectionEdges.push_back(new_connection);
        module_1->GetNode(node1_ID)->ConnectOnEdge(new_connection);
        module_2->GetNode(node2_ID)->ConnectOnEdge(new_connection);
        // Adding the dynamic joint after adding the new edge
        ConnectAndDynamicJointGeneration(module_1, module_2, node1_ID, node2_ID, new_connection);
      }
    }
    public: void ActiveConnection(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, double node_angle = 0, double node_distance = 0)
    {
      if (!AlreadyConnected(module_1, module_2, node1_ID, node2_ID))
      {
        SmoresEdgePtr new_connection(new SmoresEdge(module_1->GetNode(node1_ID),module_2->GetNode(node2_ID),node_distance,node_angle,module_1->GetNodeAxis(node1_ID),module_2->GetNodeAxis(node2_ID)));
        ConnectionEdges.push_back(new_connection);
        module_1->GetNode(node1_ID)->ConnectOnEdge(new_connection);
        module_2->GetNode(node2_ID)->ConnectOnEdge(new_connection);
        // Adding the dynamic joint after adding the new edge
        ConnectAndDynamicJointGeneration(module_1, module_2, node1_ID, node2_ID, new_connection);
      }
    }
    // Deconnect two modules on one edge
    public: void Deconnection(SmoresEdgePtr aEdge)  // This pointer must point to an element in the vector
    {
      DynamicJointDestroy(aEdge);
      aEdge->model_1->Edge.reset();
      aEdge->model_2->Edge.reset();
      cout<<"World: The crush is after reset of edge in node"<<endl;
      for (unsigned int i = 0; i < ConnectionEdges.size(); ++i)
      {
        if (ConnectionEdges.at(i)==aEdge)
        {
          ConnectionEdges.at(i).reset();
          ConnectionEdges.erase(ConnectionEdges.begin()+i);
          break;
        }
      }
    }
    // Deconnect two module base on one module and one node of that module
    public: void Deconnection(SmoresModulePtr aModule, int node_ID)
    {
      SmoresEdgePtr aEdge = aModule->GetNode(node_ID)->GetEdge();
      this->Deconnection(aEdge);

    }
    // Deconnect two module base on one module and one node of that module
    public: void Deconnection(string moduleName, int node_ID)
    {

    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // These functions are used to manage and send message to model
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public: void SendGaitTable(SmoresModulePtr module, bool flag[4], double gait_value[4], int msg_type = 3)
    {
      command_message::msgs::CommandMessage ConnectionMessage;
      ConnectionMessage.set_messagetype(msg_type);
      for (int i = 0; i < 4; ++i)
      {
        ConnectionMessage.set_jointgaittablestatus(i,flag[i]);
        ConnectionMessage.set_jointgaittable(i,gait_value[i]);
      }
      module->ModulePublisher->Publish(ConnectionMessage);
    }
    public: void SendGaitTable(SmoresModulePtr module, int joint_ID, double gait_value, int msg_type = 3)
    {
      bool flag[4] = {false};
      double gait_values[4] = {0};
      flag[joint_ID] = true;
      gait_values[joint_ID] = gait_value;
      SendGaitTable(module, flag, gait_values, msg_type);
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // These functions are utility functions
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public: int GetNodeIDByName(string node_name)
    {
      if (node_name.rfind("FrontWheel")!=string::npos)
      {
        return 0;
      }
      if (node_name.rfind("LeftWheel")!=string::npos)
      {
        return 1;
      }
      if (node_name.rfind("RightWheel")!=string::npos)
      {
        return 2;
      }
      if (node_name.rfind("UHolderBody")!=string::npos)
      {
        return 3;
      }
      return 4; // When return 4, then there is no match found
    }
    public: SmoresModulePtr GetModulePtrByName(string module_name)
    {
      SmoresModulePtr ExistModule;
      for (unsigned int i = 0; i < moduleList.size(); ++i)
      {
        if (module_name.compare(moduleList.at(i)->ModuleID)==0)
        {
          ExistModule = moduleList.at(i);
          break;
        }
      }
      return ExistModule;
    }
    public: int GetModuleIDXByName(string module_name)
    {
      int ExistModule = -1;
      for (unsigned int i = 0; i < moduleList.size(); ++i)
      {
        if (module_name.compare(moduleList.at(i)->ModuleID)==0)
        {
          ExistModule = i;
          break;
        }
      }
      return ExistModule;
    }
    public: void SetThePointerInSmoresModule(void)
    {
      if (NeedToSetPtr)
      {
        for (unsigned int i = 0; i < moduleList.size(); ++i)
        {
          moduleList.at(i)->SetModulePtr(currentWorld->GetModel(moduleList.at(i)->ModuleID));
        }
        NeedToSetPtr = false;
        cout<<"World: Pointer has been assigned"<<endl;
      }
    }
    // Check whether two nodes are connected together
    public: bool AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID)
    {
      bool HavingAConnection;
      if((bool)module_1->GetNode(node1_ID)->Edge && (bool)module_2->GetNode(node2_ID)->Edge)
      {
        HavingAConnection = true;
      }else
      {
        HavingAConnection = false;
      }
      return HavingAConnection;
    }
    // Check whether two modules have already connected on a node
    public: bool AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2)
    {
      bool HavingAConnection = false;
      for (int j = 0; j < 4; ++j)
      {
        if (module_1->GetNode(j)->Edge)
        {
          SmoresModulePtr ConnectedModule = module_1->GetNode(j)->Edge->FindMatchingNode(module_1->GetNode(j))->Parent;
          if (ConnectedModule == module_2)
          {
            HavingAConnection = true;
            break;
          }
        }
      }
      return HavingAConnection;
    }
    // Check whether a node a of module has been occupied
    public: bool AlreadyConnected(SmoresModulePtr module, int node_ID)
    {
      bool HavingAConnection = false;
      if (module->GetNode(node_ID)->Edge)
      {
        HavingAConnection = true;
      }
      return HavingAConnection;
    }
    // This function is used to count for a configuration, how many modules are there
    private: unsigned int CountModules(SmoresModulePtr module)
    {
      vector<SmoresModulePtr> vector1;
      vector<SmoresModulePtr> vector2;
      vector<SmoresModulePtr> vector3;
      vector1.push_back(module);
      vector3.push_back(module);
      unsigned int module_count = 0;
      while(1)
      {
        if (vector1.size()>0)
        {
          module_count += vector1.size();
          for (unsigned int i = 0; i < vector1.size(); ++i)
          {
            for (int j = 0; j < 4; ++j)
            {
              if(vector1.at(i)->GetNode(j)->Edge)
              {
                SmoresModulePtr ConnectedModule = vector1.at(i)->GetNode(j)->Edge->FindMatchingNode(vector1.at(i)->GetNode(j))->Parent;
                vector2.push_back(ConnectedModule);
              }
            }
          }
          vector1.clear();
          for (unsigned int i = 0; i < vector2.size(); ++i)
          {
            bool have_it = true;
            for (unsigned int j = 0; j < vector3.size(); ++j)
            {
              if (vector2.at(i) == vector3.at(j))
              {
                have_it = false;
                break;
              }
            }
            if (have_it)
            {
              vector1.push_back(vector2.at(i));
              vector3.push_back(vector2.at(i));
            }
          }
          vector2.clear();
        }else{
          break;
        }
      }
      return module_count;
    }

    private: physics::WorldPtr currentWorld;
    private: event::ConnectionPtr addEntityConnection;
    private: transport::PublisherPtr statePub;
    // private: vector<transport::PublisherPtr> WorldPublisher;
    private: vector<transport::SubscriberPtr> WorldColSubscriber;
    // The pointer vector for all the models in the world
    private: vector<SmoresModulePtr> moduleList;
    // The vectors that store the pending connection request and information
    private: vector<CollisionInformation> PendingRequest;
    // The vector for connection record
    private: vector<physics::JointPtr> DynamicConnections;
    // The event that will be refreshed in every iteration of the simulation
    private: event::ConnectionPtr updateConnection;
    // The container that has all the edges
    private: vector<SmoresEdgePtr> ConnectionEdges;
    // The indicator of a new model has been added
    private: bool NeedToSetPtr;
    //+++++++++ testing ++++++++++++++++++++++++++++
    private: int infoCounter;
    private: int numOfModules;
    };


  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ControlCenter)
}