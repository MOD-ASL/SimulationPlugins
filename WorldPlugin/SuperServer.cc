#include <sdf/sdf.hh>
#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include <string>
#include <iostream>
#include "collision_message_plus.pb.h"

#define PI 3.141598

using namespace std;

namespace gazebo
{
  typedef const boost::shared_ptr<const collision_message_plus::msgs::CollisionMessage> CollisionMessagePtr;
  string Int2String(int number) //
  {
     stringstream ss; //create a stringstream
     ss << number;    //add number to the stream
     return ss.str(); //return a string with the contents of the stream
  }
  class ControlCenter : public WorldPlugin
  {
    public: ControlCenter()
    {
      numOfModules = 0;
      infoCounter = 0;
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

      transport::NodePtr node(new transport::Node());
      node->Init();
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Welcome message generation each time a new model has been added
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      msgs::GzString welcomeMsg;
      string CurrentMessage;

      int modelnumber = this->currentWorld->GetModelCount();
      numOfModules = modelnumber-1;
      // cout<<"Number of models: "<<modelnumber<<endl;
      // cout<<"Default information: "<<_info<<endl;
      this->statePub = node->Advertise<msgs::GzString>("~/Welcome");
      CurrentMessage = "Model";
      CurrentMessage += Int2String(modelnumber);
      welcomeMsg.set_data(CurrentMessage);

      statePub->Publish(welcomeMsg);
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Dynamic publisher generation
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      transport::NodePtr node1(new transport::Node());
      node1->Init(_info);
      string NewPubName = "~/";
      NewPubName += _info;
      NewPubName += "_world";
      transport::PublisherPtr newWorldPub = node1->Advertise<msgs::Pose>(NewPubName);
      WorldPublisher.push_back(newWorldPub);

      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Dynamic subscriber of collision topic
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      string NewSubName = "~/"+_info+"_Collision";
      transport::SubscriberPtr newWorldSub = node1->Subscribe(NewSubName,&ControlCenter::CollisionMonitor, this);
      WorldColSubscriber.push_back(newWorldSub);
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function will be called in the every iteration of the simulation
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void OnSystemRunning(const common::UpdateInfo & /*_info*/)
    {
      // if (infoCounter<=2)
      // {
      //   math::Vector3 Location;
      //   math::Quaternion Rotmat;
      //   Location.Set(0.929,0.929,0.05);
      //   Rotmat.SetFromAxis(0,0,1,2.35619);
      //   math::Pose PoseInfo(Location,Rotmat);
      //   WorldPublisher.at(numOfModules-1)->Publish(gazebo::msgs::Convert(PoseInfo));
      //   infoCounter++;
      // }
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function will be called everytime receive collision information
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void CollisionMonitor(CollisionMessagePtr &msg)
    {
      // cout<<"World: First collision is : "<<msg->collision1()<<endl;
      string nameString1 = msg->collision1() + "," + msg->collision2();
      string nameString2 = msg->collision2() + "," + msg->collision1();
      string ModelOfModels1 = msg->collision1().substr(0,msg->collision1().find("::"))+","+msg->collision2().substr(0,msg->collision2().find("::"));
      string ModelOfModels2 = msg->collision2().substr(0,msg->collision2().find("::"))+","+msg->collision1().substr(0,msg->collision1().find("::"));
      math::Pose ContactLinkPos = msgs::Convert(msg->positioncol1());
      int FoundPendingOne = 0;
      for (unsigned int i = 0; i < namesOfPendingRequest.size(); ++i)
      {
        cout<<"World: Pending Array length: "<<namesOfPendingRequest.size()<<endl;
        if (namesOfPendingRequest.at(i).compare(nameString1) ==0 || namesOfPendingRequest.at(i).compare(nameString2) ==0)
        {
          physics::LinkPtr Link1, Link2;
          math::Vector3 axis;
          if (msg->collision1().substr(msg->collision1().find("::")+2,msg->collision1().rfind("::")).compare("FrontWheel")==0)
          {
            axis.Set(0,1,0);
          }

          math::Vector3 ZDirectionOffset(0,0,0.005);
          math::Vector3 newCenterPoint = 0.5*(ContactLinkPos.pos + PendingRequestPos.at(i).pos)+ZDirectionOffset;
          math::Vector3 newPositionOfLink1 = newCenterPoint + 0.051*ContactLinkPos.rot.GetYAxis();
          math::Vector3 newPositionOfLink2 = newCenterPoint - 0.051*ContactLinkPos.rot.GetYAxis();
          math::Quaternion newDirectionofLink1 = ContactLinkPos.rot;
          math::Quaternion newDirectionofLink2(-ContactLinkPos.rot.GetRoll(), PendingRequestPos.at(i).rot.GetPitch(),-(PI-ContactLinkPos.rot.GetYaw()));

          cout<<"World: Need to be set position ["<<newPositionOfLink1.x<<", "<<newPositionOfLink1.y<<", "<<newPositionOfLink1.z<<"]"<<endl;
          // cout<<"World: Model name is : "<<msg->collision1().substr(0,msg->collision1().find("::"))<<" and link name is : "<<msg->collision1().substr(msg->collision1().find("::")+2,msg->collision1().rfind("::")-msg->collision1().find("::")-2)<<endl;

          Link1 = currentWorld->GetModel(msg->collision1().substr(0,msg->collision1().find("::")))->GetLink(msg->collision1().substr(msg->collision1().find("::")+2,msg->collision1().rfind("::")-msg->collision1().find("::")-2));
          Link2 = currentWorld->GetModel(msg->collision2().substr(0,msg->collision2().find("::")))->GetLink(msg->collision2().substr(msg->collision2().find("::")+2,msg->collision2().rfind("::")-msg->collision2().find("::")-2));

          currentWorld->GetModel(msg->collision1().substr(0,msg->collision1().find("::")))->SetLinkWorldPose(math::Pose(newPositionOfLink1,newDirectionofLink1),Link1);
          currentWorld->GetModel(msg->collision2().substr(0,msg->collision2().find("::")))->SetLinkWorldPose(math::Pose(newPositionOfLink2,newDirectionofLink2),Link2);

          math::Pose newLink1PosReq = currentWorld->GetModel(msg->collision1().substr(0,msg->collision1().find("::")))->GetLink("FrontWheel")->GetWorldPose();
          cout<< "World: Position set has done."<<endl;
          cout<< "World: New Link1 Position ["<<newLink1PosReq.pos.x<<", "<<newLink1PosReq.pos.y<<", "<<newLink1PosReq.pos.z<<"]"<<endl;
          physics::JointPtr DynamicJoint;
          DynamicJoint = currentWorld->GetPhysicsEngine()->CreateJoint("revolute",  currentWorld->GetModel(msg->collision1().substr(0,msg->collision1().find("::"))));
          DynamicJoint->Attach(Link1, Link2);
          DynamicJoint->Load(Link1, Link2, math::Pose(math::Vector3(0,-0.005,0),math::Quaternion()));
          DynamicJoint->SetAxis(0, axis);
          // DynamicJoint->SetAngle(1,math::Angle(0));
          DynamicJoint->SetHighStop(1,math::Angle(0.01));
          DynamicJoint->SetLowStop(1,math::Angle(-0.01));
          DynamicConnections.push_back(DynamicJoint);
          cout<< "World: Dynamic joint has been generated."<<endl;

          existConnections.push_back(nameString1);
          existConnectedPair.push_back(ModelOfModels1);
          // cout<<"World: Connection between models: "
          namesOfPendingRequest.erase(namesOfPendingRequest.begin()+i);
          PendingRequestPos.erase(PendingRequestPos.begin()+i);

          FoundPendingOne = 1;
          // cout<<"World: Joint Generated!"<<endl;
          break;
        }
      }
      if (FoundPendingOne == 0)
      {
        for (unsigned int i = 0; i < existConnections.size(); ++i)
        {
          if (existConnections.at(i).compare(nameString1) ==0 || existConnections.at(i).compare(nameString2) ==0)
          {
            FoundPendingOne = 1;
            break;
          }
        }
        for (unsigned int i = 0; i < existConnectedPair.size(); ++i)
        {
          if (existConnectedPair.at(i).compare(ModelOfModels1) ==0 || existConnectedPair.at(i).compare(ModelOfModels2) ==0)
          {
            FoundPendingOne = 1;
            break;
          }
        }
        if (FoundPendingOne==0)
        {
          namesOfPendingRequest.push_back(nameString1);
          PendingRequestPos.push_back(ContactLinkPos);
          cout<<"World: An pending entry has been established: '"<< nameString1<<"'"<<endl;
        }
      }
    }
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function is used to check the distance between each robot
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void ready4Connection(void)
    {

    }
    private: physics::WorldPtr currentWorld;
    private: event::ConnectionPtr addEntityConnection;
    private: transport::PublisherPtr statePub;
    private: vector<transport::PublisherPtr> WorldPublisher;
    private: vector<transport::SubscriberPtr> WorldColSubscriber;
    // The pointer vector for all the models in the world
    private: vector<physics::ModelPtr> modelGroup;
    // The vectors that store the pending connection request and information
    private: vector<string> namesOfPendingRequest;
    private: vector<math::Pose> PendingRequestPos;
    // The vector that stores the real connection
    private: vector<string> existConnections;
    // The vector for connection record
    private: vector<physics::JointPtr> DynamicConnections;
    // The vector that stores the connected pair of models
    private: vector<string> existConnectedPair;
    // The event that will be refreshed in every iteration of the simulation
    private: event::ConnectionPtr updateConnection;
    //+++++++++ testing ++++++++++++++++++++++++++++
    private: int infoCounter;
    private: int numOfModules;
    };


  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ControlCenter)
}