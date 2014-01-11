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
// Libraries for meesages needed to use to communicate between bplugins
#include "collision_message_plus.pb.h"
#include "command_message.pb.h"
// Libraries for connectivity representation
// #include "SmoresEdge.hh"
// #include "SmoresNode.hh"
#include "SmoresModule.hh"

#define PI 3.141593   // 3.1411593
#define VALIDCONNECTIONDISUPPER 0.110
#define VALIDCONNECTIONDISLOWER 0.095

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
      // Store the pointers of model and model names into vectors
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // modelGroup.push_back(currentWorld->GetModel(_info));
      modelNameGroup.push_back(_info);

      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Dynamic publisher generation
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      transport::NodePtr node1(new transport::Node());
      node1->Init(_info);
      string NewPubName = "~/" + _info + "_world";
      transport::PublisherPtr newWorldPub = node1->Advertise<command_message::msgs::CommandMessage>(NewPubName);
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
      string ModelOfCollision1 = msg->collision1().substr(0,msg->collision1().find("::"));
      string ModelOfCollision2 = msg->collision2().substr(0,msg->collision2().find("::"));
      string LinkOfCollision1 = msg->collision1().substr(msg->collision1().find("::")+2,msg->collision1().rfind("::")-msg->collision1().find("::")-2);
      string LinkOfCollision2 = msg->collision2().substr(msg->collision2().find("::")+2,msg->collision2().rfind("::")-msg->collision2().find("::")-2);
      string ModelOfModels1 = ModelOfCollision1+","+ModelOfCollision2;
      string ModelOfModels2 = ModelOfCollision2+","+ModelOfCollision1;

      math::Pose ContactLinkPos = msgs::Convert(msg->positioncol1());
      int FoundPendingOne = 0;
      for (unsigned int i = 0; i < namesOfPendingRequest.size(); ++i)
      {
        // cout<<"World: Pending Array length: "<<namesOfPendingRequest.size()<<endl;
        // if (namesOfPendingRequest.at(i).compare(nameString1) ==0 || namesOfPendingRequest.at(i).compare(nameString2) ==0)
        if (namesOfPendingRequest.at(i).compare(nameString2) ==0)
        {
          int InAConnectionGroup = 0;
          cout<<"World: First model is: "<<ModelOfCollision1<<endl;
          for (unsigned int m = 0; m < existConnectionGroups.size(); ++m)
          {
            if (existConnectionGroups.at(m).find(ModelOfCollision1)!=string::npos)
            {
              InAConnectionGroup = 1;
              break;
            }
          }
          if (InAConnectionGroup != 1)
          {
            string tmpString = ModelOfCollision1;
            ModelOfCollision1 = ModelOfCollision2;
            ModelOfCollision2 = tmpString;
            tmpString = LinkOfCollision1;
            LinkOfCollision1 = LinkOfCollision2;
            LinkOfCollision2 = tmpString;
            math::Pose tmpPose = ContactLinkPos;
            ContactLinkPos = PendingRequestPos.at(i);
            PendingRequestPos.at(i) = tmpPose;
          }
          cout<<"World: Now the first model became: "<<ModelOfCollision1<<endl;
          physics::LinkPtr Link1, Link2;
          math::Vector3 axis;
          math::Vector3 ZDirectionOffset(0,0,0.000);  //0.008
          math::Vector3 newCenterPoint = 0.5*(ContactLinkPos.pos + PendingRequestPos.at(i).pos)+ZDirectionOffset;
          math::Vector3 newPositionOfLink1;
          math::Vector3 newPositionOfLink2;
          math::Quaternion newDirectionofLink1;
          math::Vector3 newZAxis;
          double AngleBetweenZAxes;
          math::Quaternion FirstRotationOfLink2;
          math::Quaternion SecondRotationOfLink2;
          math::Quaternion newDirectionofLink2;

          if (LinkOfCollision1.compare("FrontWheel")==0)
          {
          //   newPositionOfLink1 = newCenterPoint + 0.0495*ContactLinkPos.rot.GetYAxis();
          //   newPositionOfLink2 = newCenterPoint - 0.0495*ContactLinkPos.rot.GetYAxis();
            newPositionOfLink1 = ContactLinkPos.pos;
            newPositionOfLink2 = ContactLinkPos.pos - 0.0998*ContactLinkPos.rot.GetYAxis();
            newDirectionofLink1 = ContactLinkPos.rot;
            cout<<"World: 'newDirectionofLink1' Z axis [ "<<newDirectionofLink1.GetZAxis().x<<", "<<newDirectionofLink1.GetZAxis().y<<", "<<newDirectionofLink1.GetZAxis().z<<"]"<<endl;
            cout<<"World: 'newDirectionofLink1' 'Yaw' is : "<<newDirectionofLink1.GetYaw()<<endl;
            // cout<<"World: 'newDirectionofLink1' Y axis [ "<<newDirectionofLink1.GetYAxis().x<<", "<<newDirectionofLink1.GetYAxis().y<<", "<<newDirectionofLink1.GetYAxis().z<<"]"<<endl;
            // cout<<"World: 'newDirectionofLink1' 'Pitch' is : "<<newDirectionofLink1.GetPitch()<<endl;
            // cout<<"World: 'referenceQuaternion' Y axis [ "<<referenceQuaternion.GetYAxis().x<<", "<<referenceQuaternion.GetYAxis().y<<", "<<referenceQuaternion.GetYAxis().z<<"]"<<endl;
            // cout<<"World: 'referenceQuaternion' 'Pitch' is : "<<referenceQuaternion.GetPitch()<<endl;
            cout<<"World: 'PendingRequestPos' Z axis [ "<<PendingRequestPos.at(i).rot.GetZAxis().x<<", "<<PendingRequestPos.at(i).rot.GetZAxis().y<<", "<<PendingRequestPos.at(i).rot.GetZAxis().z<<"]"<<endl;
            cout<<"World: 'PendingRequestPos' 'Yaw' is : "<<PendingRequestPos.at(i).rot.GetYaw()<<endl;
            newZAxis = PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetXAxis())*newDirectionofLink1.GetXAxis();
            newZAxis = newZAxis.Normalize();
            AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis()));
            FirstRotationOfLink2.SetFromEuler(0,0,PI);
            double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
            // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
            if (DirectionReference>0)
            {
              SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
            }else{
              SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
            }
            if (LinkOfCollision2.compare("UHolderBody")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,0);
              cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<endl;
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              }else{
                SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
              }
            }
            if (LinkOfCollision2.compare("LeftWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
              cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<endl;
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
              }else{
                SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
              }
            }
            if (LinkOfCollision2.compare("RightWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
              cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<endl;
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
              }else{
                SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
              }
            }
            // math::Quaternion newDirectionofLink2(newZAxis, PI+newDirectionofLink1.GetYaw());
            newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
            // cout<<"World: 'FirstRotationOfLink2' Y axis [ "<<FirstRotationOfLink2.GetYAxis().x<<", "<<FirstRotationOfLink2.GetYAxis().y<<", "<<FirstRotationOfLink2.GetYAxis().z<<"]"<<endl;
            // cout<<"World: 'FirstRotationOfLink2' 'Pitch' is : "<<FirstRotationOfLink2.GetPitch()<<endl;
            // cout<<"World: 'newDirectionofLink2' Y axis [ "<<newDirectionofLink2.GetYAxis().x<<", "<<newDirectionofLink2.GetYAxis().y<<", "<<newDirectionofLink2.GetYAxis().z<<"]"<<endl;
            // cout<<"World: 'newDirectionofLink2' 'Pitch' is : "<<newDirectionofLink2.GetPitch()<<endl;

            // cout<<"World: Need to be set position ["<<newPositionOfLink1.x<<", "<<newPositionOfLink1.y<<", "<<newPositionOfLink1.z<<"]"<<endl;
            // cout<<"World: Model name is : "<<msg->collision1().substr(0,msg->collision1().find("::"))<<" and link name is : "<<msg->collision1().substr(msg->collision1().find("::")+2,msg->collision1().rfind("::")-msg->collision1().find("::")-2)<<endl;
            axis.Set(0,1,0);
          }
          if (LinkOfCollision1.compare("LeftWheel")==0)
          {
            newPositionOfLink1 = ContactLinkPos.pos;
            newPositionOfLink2 = ContactLinkPos.pos + 0.0998*ContactLinkPos.rot.GetXAxis();
            newDirectionofLink1 = ContactLinkPos.rot;
            newZAxis = PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetYAxis())*newDirectionofLink1.GetYAxis();
            newZAxis = newZAxis.Normalize();
            AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis()));
            FirstRotationOfLink2.SetFromEuler(0,0,PI);
            double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
            // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
            if (DirectionReference>0)
            {
              SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
            }else{
              SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
            }
            if (LinkOfCollision2.compare("FrontWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
              cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<endl;
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              }else{
                SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
              }
            }
            if (LinkOfCollision2.compare("UHolderBody")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
              // SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              }else{
                SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
              }
            }
            if (LinkOfCollision2.compare("RightWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,-PI);
              // SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
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
          if (LinkOfCollision1.compare("RightWheel")==0)
          {
            newPositionOfLink1 = ContactLinkPos.pos;
            newPositionOfLink2 = ContactLinkPos.pos + 0.0998*ContactLinkPos.rot.GetXAxis();
            newDirectionofLink1 = ContactLinkPos.rot;
            newZAxis = PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetYAxis())*newDirectionofLink1.GetYAxis();
            newZAxis = newZAxis.Normalize();
            AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis()));
            FirstRotationOfLink2.SetFromEuler(0,0,PI);
            double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
            // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
            if (DirectionReference>0)
            {
              SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
            }else{
              SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
            }
            if (LinkOfCollision2.compare("FrontWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
              cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<endl;
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              }else{
                SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
              }
            }
            if (LinkOfCollision2.compare("UHolderBody")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
              // SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              }else{
                SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
              }
            }
            if (LinkOfCollision2.compare("LeftWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,-PI);
              // SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
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
          if (LinkOfCollision1.compare("UHolderBody")==0)
          {
            newPositionOfLink1 = ContactLinkPos.pos;
            newPositionOfLink2 = ContactLinkPos.pos + 0.0998*ContactLinkPos.rot.GetYAxis();
            newDirectionofLink1 = ContactLinkPos.rot;
            newZAxis = PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PendingRequestPos.at(i).rot.GetZAxis().Dot(newDirectionofLink1.GetXAxis())*newDirectionofLink1.GetXAxis();
            newZAxis = newZAxis.Normalize();
            AngleBetweenZAxes = acos(newZAxis.Dot(newDirectionofLink1.GetZAxis()));
            FirstRotationOfLink2.SetFromEuler(0,0,PI);
            double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
            // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
            if (DirectionReference>0)
            {
              SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
            }else{
              SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
            }
            if (LinkOfCollision2.compare("FrontWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,0);
              cout<<"World: Calibrate angle: "<<AngleBetweenZAxes<<endl;
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetYAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              }else{
                SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
              }
            }
            if (LinkOfCollision2.compare("LeftWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
              // SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
              if (DirectionReference>0)
              {
                SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
              }else{
                SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
              }
            }
            if (LinkOfCollision2.compare("RightWheel")==0)
            {
              FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
              // SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
              DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PendingRequestPos.at(i).rot.GetXAxis());
              // SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
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

          Link1 = currentWorld->GetModel(ModelOfCollision1)->GetLink(LinkOfCollision1);
          Link2 = currentWorld->GetModel(ModelOfCollision2)->GetLink(LinkOfCollision2);

          currentWorld->GetModel(ModelOfCollision1)->SetLinkWorldPose(math::Pose(newPositionOfLink1,newDirectionofLink1),Link1);
          currentWorld->GetModel(ModelOfCollision2)->SetLinkWorldPose(math::Pose(newPositionOfLink2,newDirectionofLink2),Link2);

          math::Pose newLink1PosReq = currentWorld->GetModel(ModelOfCollision1)->GetLink("FrontWheel")->GetWorldPose();
          // cout<< "World: Position set has done."<<endl;
          // cout<< "World: New Link1 Position ["<<newLink1PosReq.pos.x<<", "<<newLink1PosReq.pos.y<<", "<<newLink1PosReq.pos.z<<"]"<<endl;
          physics::JointPtr DynamicJoint;
          DynamicJoint = currentWorld->GetPhysicsEngine()->CreateJoint("revolute",  currentWorld->GetModel(ModelOfCollision1));
          DynamicJoint->Attach(Link1, Link2);
          DynamicJoint->Load(Link1, Link2, math::Pose(math::Vector3(0,-0.00,0),math::Quaternion()));
          DynamicJoint->SetAxis(0, axis);
          DynamicJoint->SetName("Dynamic_Joint");
          currentWorld->GetModel(ModelOfCollision1)->GetJointController()->AddJoint(DynamicJoint);
          // DynamicJoint->Init();
          // cout<<"World: The parent of the new joint is '"<<DynamicJoint->GetParent()->GetName()<<"'"<<endl; 
          // cout<<"World: The children of the new joint is '"<<DynamicJoint->GetChild()->GetName()<<"'"<<endl; 
          DynamicJoint->SetAngle(0,math::Angle(0));
          DynamicJoint->SetHighStop(0,math::Angle(0.01));
          DynamicJoint->SetLowStop(0,math::Angle(-0.01));
          DynamicConnections.push_back(DynamicJoint);
          // cout<< "World: Dynamic joint has been generated."<<endl;

          // After finishing the connection
          existConnections.push_back(nameString1);
          existConnectedPair.push_back(ModelOfModels1);
          int InOneOfTheGroup = 0;
          for (unsigned int m = 0; m < existConnectionGroups.size(); ++m)
          {
            if (existConnectionGroups.at(m).find(ModelOfCollision1) != string::npos)
            {
              if (existConnectionGroups.at(m).at(existConnectionGroups.at(m).find(ModelOfCollision1)+ModelOfCollision1.size())==',')
              {
                int InOtherGroup = 0;
                for (unsigned int j = 0; j < existConnectionGroups.size(); ++j)
                {
                  if (existConnectionGroups.at(j).find(ModelOfCollision2) != string::npos)
                  {
                    if (existConnectionGroups.at(j).at(existConnectionGroups.at(j).find(ModelOfCollision2)+ModelOfCollision2.size())==',')
                    {
                      existConnectionGroups.at(m) += ','+existConnectionGroups.at(j);
                      existConnectionGroups.erase(existConnectionGroups.begin()+j);
                      InOtherGroup = 1;
                      break;
                    }
                  }
                }
                if (InOtherGroup == 0)
                {
                  existConnectionGroups.at(m) += ','+ModelOfCollision2;
                }
                InOneOfTheGroup = 1;
                break;
              }
            }
            if (existConnectionGroups.at(m).find(ModelOfCollision2) != string::npos)
            {
              if (existConnectionGroups.at(m).at(existConnectionGroups.at(m).find(ModelOfCollision2)+ModelOfCollision2.size())==',')
              {
                int InOtherGroup = 0;
                for (unsigned int j = 0; j < existConnectionGroups.size(); ++j)
                {
                  if (existConnectionGroups.at(j).find(ModelOfCollision1) != string::npos)
                  {
                    if (existConnectionGroups.at(j).at(existConnectionGroups.at(j).find(ModelOfCollision1)+ModelOfCollision1.size())==',')
                    {
                      existConnectionGroups.at(m) += ','+existConnectionGroups.at(j);
                      existConnectionGroups.erase(existConnectionGroups.begin()+j);
                      InOtherGroup = 1;
                      break;
                    }
                  }
                }
                if (InOtherGroup == 0)
                {
                  existConnectionGroups.at(m) += ','+ModelOfCollision1;
                }
                InOneOfTheGroup = 1;
                break;
              }
            }
          }
          if (InOneOfTheGroup==0)
          {
            existConnectionGroups.push_back(ModelOfModels1);
          }
          // +++++  Testing Script ++++++++++++++++++++++
          for (unsigned int m = 0; m < existConnectionGroups.size(); ++m)
          {
            cout<<"Wprld: One of the connected groups is: "<<existConnectionGroups.at(m)<<endl;
          }
          // // ++++++++++++++++++++++++++++++++++++++++++++
          // cout<<"World: Connection between models: "
          namesOfPendingRequest.erase(namesOfPendingRequest.begin()+i);
          PendingRequestPos.erase(PendingRequestPos.begin()+i);

          // Publish the connection information to model
          int InfoPushedCount;
          for (unsigned int m = 0; m < modelNameGroup.size(); ++m)
          {
            if (modelNameGroup.at(m).compare(ModelOfCollision1)==0)
            {
              command_message::msgs::CommandMessage ConnectionMessage;
              ConnectionMessage.set_messagetype(1);
              ConnectionMessage.set_whichmodelconnectedto(ModelOfCollision2);
              WorldPublisher.at(m)->Publish(ConnectionMessage);
              InfoPushedCount++;
              if (InfoPushedCount==2)
              {
                break;
              }
            }
            if (modelNameGroup.at(m).compare(ModelOfCollision2)==0)
            {
              command_message::msgs::CommandMessage ConnectionMessage;
              ConnectionMessage.set_messagetype(1);
              ConnectionMessage.set_whichmodelconnectedto(ModelOfCollision2);
              WorldPublisher.at(m)->Publish(ConnectionMessage);
              InfoPushedCount++;
              if (InfoPushedCount==2)
              {
                break;
              }
            }
          }

          FoundPendingOne = 1;
          // cout<<"World: Joint Generated!"<<endl;
          break;
        }
      }
      if (FoundPendingOne == 0)
      {
        for (unsigned int i = 0; i < namesOfPendingRequest.size(); ++i)
        {
          // Mistake in the condition
          if (namesOfPendingRequest.at(i).find(nameString1.substr(0,nameString1.find(","))) !=string::npos || namesOfPendingRequest.at(i).find(nameString1.substr(nameString1.find(",")+1,-1)) !=string::npos || (namesOfPendingRequest.at(i).find(ModelOfCollision1) != string::npos && namesOfPendingRequest.at(i).find(ModelOfCollision2) != string::npos))
          {
            // cout<<"World: Connection discard because of One part already in pending"<<endl;
            FoundPendingOne = 1;
            break;
          }
        }
        for (unsigned int i = 0; i < existConnections.size(); ++i)
        {
          if (existConnections.at(i).find(nameString1.substr(0,nameString1.find(","))) !=string::npos || existConnections.at(i).find(nameString1.substr(nameString1.find(",")+1,-1)) !=string::npos)
          {
            // cout<<"World: Connection discard because of One component has been connected"<<endl;
            FoundPendingOne = 1;
            break;
          }
        }
        for (unsigned int i = 0; i < existConnectedPair.size(); ++i)
        {
          if (existConnectedPair.at(i).compare(ModelOfModels1) ==0 || existConnectedPair.at(i).compare(ModelOfModels2) ==0)
          {
            // cout<<"World: Connection discard because of two model already connected"<<endl;
            FoundPendingOne = 1;
            break;
          }
          // if (existConnectedPair.at(i).substr(0,existConnectedPair.at(i).find(",")).compare(ModelOfCollision1)==0)
          // {

          // }else{
          //   if (existConnectedPair.at(i).substr(existConnectedPair.at(i).find(",")+1,-1).compare(ModelOfCollision1)==0)
          //   {
          //     /* code */
          //   }
          // }
        }
        if (FoundPendingOne==0)
        {
          // Check the distance between the center of the two models
          math::Vector3 CenterModel1 = currentWorld->GetModel(ModelOfCollision1)->GetWorldPose().pos;
          math::Vector3 CenterModel2 = currentWorld->GetModel(ModelOfCollision2)->GetWorldPose().pos;
          // cout<<"World: Distance between centers: "<<(CenterModel1-CenterModel2).GetLength()<<endl;
          if((CenterModel1-CenterModel2).GetLength()<VALIDCONNECTIONDISUPPER && (CenterModel1-CenterModel2).GetLength()>VALIDCONNECTIONDISLOWER)
          {
          //
            cout<<"World: Distance between centers: "<<(CenterModel1-CenterModel2).GetLength()<<endl;
            namesOfPendingRequest.push_back(nameString1);
            PendingRequestPos.push_back(ContactLinkPos);
            cout<<"World: An pending entry has been established: '"<< nameString1<<"'"<<endl;
          }
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
    private: vector<string> modelNameGroup;
    // The vectors that store the pending connection request and information
    private: vector<string> namesOfPendingRequest;
    private: vector<math::Pose> PendingRequestPos;
    // The vector that stores the real connection
    private: vector<string> existConnections;
    // The vector that stores name of the models that all connect togather
    private: vector<string> existConnectionGroups;
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