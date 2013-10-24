#include <sdf/sdf.hh>
#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include <string>
#include <iostream>

using namespace std;

namespace gazebo
{
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
      cout<<"Number of models: "<<modelnumber<<endl;
      cout<<"Default information: "<<_info<<endl;
      this->statePub = node->Advertise<msgs::GzString>("~/Welcome");
      CurrentMessage = "Model";
      CurrentMessage += Int2String(modelnumber);
      welcomeMsg.set_data(CurrentMessage);

      statePub->Publish(welcomeMsg);
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // Dynamic publisher generation
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      string NewPubNAme = "~/";
      NewPubNAme += _info;
      NewPubNAme += "_world";
      transport::PublisherPtr newWorldPub = node->Advertise<msgs::Pose>(NewPubNAme);
      WorldPublisher.push_back(newWorldPub);


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
    // This function is used to check the distance between each robot
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void ready4Connection(void)
    {

    }
    private: physics::WorldPtr currentWorld;
    private: event::ConnectionPtr addEntityConnection;
    private: transport::PublisherPtr statePub;
    private: vector<transport::PublisherPtr> WorldPublisher;
    // The pointer vector for all the models in the world
    private: vector<physics::ModelPtr> modelGroup;
    // The event that will be refreshed in every iteration of the simulation
    private: event::ConnectionPtr updateConnection;
    //+++++++++ testing ++++++++++++++++++++++++++++
    private: int infoCounter;
    private: int numOfModules;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ControlCenter)
}