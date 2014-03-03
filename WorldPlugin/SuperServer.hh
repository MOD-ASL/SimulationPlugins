#ifndef _GAZEBO_SUPER_SERVER_HH_
#define _GAZEBO_SUPER_SERVER_HH_

#include <sdf/sdf.hh>
#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include <string>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
// Libraries for messages needed to use to communicate between plugins
#include "collision_message_plus.pb.h"
// #include "command_message.pb.h"
// Libraries for connectivity representation
#include "SmoresModule.hh"
// XML paser library
#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"

#define PI 3.141593   // 3.1411593
#define VALIDCONNECTIONDISUPPER 0.110
#define VALIDCONNECTIONDISLOWER 0.098
#define MODULEPATH "SMORE.sdf"  // Depende on current execution folder, I think
#define INTIALCONFIGURATION "InitialConfiguration"

using namespace std;
using namespace rapidxml;

string Int2String(int number)
{
  stringstream ss; //create a stringstream
  ss << number;    //add number to the stream
  return ss.str(); //return a string with the contents of the stream
}

namespace gazebo
{
	typedef const boost::shared_ptr<const collision_message_plus::msgs::CollisionMessage> CollisionMessagePtr;
  typedef const boost::shared_ptr<const command_message::msgs::CommandMessage> CommandMessagePtr;

  class CollisionInformation
  {
  //++++++++++++++ Class Methods +++++++++++++++++++++++++++++++++
  public:
    CollisionInformation(string collision1, string collision2, string link_collision1, string link_collision2);

    bool SameCollision(string collision1, string collision2, string link_collision1, string link_collision2);
    
  //++++++++++++++ Class Members +++++++++++++++++++++++++++++++++
  public:
    string Model1;
    string Model2;
    string LinkOfModel1;
    string LinkofModel2;
  };

  class ControlCenter : public WorldPlugin
  {
    public: ControlCenter();

    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    
    private: void addEntity2World(std::string & _info);
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function will be called in the every iteration of the simulation
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void OnSystemRunning(const common::UpdateInfo & /*_info*/);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function is used to build initial configuration using a XML file
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void BuildConfigurationFromXML(void);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function is used to build connection using a XML file
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void BuildConnectionFromXML(void);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function will be called everytime receive command information
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void FeedBackMessageDecoding(CommandMessagePtr &msg);
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function will be called everytime receive collision information
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void AutomaticMagneticConnectionManagement(CollisionMessagePtr &msg);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // These function are used to physically connect different models by generating dynamic joint
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void ConnectAndDynamicJointGeneration(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, SmoresEdgePtr an_edge);
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function is used to physically destroy the connection between different modules, which is dynamic joint here
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void DynamicJointDestroy(SmoresEdgePtr aEdge);
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // This function is used to insert a model to the current world
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public: void InsertModel(string name, math::Pose position);

    public: void InsertModel(string name, math::Pose position, string joint_angles);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // These functions are used to connect or deconnect modules
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Connect two modules by pointers and node_ID
    public: void PassiveConnection(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, double node_angle = 0, double node_distance = 0);

    public: void ActiveConnection(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, double node_angle = 0, double node_distance = 0);

    // Deconnect two modules on one edge
    public: void Deconnection(SmoresEdgePtr aEdge);  // This pointer must point to an element in the vector
    
    // Deconnect two module base on one module and one node of that module
    public: void Deconnection(SmoresModulePtr aModule, int node_ID);

    // Deconnect two module base on one module and one node of that module
    public: void Deconnection(string moduleName, int node_ID);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // These functions are used to manage and send message to model
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: void CommandManager(void);
    
    public: void SendGaitTable(SmoresModulePtr module, bool flag[4], double gait_value[4], int priority = 0, int msg_type = 3);
 
    public: void SendGaitTable(SmoresModulePtr module, int joint_ID, double gait_value, int priority = 0, int msg_type = 3);

    public: void SendPosition(SmoresModulePtr module, double x, double y, double orientation_angle, int priority = 0);
    
    // Those commands are not recommended for sending the gait table or position coordinates
    // But could be used in the direct driving situation
    public: void SendGaitTableInstance(SmoresModulePtr module, bool flag[4], double gait_value[4], int msg_type = 4);

    public: void SendPositionInstance(SmoresModulePtr module, double x, double y, double orientation_angle);

    // Erase all the existing commands of a specific module
    public: void EraseComaands(SmoresModulePtr module);  // Need to be tested

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // These functions are utility functions
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    public: int GetNodeIDByName(string node_name);

    public: SmoresModulePtr GetModulePtrByName(string module_name);

    // public: ModuleCommandsPtr GetCommandPtrByModule(SmoresModulePtr module_ptr)
    // {
    //   ModuleCommandsPtr command_squence;
    //   for (unsigned int i = 0; i < ModuleCommandContainer.size(); ++i)
    //   {
    //     if (module_ptr == ModuleCommandContainer.at(i)->WhichModule)
    //     {
    //       command_squence = ModuleCommandContainer.at(i);
    //       break;
    //     }
    //   }
    //   return command_squence;
    // }
    public: void EraseCommandPtrByModule(SmoresModulePtr module_ptr);
 
    public: int GetModuleIDXByName(string module_name);

    // public: void SetThePointerInSmoresModule(void)
    // {
    //   if (NeedToSetPtr)
    //   {
    //     for (unsigned int i = 0; i < moduleList.size(); ++i)
    //     {
    //       moduleList.at(i)->SetModulePtr(currentWorld->GetModel(moduleList.at(i)->ModuleID));
    //     }
    //     NeedToSetPtr = false;
    //     cout<<"World: Pointer has been assigned"<<endl;
    //   }
    // }
    // Check whether two nodes are connected together
    public: bool AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID);
    
    // Check whether two modules have already connected on a node
    public: bool AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2);
    
    // Check whether a node a of module has been occupied
    public: bool AlreadyConnected(SmoresModulePtr module, int node_ID);
    
    // This function is used to count for a configuration, how many modules are there
    private: unsigned int CountModules(SmoresModulePtr module);
    
    // This function is only for demonstration
    void readFileAndGenerateCommands(const char* fileName);

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
    private: int NeedToSetPtr;
    // A String vector which contain the initial joint angles of modules
    private: vector<string> InitalJointValue;
    private: vector<math::Pose> InitialPosition;
    
    private: vector<ModuleCommandsPtr> ModuleCommandContainer;
    //+++++++++ testing ++++++++++++++++++++++++++++
    private: int infoCounter;
    private: int numOfModules;
    private: int test_count;
    bool insertModuleFlag;
    bool AlreadyBuild;
    // private: bool FinishFlag;
    // vector<std::array<double,4>> jointCommands;
    double jointCommands[10][4];
    vector<int> relatedModules;
   };
}
#endif