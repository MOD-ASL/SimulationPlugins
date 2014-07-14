//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: This is a world plugin template for this simulation that
//              provides an interface to serve as a middle ware. The main 
//              functions of this plugin are managing magnetic connections,
//              loading configurations, interpreting gait commands, managing 
//              and routing communications, providing APIs to manage the
//              shared libraries, managing module configurations, etc.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_WORLD_SERVER_HH_
#define _GAZEBO_WORLD_SERVER_HH_
#ifndef _GAZEBO_CUTOMIZED_WORLD_PLUGIN
#define _GAZEBO_CUTOMIZED_WORLD_PLUGIN WorldServer
#endif
#include <stdlib.h>

#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <boost/bind.hpp>
#include <queue>
#include <unistd.h>

#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
// Libraries for messages needed to use to communicate between plugins
#include "collision_message.pb.h"
#include "command_message.pb.h"
// XML paser libraries
#include "rapidxml.hpp"
#include "rapidxml_utils.hpp"
// Libraries for connectivity representation
#include "SmoresModule.hh"
// Other useful classes for storing information
#include "CollisionInformation.hh"
#include "Condition.hh"
// Library for colored log text
#include "ColorLog.hh"
// Libraries for configuration motion feedback control
#include "LibraryTemplate.hh"

#define PI 3.141593   // 3.1411593
#define VALIDCONNECTIONDISUPPER 0.110
#define VALIDCONNECTIONDISLOWER 0.098
// TODO: tests needed to make sure it depende on current execution folder
#define MODULEPATH "SMORE.sdf"
// #define INTIALCONFIGURATION "InitialConfiguration"

using std::string;
using std::vector;

typedef const boost::shared_ptr
    <const collision_message::msgs::CollisionMessage> CollisionMessagePtr;
typedef const boost::shared_ptr
    <const command_message::msgs::CommandMessage> CommandMessagePtr;
typedef boost::shared_ptr<gazebo::Condition> ConditionPtr;

// TODO: Considering 
namespace {
inline string Int2String(int number)
{
  stringstream ss; //create a stringstream
  ss << number;    //add number to the stream
  return ss.str(); //return a string with the contents of the stream
} // Int2String
} // namespace 
namespace gazebo
{
class WorldServer : public WorldPlugin
{
 public:
  WorldServer();
  ~WorldServer();
  /// This function will be called in Load() to perform extra initialization
  virtual void ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf);
  /// Load a shared library in linux
  LibraryTemplate *DynamicallyLoadedLibrary(const char* library_path, 
      void *lib_handle);
  /// Close the loaded libraries
  void CloseLoadedLibrary(void **lib_handle);
  virtual void OnSystemRunningExtra(const common::UpdateInfo & _info);
  /// Insert a model to the current world
  virtual void InsertModel(string name, math::Pose position);
  /// Insert a model to the current world, with joint angles specified
  virtual void InsertModel(string name, math::Pose position, string joint_angles);
  /// Add a new position to set at the end of 'intialPosition' vector
  void AddInitialPosition(math::Pose position);
  /// Add new initial joint values to set at the end of 'initalJointValue' vector
  void AddInitialJoints(string joint_angles);
  /// Delete a model that already in the world
  void DeleteModule(string module_name);
  /// This function is used to build a configuration using a XML file
  void BuildConfigurationFromXML(string file_name);
  /// This function is used to build connection using a XML file
  void BuildConnectionFromXML(string file_name);
  /// This function will be called after set the model initial position
  virtual void ExtraWorkWhenModelInserted(CommandMessagePtr &msg);
  /// Convert angles so that their absolutely value always smaller than Pi 
  double ConversionForAngleOverPi(double angle);
  /// Enable auto magnetic connection, all module
  /// Have to be called in Load()
  /// Default: disabled
  void EnableAutoMagneticConnection(void);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // These functions are used to connect or deconnect modules
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Connect two modules by pointers and node_ID
  void PassiveConnect(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID, double node_angle, double node_distance);
  /// With default angle and distance offset equal to 0
  void PassiveConnect(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID);
  /// TODO: Need add 'active' feature to this function
  void ActiveConnect(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID, double node_angle, double node_distance);
  /// With default angle and distance offset equal to 0
  void ActiveConnect(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID);
  /// Disconnect two modules on one edge
  /// TODO: This pointer must point to an element in the vector
  void Disconnect(SmoresEdgePtr aEdge);
  /// Disconnect two module base on one module and one node of that module
  void Disconnect(SmoresModulePtr aModule, int node_ID);
  /// Disconnect two module base on one module and one node of that module
  void Disconnect(string moduleName, int node_ID);
  /// Disconnect two module base on their names
  void Disconnect(string moduleName1, string moduleName2);

  /// Theses functions are used to send 'gait table'
  /// TODO: Think about how to combine these functions
  void SendGaitTable(SmoresModulePtr module, const bool *flag, 
      const double *gait_value, int msg_type, unsigned int time_stamp, 
      string condition_str, string dependency_str);
  void SendGaitTable(SmoresModulePtr module, const bool *flag, 
      const double *gait_value, int msg_type, unsigned int time_stamp);
  void SendGaitTable(SmoresModulePtr module, const bool *flag, 
      const double *gait_value, int msg_type, 
      string condition_str, string dependency_str);
  void SendGaitTable(SmoresModulePtr module, const bool *flag, 
      const double *gait_value, int msg_type);
  void SendGaitTable(SmoresModulePtr module, int joint_ID, 
      double gait_value, int msg_type, unsigned int time_stamp, 
      string condition_str, string dependency_str);
  void SendGaitTable(SmoresModulePtr module, int joint_ID, 
      double gait_value, int msg_type, unsigned int time_stamp);
  void SendGaitTable(SmoresModulePtr module, int joint_ID, 
      double gait_value, int msg_type, 
      string condition_str, string dependency_str);
  void SendGaitTable(SmoresModulePtr module, int joint_ID, 
      double gait_value, int msg_type);
  void SendGaitTable(SmoresModulePtr module, string module1, string module2, 
      int node1, int node2, int commandtype, unsigned int time_stamp, 
      string condition_str, string dependency_str);
  void SendGaitTable(SmoresModulePtr module, string module1, string module2, 
      int node1, int node2, int commandtype, unsigned int time_stamp);
  void SendGaitTable(SmoresModulePtr module, string module1, string module2, 
      int node1, int node2, int commandtype, 
      string condition_str, string dependency_str);
  void SendGaitTable(SmoresModulePtr module, string module1, string module2, 
      int node1, int node2, int commandtype);
  /// These two functions can only be used in the direct driving situation
  void SendGaitTableInstance(SmoresModulePtr module, const bool *flag, 
      const double *gait_value, int msg_type);
  void SendGaitTableInstance(SmoresModulePtr module, const bool *flag, 
      const double *gait_value);
  void SendPositionInstance(SmoresModulePtr module, double x, double y, 
      double orientation_angle);
  /// Erase all the existing commands of a specific module
  /// TODO: Need to be tested
  void EraseComaands(SmoresModulePtr module);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // These functions are utility functions
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  int GetNodeIDByName(string node_name);
  /// Get SmoresModule object by specifying the name
  SmoresModulePtr GetModulePtrByName(string module_name);
  /// Get SmoresModule object by specifying the index in the vector moduleList
  SmoresModulePtr GetModulePtrByIDX(unsigned int idx);
  /// Get the count of the modules that are in the list
  unsigned int GetModuleListSize(void);
  void EraseCommandPtrByModule(SmoresModulePtr module_ptr);
  int GetModuleIndexByName(string module_name);
  /// Check whether two nodes are connected together
  bool AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2, 
      int node1_ID, int node2_ID);
  /// Check whether two modules have already connected on a node
  bool AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2);
  /// Check whether a node a of module has been occupied
  bool AlreadyConnected(SmoresModulePtr module, int node_ID);
  /// Read a 'gait table' stored in a text file
  void ReadFileAndGenerateCommands(const char* fileName);
  /// Used to interpret the number in gait table 
  void FigureInterpret(string joints_spec, bool *type_flags, double *joint_values);
  /// Interpret normal command string
  void InterpretCommonGaitString(string a_command_str);
  /// Interpret special command
  void InterpretSpecialString(string a_command_str);
  /// Count how many modules are there in a cluster
  unsigned int CountModules(SmoresModulePtr module);
  /// Get the length of the initial joint value setting sequence
  unsigned int GetInitialJointSequenceSize(void);

 private: 
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  /// Callback when there is an entity added to the world
  void AddEntityToWorld(std::string & _info);
  /// This function will be called in the every iteration of the simulation
  void OnSystemRunning(const common::UpdateInfo & /*_info*/);
  /// Command information receiving callback
  void FeedBackMessageDecoding(CommandMessagePtr &msg);
  /// Collision information receiving callback
  /// Used by automatic magnetic connection
  /// TODO: Need to be enabled by each individual module
  void AutomaticMagneticConnectionManagement(CollisionMessagePtr &msg);
  /// The function is used to physically connect different models 
  /// by generating dynamic joint
  void ConnectAndDynamicJointGeneration(SmoresModulePtr module_1, 
      SmoresModulePtr module_2, int node1_ID, int node2_ID, SmoresEdgePtr an_edge);
  /// Calculate the rotation matrix of cluster 2
  /// Used in 'ConnectAndDynamicJointGeneration'
  void RotationQuaternionCalculation(math::Vector3 normal_axis,
    math::Vector3 z_axis_of_link1, math::Vector3 z_axis_of_link2, 
    math::Vector3 first_rotation, math::Vector3 second_rotation,
    math::Quaternion *first_rotation_of_link2, 
    math::Quaternion *second_rotation_of_link2);
  /// Calculate the new position of the connecting modules
  void NewPositionCalculation(SmoresEdgePtr an_edge,
    math::Pose old_pose_of_module1, math::Pose old_pose_of_module2, 
    int node1_ID, int node2_ID, 
    math::Pose *new_pose_of_module1, math::Pose *new_pose_of_module2);
  /// Destroyer of the connection between different modules,
  /// which is dynamic joint here
  void DynamicJointDestroy(SmoresEdgePtr aEdge);
  /// These functions are used to manage and send message to model
  void CommandManager(void); 
  void CommandExecution(ModuleCommandsPtr current_command_container);
  /// Condition manipulation
  void AddCondition(string conditionid);
  void FinishOneConditionCommand(string conditionid);
  bool CheckCondition(string conditionid);
  /// Find the timer in a command string
  int StripOffTimerInCommandString(string &command_string);
  /// Find the condition in a command string
  string StripOffCondition(string &command_string);
  /// Find the dpendency in a command string
  string StripOffDependency(string &command_string);

 public:
  physics::WorldPtr currentWorld;
 private: 
  event::ConnectionPtr addEntityConnection;
  transport::PublisherPtr welcomePub;
  /// The pointer vector for all the models in the world
  vector<SmoresModulePtr> moduleList;
  /// The vectors that store the pending connections request and information
  vector<CollisionInformation> pendingRequest;
  /// The vector for connection record
  vector<physics::JointPtr> dynamicConnections;
  /// The event that will be refreshed in every iteration of the simulation
  event::ConnectionPtr updateConnection;
  /// The container that has all the edges
  vector<SmoresEdgePtr> connectionEdges;
  /// The indicator of a new model has been added
  /// TODO: A better mechnisam should be designed
  int needToSetPtr;
  /// A String vector which contain the initial joint angles of modules
  vector<string> initalJointValue;
  vector<math::Pose> initialPosition;
  /// A vector created for command management
  vector<ConditionPtr> commandConditions;
  vector<ModuleCommandsPtr> moduleCommandContainer;
// private: vector<transport::PublisherPtr> WorldPublisher;
  /// This is used for automatical magnetic connection
  vector<transport::SubscriberPtr> WorldColSubscriber;
  /// Auto Magnetic Connection Enable Flag
  bool autoMagneticConnectionFlag;
  /// The path of the file of configuration
  string configurationFile;
//+++++++++ testing ++++++++++++++++++++++++++++
};
} // namespace gazebo
#endif