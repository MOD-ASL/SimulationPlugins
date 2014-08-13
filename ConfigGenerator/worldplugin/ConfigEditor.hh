//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: This is the world plugin that used by configuration
//              generator. It basically inherit all the functions from
//              world plugin template. However there are several main
//              differences, 1. This world plugin will not load 
//              initial configuration, but will have a subscriber that
//              subscribe configuration information; 2. This world plugin
//              will publish configuration information to gui plugin that 
//              will force graphic updates happening
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_CONFIG_EDITOR_HH_
#define _GAZEBO_CONFIG_EDITOR_HH_

#include "WorldServer.hh"
#include "config_message.pb.h"

typedef const boost::shared_ptr
    <const config_message::msgs::ConfigMessage> ConfigMessagePtr;
    
namespace gazebo{
class ConfigEditor : public WorldServer
{
 public:
  ConfigEditor();
  ~ConfigEditor();
  /// A slightly different insertion function that will get around 
  /// simulation graphic not updating issue
  virtual void InsertModel(string name, math::Pose position, string joint_angles);
  virtual void InsertModel(string name, math::Pose position, string joint_angles, 
      string model_path);
  /// The function that perform extra initialization
  /// In the base class, configuration wil be built in this function
  /// Which means we need to diable it here
  virtual void ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf);
  /// Need to be set to empty so the world plugin will not read in gaits
  void ExtraWorkWhenModelInserted(CommandMessagePtr &msg);
 private:
  /// Call back when receiving configuration message
  void ConfigMessageDecoding(ConfigMessagePtr &msg);
 private:
  transport::SubscriberPtr configSub;
  transport::PublisherPtr configPub;
};
} // namespace gazebo
#endif