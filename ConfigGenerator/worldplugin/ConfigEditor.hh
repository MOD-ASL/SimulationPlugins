//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: 
/*! 
  This is the world plugin that used by configuration
  generator. It basically inherit all the functions from
  world plugin template. However there are several main
  differences, 1. This world plugin will not load 
  initial configuration, but will have a subscriber that
  subscribe configuration information; 2. This world plugin
  will publish configuration information to gui plugin that 
  will force graphic updates happening
*///+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
  /// Constructor
  ConfigEditor();
  /// Deconstructor
  ~ConfigEditor();
  /// Insert a default muscle module 
  /*!
    \param name Module name
    \param position position of the inserted module
    \param joint_angles a string which has the value for four joint angles
  */
  virtual void InsertModel(string name, math::Pose position, string joint_angles);
  /// Insert a different type of module by specifying the sdf file path
  /*!
    \param name Module name
    \param position position of the inserted module
    \param joint_angles a string which has the value for four joint angles
    \param model_path the path of the model sdf file
  */
  virtual void InsertModel(string name, math::Pose position, string joint_angles, 
      string model_path);
  /// The function that perform extra initialization
  /*!
    \param _parent gazebo world object
    \param _sdf gazebo sdf object of the world
  */
  virtual void ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf);
  /// Need to be set to empty so the world plugin will not read in gaits
  /*!
    Because this function will be called in the command callback function when
    receiving messgae from model.
    \param msg reference of the command message object pointer
  */
  void ExtraWorkWhenModelInserted(CommandMessagePtr &msg);
 private:
  /// Callback when receiving configuration message from python gui
  /*!
    \param msg Config Message object pointer reference
  */
  void ConfigMessageDecoding(ConfigMessagePtr &msg);
 private:
  /// Subscriber of the topic published by configuration editor
  transport::SubscriberPtr configSub;
  /// Publisher which is used to publish message to gui plugin
  transport::PublisherPtr configPub;
};
} // namespace gazebo
#endif