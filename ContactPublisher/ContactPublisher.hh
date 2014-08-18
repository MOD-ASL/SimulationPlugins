//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
// Author: Edward Yunkai Cui
// Description: This sensor pluginin is used to populate message when
//              two models are close enough to connect, this messgae 
//              will eventually pass to the world plugin, the system 
//              monitor.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
#ifndef _GAZEBO_CONTACT_SENSOR_HH_
#define _GAZEBO_CONTACT_SENSOR_HH_

#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
/// This class will be used in automatically connection under magnetic force
/*! The trigger condition of magnetic connections is the face contact of two modules  */
class ContactSensor : public SensorPlugin
{         
 public: 
  /// Constructor.
  ContactSensor();
  /// Destructor.
  ~ContactSensor();
  /// Load the sensor plugin.
  /*!
    \param _sensor: Pointer to the sensor that loaded this plugin.
    \param _sdf: SDF element that describes the plugin.
  */
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  /// Callback that recieves the contact sensor's update signal.
  void OnUpdate();

 private: 
  /// Pointer to the contact sensor
  sensors::ContactSensorPtr parentSensor;
  /// Connection event handle
  /* Connection that maintains a link between the contact sensor's
      updated signal and the OnUpdate callback. */
  event::ConnectionPtr updateConnection;
  /// Collision publisher
  transport::PublisherPtr collisionPub; 
}; // class ContactSensor
} // namespace gazebo
#endif