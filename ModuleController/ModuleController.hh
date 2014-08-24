//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: Thie is a customized model plugin class, in which we 
//              implemented the low level controllers of each individual
//              model. Those controllers including joint position
//              control, joint speed control, model plane motion control,
//              model plane position and orientation control
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//! \file Defines the modelplugin for low level control of modules
#ifndef _GAZEBO_MODEL_CONTROLLER_HH_
#define _GAZEBO_MODEL_CONTROLLER_HH_

#include <iostream>
#include <cmath>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// Classes for communication between different plugins
#include "collision_message.pb.h"
#include "command_message.pb.h"
// Library for colored log text
#include "ColorLog.hh"
// The head file which contains the joint_plus structure
#include "SmoresJoint.hh"

#define PI 3.1415926
#define EXECUTIONERROR 0.08

using std::vector;
using std::string;
/// shared_ptr of GzString object
typedef const boost::shared_ptr<const gazebo::msgs::GzString> GzStringPtr;
/// shared_ptr of Pose
typedef const boost::shared_ptr<const gazebo::msgs::Pose> PosePtr;
/// shared_ptr of CommandMessage
typedef const boost::shared_ptr<const command_message::msgs::CommandMessage> 
    CommandMessagePtr;

namespace gazebo
{
/// The modelplugin that actually controls the model and provide low level apis
/*! This class programmed follows the gazebo model plugin standard.
    All the low level controllers of SMORES robot are in here.
    A graphic structure can be find here: 
*/
class ModuleController : public ModelPlugin 
{
 public:
  /// Constructor
  ModuleController();
  /// Destructor
  ~ModuleController();

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+  Useful functions to get model status and other tool functions  +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Calculates the angular velocity of a revolute joint
  /*!
    \param joint Joint object pointer whose revolution rate needs to be
    calculate
    \param axis_index Index of the axis, for a single joint type,
    it is always 0
    \return Joint speed in rad/s
  */
  double RevolutionSpeedCal(physics::JointPtr joint, const int axis_index);
  /// Gets the coordinates and direction of the current model
  math::Pose GetModelCentralCoor(void);
  /// This function is used to calculate the angle in the world frame 
  /// of two points in the world frame
  /// It is useful when moving object on a specific surface
  math::Angle AngleCalculation2Points(math::Vector2d start_point, 
      math::Vector2d end_point);
  /// This function is used to apply a complementary filter
  /// Without any default factors
  double ComplementaryFilter(double filtering_value, 
      double complement_filter_par, double *value_history);
  /// This function is used to apply a complementary filter
  /// With a default factor of 0.9
  double ComplementaryFilter(double filtering_value, double *value_history);
  /// This function is used to return the SmoresJoint structure according to the node ID
  SmoresJoint & GetJointPlus(int node_ID);
  /// This function is used to return the index of joint axis according to the node ID
  int GetJointAxis(int node_ID);
  /// This function will return the angle of the specified joint
  math::Angle GetJointAngle(physics::JointPtr current_joint, int rot_axis);
  /// Callback that decodes collision message and populate it to world plugin
  /*! Used by magnetic connection
    \param msg GzStringPtr object of the collision message
  */
  virtual void CollisionReceivingCallback(GzStringPtr &msg);
  /// Extra stuff taht should be set on load
  virtual void ExtraOnload(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

 private:
  /// Callback function when model plugin has been loaded
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  /// Callback when receive welcome informaation from world plugin
  void WelcomInfoProcessor(GzStringPtr &msg);
  /// Plugin initialization function
  void SystemInitialization(physics::ModelPtr parent_model);
  /// Callback that runs in every 0.001s of simulation.
  void OnSystemRunning(const common::UpdateInfo & /*_info*/);
  /// Collision Topics Publisher and Subscriber initialization
  /// Used by magnetic connection
  void CollisionPubAndSubInitialization(void);
  /// Callback that decode commands from world plugin and execute them
  void CommandDecoding(CommandMessagePtr &msg);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+            Low level model control functions                    +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// This function force to set joint to a specific angle
  /// Only used in simulation
  void SetJointAngleForce(physics::JointPtr current_joint, int rot_axis, 
      double position);
  /// This function is the actuall joint control function used now
  /// This function apply a PID controller to control the joint speed 
  /// and to achieve the specified angle
  void JointPIDController(double angle_desired_radian, 
      double desire_speed, SmoresJoint *current_joint);
  void JointPIDController(double angle_desired_radian, SmoresJoint *current_joint);
  /// Joint Plus member update function
  void JointAngleUpdateInJointPlus(void);
  /// This function will set the rotation rate of the joint
  /// The unit of this speed is rad/s
  void SetJointSpeed(physics::JointPtr CurrentJoint, int RotAxis, 
      double SpeedDesired);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+            Command Execution Tracking Functions                 +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  void JointAngleTracking(void);
  void PositionTracking(void);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+          Functions that control the model's planar motion       +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// This function is used to control the planar motion orientation 
  /// of SMORES on the ground
  void AnglePIDController(math::Angle desired_angle, math::Angle current_angle, 
      math::Vector2d current_speed);
  /// This function is used to control the model drive to a specific point 
  /// on the ground plane
  void Move2Point(math::Vector2d DesiredPoint, math::Angle DesiredOrientation);
 public:
  /// Current Model Pointer
  physics::ModelPtr model;
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+                       Model Pramaters                           +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Default Joint Angle PID controller parameter
  math::Vector3 jointAngleKPID;   // First digit is Kp, 
                                  // second digit is Ki and 
                                  // third digit is Kd
  // Default Joint Angle PID controller parameter
  math::Vector3 modelAngleKPID;   // First digit is Kp,
                                  // second digit is Ki and
                                  // third digit is Kd
  /// Maximium rotation rate of each joint, unit: rad/s
  double maxiRotationRate;  // Maximium rotation rate of each joint
  double accelerationRate;
  double planarMotionStopThreshold;
  double wheelRadius;

  transport::PublisherPtr collisionInfoToServer;
 private: 
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+       Pointers of Model and joints and a emhanced struct        +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Pointers to all the Joints the current model has
  physics::JointPtr jointWR;
  physics::JointPtr jointWL;
  physics::JointPtr jointWF;
  physics::JointPtr jointCB;
  // An enhanced struct which is useful when apply PID controllers
  SmoresJoint jointWRP;
  SmoresJoint jointWLP;
  SmoresJoint jointWFP;
  SmoresJoint jointCBP;

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+                     Model plugin events                         +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // The event that will be refreshed in every iteration of the simulation
  event::ConnectionPtr updateConnection;

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+         Communication Publishers and Subscribers                +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  transport::SubscriberPtr welcomeInfoSub;
  transport::SubscriberPtr commandSub;
  transport::PublisherPtr  commandPub;
  transport::SubscriberPtr linkCollisonSub[4];

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+                       Useful Buffers                            +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Contains all the names of connected models
  vector<string> nameOfConnectedModels;

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+                 Model Execution Variables                       +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Execution state
  int executionState; // 0 for waiting mode, 
                     // 1 for planar motion mode, 
                     // 2 for joint control mode, 
                     // 3 for direct control mode 
  /// Desired joint angles
  double jointAngleShouldBe[4];
  /// Joint speed
  double jointSpeeds[4];
  /// Desired robot position when controled to move to a position 
  math::Pose targetPosition;
  /// Left wheel speed
  double lftWheelSpeed;
  /// Right wheel speed
  double rgtWheelSpeed;
  /// Execution status
  bool startExecution;
}; // class ModuleController
} // namespace gazebo
#endif
