//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: Thie is a customized model plugin class, in which we 
//              implemented the low level controllers of each individual
//              model. Those controllers including joint position
//              control, joint speed control, model plane motion control,
//              model plane position and orientation control
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*! \file Defines the modelplugin for low level control of modules*/
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
/*! This class is a gazebo modelplugin.
    All the low level controllers of SMORES robot are defined here.
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
  /*!
    \return The global coordinates of the current model, 
            which is a math::Pose object
  */
  math::Pose GetModelCentralCoor(void);
  /// Calculates the anlge of the line connected two points in the world frame
  /*!
    \param start_point Coordinates of the Start point
    \param end_point Coordinates of the End point
    \return Angle between 2 points in rad in the global frame
            math::Angle object
  */
  math::Angle AngleCalculation2Points(math::Vector2d start_point, 
      math::Vector2d end_point);
  /// Apply a complementary filter a raw sensor value
  /*!
    \param filtering_value The value need to be filtered
    \param complement_filter_par Complementry filter factor, defined in the 
            following equation: 
            complement_filter_par*value_history + (1 - complement_filter_par)
            *filtering_value
    \param *value_history The pointer points to the prrior filtering value.
    /return The filered value
  */
  double ComplementaryFilter(double filtering_value, 
      double complement_filter_par, double *value_history);
  /// Applies a complementary filter with a default factor of 0.9
  /*!
    \param filtering_value The value need to be filtered
    \param *value_history The pointer points to the prrior filtering value.
    /return The result the filered value
  */
  double ComplementaryFilter(double filtering_value, double *value_history);
  /// Returns the SmoresJoint structure according to the node ID
  /*!
    \param node_ID ID of node
    \return Corresponding SmoresJoint object
  */
  SmoresJoint & GetJointPlus(int node_ID);
  /// Returns the index of joint axis according to the node ID
  /*!
    \param node_ID ID of node
    \return Index of the corresponding axis
  */
  int GetJointAxis(int node_ID);
  /// Returns the angle of the specified joint
  /*!
    \param current_joint Current joint intersted
    \param rot_axis Axis rotated along
    \return Joint angle position in a math::Angle object
  */
  math::Angle GetJointAngle(physics::JointPtr current_joint, int rot_axis);

 private:
  /// Callback function when model plugin has been loaded
  /*! Including register GaitMessage receiving callback
    \param _parent Current WorldPtr object
    \param _sdf ElementPtr object of the current world
  */
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  /// Callback when receive welcome informaation from world plugin
  /*! TODO: currently there is no reaction when receiving welcome information
    \param msg GzStringPtr object of the incoming message  
  */
  void WelcomInfoProcessor(GzStringPtr &msg);
  /// Plugin initialization function
  /*!
    \param parent_model Pointer of the model object
  */
  void SystemInitialization(physics::ModelPtr parent_model);
  /// Main control loop function, running in 1000Hz
  void OnSystemRunning(const common::UpdateInfo & /*_info*/);
  /// Collision Topics Publisher and Subscriber initialization
  /*! Used by magnetic connection */
  void CollisionPubAndSubInitialization(void);
  /// Callback that decode collision information and populate it to worldplugin
  /*! Used by magnetic connection */
  void CollisionReceivingCallback(GzStringPtr &msg);
  /// Callback that decodes commands from worldplugin
  /*!
    \param msg CommandMessagePtr object of the incoming message  
  */
  void CommandDecoding(CommandMessagePtr &msg);


  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+            Low level model control functions                    +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// This function force to set joint to a specific angle
  /*!
    Only used in simulation
    \param current_joint The joint interested  
    \param rot_axis  Desired rotation axis
    \param position Desired position
  */
  void SetJointAngleForce(physics::JointPtr current_joint, int rot_axis, 
      double position);
  /// Applies a PID controller to control the joint position 
  /*!
    \param angle_desired_radian Desired angle to achieve in rad
    \param desire_speed Desired Speed in percentage of the maximum speed
    \param *current_joint SmoresJoint object of the current joint
  */
  void JointPIDController(double angle_desired_radian, 
      double desire_speed, SmoresJoint *current_joint);
  /// Applies a PID controller to control the joint position with a fixed speed
  /*!
    Default speed 80% of the maximum speed.
    \param angle_desired_radian Desired angle to achieve in rad
    \param *current_joint Current joint
  */
  void JointPIDController(double angle_desired_radian, SmoresJoint *current_joint);
  /// Update all SmoresJoint object in the main loop
  void JointAngleUpdateInJointPlus(void);
  /// Aet the rotation rate of the joint
   /*!
    The unit of this speed is rad/s
    \param CurrentJoint Current Joint 
    \param RotAxis The axis desired to rotate along
    \param SpeedDesired Desired Speed in rad/s
  */
  void SetJointSpeed(physics::JointPtr CurrentJoint, int RotAxis, 
      double SpeedDesired);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+            Command Execution Tracking Functions                 +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Track the status of all joint angles
  /*! Send feedback message to worldplugin when joint position acheived*/
  void JointAngleTracking(void);
  /// Track the position of the module
  /*! Send feedback message to worldplugin when model position acheived*/
  void PositionTracking(void);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+          Functions that control the model's planar motion       +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Control the orientation of the planar motion of SMORES on the ground
  /*!
    /param desired_angle Desired angle  
    /param current_angle Current angle
    /param current_speed Current speed
  */
  void AnglePIDController(math::Angle desired_angle, math::Angle current_angle, 
      math::Vector2d current_speed);
  /// Control the model drive to a specific point on the ground plane
  /*!
    \param DesiredPoint Desired point to drive to
    \param DesiredOrientation Desired Orientation to drive to 
  */
  void Move2Point(math::Vector2d DesiredPoint, math::Angle DesiredOrientation);
 public:
  /// Current Model Pointer
  physics::ModelPtr model;
  /// Publisher of the collision information
  transport::PublisherPtr collisionInfoToServer;
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+                       Model Pramaters                           +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Default Joint Angle PID controller parameter
  /*!
    First digit is Kp, 
    Second digit is Ki and 
    Third digit is Kd
  */
  math::Vector3 jointAngleKPID; 
  // Default Joint Angle PID controller parameter
  /*!
    First digit is Kp,
    Second digit is Ki and
    Third digit is Kd
  */
  math::Vector3 modelAngleKPID;   
  /// Maximium rotation rate of each joint, unit: rad/s
  double maxiRotationRate;  // Maximium rotation rate of each joint
  /// Acceleration rate of the planar motion
  double accelerationRate;
  /// Planar motion control threshold
  double planarMotionStopThreshold;
  /// Radius of the wheels
  double wheelRadius;
 private: 
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+       Pointers of Model and joints and a emhanced struct        +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Pointer to right wheel joint
  physics::JointPtr jointWR;
  /// Pointer to left wheel joint
  physics::JointPtr jointWL;
  /// Pointer to front wheel joint
  physics::JointPtr jointWF;
  /// Pointer to the central bending joint
  physics::JointPtr jointCB;
  /// Enhanced joint object of right wheel joint
  SmoresJoint jointWRP;
  /// Enhanced joint object of left wheel joint
  SmoresJoint jointWLP;
  /// Enhanced joint object of front wheel joint
  SmoresJoint jointWFP;
  /// Enhanced joint object of central bending joint
  SmoresJoint jointCBP;

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+                     Model plugin events                         +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// The event that will be refreshed in every iteration of the simulation
  event::ConnectionPtr updateConnection;

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+         Communication Publishers and Subscribers                +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Subscriber of the welcome information
  transport::SubscriberPtr welcomeInfoSub;
  /// Subscriber of the command
  transport::SubscriberPtr commandSub;
  /// Publisher of the command
  transport::PublisherPtr  commandPub;
  /// Subscribers of 4 links
  transport::SubscriberPtr linkCollisonSub[4];

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+                       Useful Buffers                            +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// The vector that is a list of names of all connected models
  vector<string> nameOfConnectedModels;

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+                 Model Execution Variables                       +
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  /// Execution state
  /*!
    0 for waiting mode, 
    1 for planar motion mode, 
    2 for joint control mode, 
    3 for direct control mode 
  */
  int executionState; 
  /// Desired joint angles
  double jointAngleShouldBe[4];
  /// Desired joint speeds
  double jointSpeeds[4];
  /// Desired robot position when controled to move to a position 
  math::Pose targetPosition;
  /// Left wheel speed
  double lftWheelSpeed;
  /// Right wheel speed
  double rgtWheelSpeed;
  /// Execution status
  bool startExecution;
  // TODO: command id should be implemented in the future
}; // class ModuleController
} // namespace gazebo
#endif
