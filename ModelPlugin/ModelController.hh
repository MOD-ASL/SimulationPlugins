#ifndef _GAZEBO_MODEL_CONTROLLER_HH_
#define _GAZEBO_MODEL_CONTROLLER_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <cmath>
// Google protocol buffers
// Classes for information exchanges in different plugins
#include "collision_message_plus.pb.h"
#include "command_message.pb.h"
// The head file which contains the joint_plus structure
#include "JointPlus.hh"

// Parameters
#define PI 3.1415926

using namespace std;

// Useful pointer type declaration
typedef const boost::shared_ptr<const msgs::GzString> GzStringPtr;
typedef const boost::shared_ptr<const msgs::Pose> PosePtr;
typedef const boost::shared_ptr<const command_message::msgs::CommandMessage> CommandMessagePtr; 

namespace gazebo
{
  // This is the controlller for SMORES
  // Please find the graphic structure on the lab wiki
  class ModelController : public ModelPlugin
  {
    /// Constructor.
    public: ModelController();

    /// Destructor.
    public: virtual ~ModelController();

    /// Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// Callback when receive welcome informaation from world plugin
    public: virtual void welcomInfoProcessor(GzStringPtr &msg);

    /// Plugin initialization function
    private: void SystemInitialization(physics::ModelPtr parentModel);

    /// Callback that runs in every iteration.
    private: virtual void OnSystemRunning(const common::UpdateInfo & /*_info*/);

    /// Collision Topics Publisher and Subscriber initialization
    /// Used by magnetic connection
    private: void CollisionPubAndSubInitialization(void);

    /// Callback that decode collision information and populate it to world plugin
    /// Used by magnetic connection
    private: void CollisionReceiverProcessor(GzStringPtr &msg);

    /// Callback that decode commands from world plugin and execute them
    private: void CommandDecoding(CommandMessagePtr &msg);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+            Low level model control functions                    +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    /// This function force to set joint to a specific angle
    /// Only used in simulation
    public: virtual void SetJointAngleForce(physics::JointPtr CurrentJoint, int RotAxis, math::Angle AngleDesired);

    /// This function will be implemented future and will call JointPIDController()
    public: virtual void JointAngleControl(void);

    /// This function is the actuall joint control function used now
    /// This function apply a PID controller to control the joint speed to achieve the specified angle
    private: virtual void JointPIDController(JointPlus &CurrentJoint, int RotAxis, math::Angle AngleDesired, double DesireSpeed = 1);

    /// This function will return the angle of the specified joint
    public: virtual math::Angle GetJointAngle(physics::JointPtr CurrentJoint, int RotAxis);

    /// This function will set the rotation rate of the joint
    /// The unit of this speed is rad/s
    public: virtual void SetJointSpeed(physics::JointPtr CurrentJoint, int RotAxis, double SpeedDesired);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+  Useful functions to get model status and other tool functions  +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    /// This function is used to calculate the angular velocity of a revolute joint
    public: virtual double RevolutionSpeedCal(physics::JointPtr JointNTBC, const int AxisIndex);

    /// This function is used to get the coordinates and direction of the current model
    public: virtual math::Pose GetModelCentralCoor(void);

    /// This function is used to calculate the angle in the world frame of two points in the world frame
    /// It is useful when moving object on a specific surface
    public: virtual math::Angle AngleCalculation2Points(math::Vector2d StartPoint, math::Vector2d EndPoint);

    /// This function is used to apply a complementary filter
    public: double ComplementaryFilter(double FilteringValue, double ComplementFilterPar = 0.9);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+          Functions that control the model planar motion         +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    /// This function is used to control the planar motion direction of SMORES on the ground
    private: void AnglePIDController(math::Angle DesiredAngle, math::Angle CurrentAngle, math::Vector2d CurrentSpeed);

    /// This function is used to control the model drive to a specofc point on the ground plane
    private: void Move2Point(math::Vector2d DesiredPoint, math::Angle DesiredOrientation);

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+       Pointers of Model and joints and a emhanced struct        +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Current Model Pointer
    private: physics::ModelPtr model;
    // Pointers to all the Joints the current model has
    private: physics::JointPtr JointWR;
    private: physics::JointPtr JointWL;
    private: physics::JointPtr JointWF;
    private: physics::JointPtr JointCB;
    // An enhanced struct which is useful when apply PID controllers
    private: JointPlus JointWRP;
    private: JointPlus JointWLP;
    private: JointPlus JointWFP;
    private: JointPlus JointCBP;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+                     Model plugin events                         +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // The event that will be refreshed in every iteration of the simulation
    private: event::ConnectionPtr updateConnection;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+         Communication Publishers and Subscribers                +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    private: transport::SubscriberPtr sub;
    private: transport::SubscriberPtr CommandSub;
    private: transport::SubscriberPtr LinkCollisonSub[4];
    private: transport::PublisherPtr CollisionInfoToServer;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+                       Useful Buffers                            +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Vector that contains all the names of connected models
    private: vector<string> NameOfConnectedModels;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+                       Model Pramaters                           +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Default Joint Angle PID controller parameter
    public:  math::Vector3 JointAngleKPID;  // First digit is Kp, second digit is Ki and third digit is Kd
    // Default Joint Angle PID controller parameter
    public:  math::Vector3 ModelAngleKPID;  // First digit is Kp, second digit is Ki and third digit is Kd
    // Maximium rotation rate of each joint
    public:  double MaxiRotationRate;
    public:  double AccelerationRate;
    public:  double PlanarMotionStopThreshold;

    private: double WheelRadius;

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //+                  Varibles only for testing                      +
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
  };
}
#endif
