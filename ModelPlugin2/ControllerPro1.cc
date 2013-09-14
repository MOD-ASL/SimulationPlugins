#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
#include <cmath>
using namespace std;

#define PI 3.1415926 

namespace gazebo
{
	class ModelController1 : public ModelPlugin
	{
		public: ModelController1() : ModelPlugin(),DefaultKPID(0.01,0,0)
				{
					// Initialize variables
					WheelRadius =  0.045275;
					// A hint of model been initialized
					printf("Model Initiated\n");
				}
		public: void Load (physics::ModelPtr _parent, sdf::ElementPtr _sdf)
				{
					// Initialize the whole system
					SystemInitialization(_parent);
					// Testing codes
					// this->JointWR->SetVelocity(0,0);
					// this->JointWL->SetVelocity(0,0);
					// math::Angle AngleInterested(0.5235987);
					// this->JointCB->SetAngle(0,AngleInterested);
					// Event register, which will make the function be executed in each iteration
					this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelController1::OnSystemRunning, this, _1));
				}
		private: void SystemInitialization(physics::ModelPtr parentModel)
				{
					// Get all the pointers point to right objects
					this->model = parentModel;
					this->JointWR = model->GetJoint("Right_wheel_hinge");
					this->JointWL = model->GetJoint("Left_wheel_hinge");
					this->JointWF = model->GetJoint("Front_wheel_hinge");
					this->JointCB = model->GetJoint("Center_hinge");
					// Setting the model states
					// Setting the maximium torque of the two wheels
					// this->JointWR->SetMaxForce(0,1000);
					// this->JointWL->SetMaxForce(0,1000);
					// Setting the maximium torque of the front wheel
					// this->JointWF->SetMaxForce(0,1000);
					// Setting the maximium torque of the body bending joint
					// this->JointCB->SetMaxForce(0,1000);
					// Set the angle of the hinge in the center to zero
					math::Angle InitialAngle(0);
					this->JointCB->SetAngle(0, InitialAngle);
				}
		// Testing function
		private: void OnSystemRunning(const common::UpdateInfo & /*_info*/)
				{
					double AngleValue;
					double force;
					// force = JointCB->GetForce(0);
					force = this->JointCB->GetMaxForce(0);
					cout<<"The Maximium Force of the Joint:"<<force<<endl;
					// AngleValue = this->RevolutionSpeedCal(JointWR,0);
					// cout<<"The real rotation rate is "<<AngleValue<<" rad/s"<<endl;

				}
		// The unit of the angle is radian
		// This function will only be used in simulation
		private: void SetJointAngle(physics::JointPtr CurrentJoint, int RotAxis, math::Angle AngleDesired)
				{
					CurrentJoint->SetAngle(RotAxis, AngleDesired);
				}
		// This function is used to replace "SetJointAngle" when actually control the joint
		// This function will drive the joint to the desire angle according to the speed specified
		// The speed is the percentage of the maximum speed in the simulation
		// The speed in the real worl depends on the external torque that actually applies on the joint
		private: void JointAngleControl(physics::JointPtr CurrentJoint, int RotAxis, math::Angle AngleDesired, double DesireSpeed = 1)
				{

				}
		// This function will return the angle of the specified joint
		// This function will be set to virtual in the future
		public: math::Angle GetJointAngle(physics::JointPtr CurrentJoint, int RotAxis)
				{
					math::Angle CurrentJointAngle;
					CurrentJointAngle = CurrentJoint->GetAngle(RotAxis);
					return CurrentJointAngle;
				}
		// This function will set the rotation rate of the joint
		// The unit of this speed is rad/s
		// In the future, this function should be defined as virtual
		public: void SetJointSpeed(physics::JointPtr CurrentJoint, int RotAxis, double SpeedDesired)
				{
					CurrentJoint->SetVelocity(RotAxis,SpeedDesired);
				}
		// This public function is used to calculate the angular velocity of a revolute joint
		// In the future, this function should be defined as virtual
		public: double RevolutionSpeedCal(physics::JointPtr JointNTBC, const int AxisIndex)
				{
					double ResSpeedRef;

					// In the API instruction, fucntion GetVelocity() returns the "rotation rate"
					// The unit of this rotation rate is "rad/s"
					ResSpeedRef = JointNTBC->GetVelocity(AxisIndex);
					//cout<<"ResSpeedRef = "<<ResSpeedRef<<" rad/s"<<endl;

					return ResSpeedRef;
				}
		// Get the coordinates and direction of the current model
		// Need to be virtual in the furture
		public: math::Pose GetModelCentralCoor(void)
				{
					math::Pose ModelPosition;
					physics::ModelState CurrentModelState(model);
					ModelPosition = CurrentModelState.GetPose();
					return ModelPosition;
				}
		// This function is an old version function of RevolutionSpeedCal()
		// Which has the same functionality as RevolutionSpeedCal()
		private: double RevolutionSpeedCalOld(physics::JointPtr JointNTBC, const int AxisIndex)
				{
					math::Angle CurrentAngle;
					double CurrentAngleRadVal;
					static double CurrentAngleRadValHis = 0;
					common::Time TimeTmp;
					physics::JointState CurrentJointState(JointNTBC);
					double TimeStampNow;
					static double TimeStampNowHis = 0;
					double RevSpeed;

					CurrentAngle = JointNTBC->GetAngle(AxisIndex);
					CurrentAngleRadVal = CurrentAngle.Radian();

					TimeTmp = CurrentJointState.GetSimTime();
					TimeStampNow = TimeTmp.Double();

					// This method uses the change of the angle divided by the time interval
					RevSpeed = (CurrentAngleRadVal-CurrentAngleRadValHis)/(TimeStampNow-TimeStampNowHis); // The unit is rad/sec

					CurrentAngleRadValHis = CurrentAngleRadVal;
					TimeStampNowHis = TimeStampNow;

					return RevSpeed;
				}
		// Angle calculation base on the two points
		// This angle is in the static global frame
		// This function can be used when the robot is on the ground
		// And aim to move to another point on the ground
		private: math::Angle AngleCalculation2Points(math::Vector2d StartPoint, math::Vector2d EndPoint)
				{
					double displacementX, displacementY, angleC;
					displacementX = EndPoint.x - StartPoint.x;
					displacementY = EndPoint.y - EndPoint.y;
					angleC = atan2(displacementY,displacementX);
					math::Angle ReturnAngle(angleC);
					return ReturnAngle;
				}
		// Angle PID Controller
		// DesiredAngleis in the static global frame
		// CurrentSpeed is the rotation rate of the wheel
		private: void AnglePIDController(math::Angle DesiredAngle, math::Angle CurrentAngle, math::Vector2d CurrentSpeed)
				{

				}
		// A complementary filter
		// The default coefficient of the filter is 0.9
		public: double ComplementaryFilter(double FilteringValue, double ComplementFilterPar = 0.9)
				{
					static double ValueRecorder = 0;
					double FiltedValue;
					FiltedValue = (1 - ComplementFilterPar)*FilteringValue + ComplementFilterPar*ValueRecorder;

					ValueRecorder = FiltedValue;

					return FiltedValue;
				}

		// Current Model Pointer
		private: physics::ModelPtr model;
		// Pointers to all the Joints the current model has
		private: physics::JointPtr JointWR;
		private: physics::JointPtr JointWL;
		private: physics::JointPtr JointWF;
		private: physics::JointPtr JointCB;
		// The event that will be refreshed in every iteration of the simulation
		private: event::ConnectionPtr updateConnection;
		// Default PID controller
		public:  math::Vector3 DefaultKPID;	// First digit is Kp, second digit is Ki and third digit is Kd
		//################# Variables for testing ############################
		// In the future version, this radius value will be eliminate
		private: double WheelRadius;

		//####################################################################
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelController1)
}