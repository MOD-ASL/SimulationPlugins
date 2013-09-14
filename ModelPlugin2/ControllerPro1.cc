#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <iostream>
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
					// Get all the pointers point to right objects
					this->model = _parent;
					this->JointWR = model->GetJoint("Right_wheel_hinge");
					this->JointWL = model->GetJoint("Left_wheel_hinge");
					this->JointWF = model->GetJoint("Front_wheel_hinge");
					this->JointCB = model->GetJoint("Center_hinge");
					// Setting the model state
					// Setting the maximium torque of the two wheels
					this->JointWR->SetMaxForce(0,1000);
					this->JointWL->SetMaxForce(0,1000);
					// Setting the maximium torque of the front wheel
					this->JointWF->SetMaxForce(0,1000);
					// Setting the maximium torque of the body bending joint
					this->JointCB->SetMaxForce(0,1000);
					// Testing codes
					this->JointWR->SetVelocity(0,0);
					this->JointWL->SetVelocity(0,0);
					math::Angle AngleInterested(0.5235987);
					this->JointCB->SetAngle(0,AngleInterested);
					// Event register, which will make the function be executed in each iteration
					this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelController1::OnSystemRunning, this, _1));
				}
		// Testing function
		private: void OnSystemRunning(const common::UpdateInfo & /*_info*/)
				{
					double AngleValue;
					double force;
					// math::Angle angleOfJoint1;
					// angleOfJoint1 = Joint1->GetAngle(2);
					// AngleValue = angleOfJoint1.Degree();		
					//****************************************************			
					//common::Time TimeTmp;
					//physics::ModelState currentModelState(this->model);
					//TimeTmp = currentModelState.GetSimTime();
					//AngleValue = TimeTmp.Double();
					//***************************************************
					//AngleValue = this->RevolutionSpeedCal(Joint1,1);
					//cout<<"Angle:"<<AngleValue<<endl;
					//cout<<"Rotation Speed:"<<AngleValue<<" rad/s"<<endl;
					// math::Vector3 TmpPID(0,0,0);
					// PIDSpeedController(Joint1,0,PI/10,TmpPID);
					force = JointCB->GetForce(0);
					cout<<"The Angle of the bending:"<<force<<endl;
					// if (force > 0)
					// {
					// 	this->JointCB->SetVelocity(0,-0.1);
					// }else{
					// 	this->JointCB->SetVelocity(0,0);
					// }
					AngleValue = this->RevolutionSpeedCal(JointWR,0);
					cout<<"The real rotation rate is "<<AngleValue<<" rad/s"<<endl;

				}
		// This public function is used to control the speed of joint
		// The third parameter determine what kind of speed the user want to controll: 0 for rotation, 1 for linear, 2 for prismatic
		// If want to use the default PID controller parameters, just pass math::Vector3::Zero to KPID
		// If there is any errors or uncompleted information, the function will return false
		public: bool PIDSpeedController(const physics::JointPtr JointNTBC, const int AxisIndex, const double ExpectedSpeed,
										math::Vector3 KPID , const int SpeedType = 0, const double Radius = 0)
				{
					double SpeedCurrent;
					double TorqueIncrement;
					static double TotalTorque;

					// If PID parameters are not specified, set them to default
					if (KPID.x==0 && KPID.y==0 && KPID.z == 0)
					{
						KPID = DefaultKPID;
					}
					if (SpeedType == 0)
					{
						double SpeedErrorT0;
						static double SpeedErrorHis1T0 = 0, SpeedErrorHis2T0 = 0;
						static double SpeedErrorAccu = 0;

						SpeedCurrent = this->ComplementaryFilter(this->RevolutionSpeedCal(JointNTBC,AxisIndex));
						cout<<"SpeedCurrent = "<<SpeedCurrent<<endl;
						cout<<"ExpectedSpeed = "<<ExpectedSpeed<<endl;

						SpeedErrorT0 = ExpectedSpeed - SpeedCurrent;
						cout<<"SpeedErrorT0 = "<<SpeedErrorT0<<endl;
						//cout<<"The Current Error is "<<SpeedErrorT0<<" rad/s"<<endl;
						SpeedErrorAccu += SpeedErrorT0;

						TorqueIncrement = KPID.x*SpeedErrorT0 + KPID.y*SpeedErrorAccu + KPID.z*(SpeedErrorT0-SpeedErrorHis1T0);
						cout<<"The Torque Increment "<<TorqueIncrement<<endl;
						SpeedErrorHis2T0 = SpeedErrorHis1T0;
						SpeedErrorHis1T0 = SpeedErrorT0;
					}else{
						if (SpeedType == 1)
						{
							// If the function is configured to calculate linar speed for a wheel but no radiu has been specified
							// return false and end the function
							if (Radius == 0)
							{
								cout<<"Wrong radius input when calculating linear speed of a wheel."<<endl;
								return false;
							}else{

							}
						}else{
							if (SpeedType == 2)
							{
								/* code */
							}else{
								cout<<"Wrong Speed Type."<<endl;
								return false;
							}
						}
					}

					TotalTorque += TorqueIncrement;
					cout<<"TotalTorque = "<<TotalTorque<<endl;
					double TotalTorqueRef;
					TotalTorqueRef = JointNTBC->GetForce(AxisIndex) + TorqueIncrement;
					cout<<"TotalTorqueRef = "<<TotalTorqueRef<<endl;
					JointNTBC->SetForce(AxisIndex,(JointNTBC->GetForce(AxisIndex) + TorqueIncrement));
					return true;
				}
		// This public function is used to calculate the angular velocity of a revolute joint
		// This method is only valid in simulation
		public: double RevolutionSpeedCal(physics::JointPtr JointNTBC, const int AxisIndex)
				{
					double ResSpeedRef;

					// In the API instruction, fucntion GetVelocity() returns the "rotation rate"
					// The unit of this rotation rate is "rad/s"
					ResSpeedRef = JointNTBC->GetVelocity(AxisIndex);
					//cout<<"ResSpeedRef = "<<ResSpeedRef<<" rad/s"<<endl;

					return ResSpeedRef;
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