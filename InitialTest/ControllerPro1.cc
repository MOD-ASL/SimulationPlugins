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
					WheelRadius =  0.045275;
					ComplementFilterPar = 0.9; // The coeffecient of the previous value
					//DefaultKPID(1.0,0.0,0.0);
					printf("Model Initiated\n");
					//cout<<"Please enter the speed you want:"<<endl;
					//cin>>this->ExpectedSpeedR;
					//this->ExpectedSpeedL = this->ExpectedSpeedR;
				}
		public: void Load (physics::ModelPtr _parent, sdf::ElementPtr _sdf)
				{
					this->model = _parent;
					this->Joint1 = model->GetJoint("Right_wheel_hinge");
					this->Joint2 = model->GetJoint("Left_wheel_hinge");
					this->Joint1->SetMaxForce(0,1000);
					this->Joint2->SetMaxForce(0,1000);
					this->Joint1->SetVelocity(0,PI);
					this->Joint2->SetVelocity(0,PI);
					this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelController1::GetAndPostJointAngleValue, this, _1));
				}
		// Testing function
		private: void GetAndPostJointAngleValue(const common::UpdateInfo & /*_info*/)
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
					force = Joint1->GetMaxForce(0);
					cout<<"The maximum force:"<<force<<endl;
					AngleValue = this->RevolutionSpeedCal(Joint1,0);
					cout<<"The real rotation rate is "<<AngleValue<<" rad/s"<<endl;

				}
		//private: void PIDSpeedController(const common::UpdateInfo & /*_info*/)
				/*{
					double RadiusWheelL, RadiusWheelR;
					double SpeedWheelL, SpeedWheelR;
					double TimeStampNow, TimeStampHistory=0;
					static double RadiusWheelLHis = 0;
					static double RadiusWheelRHis = 0;
					math::Angle angleL, angleR;
					common::Time TimeTmp;
					physics::ModelState currentModelState(this->model);
					double Kp;
					double Ki;
					double Kd;
					double SpeedErrorR, SpeedErrorRHis1, SpeedErrorRHis2;
					double SpeedErrorL, SpeedErrorLHis1, SpeedErrorLHis2;

					TimeTmp = currentModelState.GetSimTime();
					TimeStampNow = TimeTmp.Double();
					angleR = Joint1->GetAngle(1);
					angleL = Joint2->GetAngle(1);
					RadiusWheelR = angleR.Radian();
					RadiusWheelL = angleL.Radian();
					SpeedWheelR = (RadiusWheelR-RadiusWheelRHis)*WheelRadius/(TimeStampNow-TimeStampHistory);
					SpeedWheelL = (RadiusWheelL-RadiusWheelLHis)*WheelRadius/(TimeStampNow-TimeStampHistory);

					SpeedErrorR = ExpectedSpeedR - SpeedWheelR;
					SpeedErrorL = ExpectedSpeedL - SpeedWheelL;		

					TimeStampHistory = TimeStampNow;
					RadiusWheelRHis = RadiusWheelR;
					RadiusWheelLHis = RadiusWheelL;

				}*/
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

						SpeedCurrent = this->aFilter(this->RevolutionSpeedCal(JointNTBC,AxisIndex));
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
		public: double aFilter(double FilteringValue)
				{
					static double ValueRecorder = 0;
					double FiltedValue;
					FiltedValue = (1 - ComplementFilterPar)*FilteringValue + ComplementFilterPar*ValueRecorder;

					ValueRecorder = FiltedValue;

					return FiltedValue;
				}
		//private: physics::ModelState currentModelState;
		private: physics::ModelPtr model;
		private: physics::JointPtr Joint1;
		private: physics::JointPtr Joint2;
		private: event::ConnectionPtr updateConnection;
		public:  math::Vector3 DefaultKPID;	// First digit is Kp, second digit is Ki and third digit is Kd
		// In the future version, this radius value will be eliminate
		private: double WheelRadius;

		public:  double ComplementFilterPar;
	};

	GZ_REGISTER_MODEL_PLUGIN(ModelController1)
}