//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Filename:  ModelController.hh
// Author:  Edward Cui
// Contact: cyk1990995@gmail.com
// Last updated:  1/6/2014
// Description: 
// Commit info: git@github.com:princeedward/SimulationPlugins.git
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "ModelController.hh"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(ModelController)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Model Controller Constructor
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ModelController::ModelController() : ModelPlugin(),JointAngleKPID(1.5,0,0),ModelAngleKPID(1,0,0)
{
	// Initialize variables
	WheelRadius =  0.045275;
	MaxiRotationRate = 2.4086;
	AccelerationRate = 8;
	PlanarMotionStopThreshold = 0.02;

	// A hint of model been initialized
	printf("Model Initiated\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be executed When Model Has Been Loaded
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
	// Initialize the whole system
	SystemInitialization(_parent);
	// Initialize the subscribers
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	this->sub = node->Subscribe("~/Welcome",&ModelController::welcomInfoProcessor, this);
		// commandSubscriber = node->Subscribe("~/collision_map/command", &CollisionMapCreator::create, this);
	
	// Bind function which will be executed in each iteration
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ModelController::OnSystemRunning, this, _1));
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be executed When model received welcome message from world plugin
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::welcomInfoProcessor(GzStringPtr &msg)
{
	string InfoReceived = msg->data();
	// Do something useful here
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be called in function Load() to initialize the model
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::SystemInitialization(physics::ModelPtr parentModel)
{
	// Get all the pointers point to right objects
	this->model = parentModel;
	this->JointWR = model->GetJoint("Right_wheel_hinge");
	this->JointWL = model->GetJoint("Left_wheel_hinge");
	this->JointWF = model->GetJoint("Front_wheel_hinge");
	this->JointCB = model->GetJoint("Center_hinge");
	JointWRP.JointX = JointWR;
	JointWRP.Need2BeSet = false;
	JointWRP.JointErrorHis = 0;
	JointWRP.JointErrorAccu = 0;
	JointWLP.JointX = JointWL;
	JointWLP.Need2BeSet = false;
	JointWLP.JointErrorHis = 0;
	JointWLP.JointErrorAccu = 0;
	JointWFP.JointX = JointWF;
	JointWFP.Need2BeSet = false;
	JointWFP.JointErrorHis = 0;
	JointWFP.JointErrorAccu = 0;
	JointCBP.JointX = JointCB;
	JointCBP.Need2BeSet = false;
	JointCBP.JointErrorHis = 0;
	JointCBP.JointErrorAccu = 0;
	// Setting the model states
	// Setting the maximium torque of the two wheels
	this->JointWR->SetMaxForce(0,JointWR->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->GetValueDouble());
	this->JointWL->SetMaxForce(0,JointWL->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->GetValueDouble());
	// Setting the maximium torque of the front wheel
	this->JointWF->SetMaxForce(0,JointWF->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->GetValueDouble());
	// Setting the maximium torque of the body bending joint
	this->JointCB->SetMaxForce(0,JointCB->GetSDF()->GetElement("physics")->GetElement("ode")->GetElement("max_force")->GetValueDouble());
	// Set the angle of the hinge in the center to zero
	math::Angle InitialAngle(0.05);
	this->JointCB->SetAngle(0, InitialAngle);
	// math::Angle AngleNeed2Be(0.49778);
	// this->JointCB->SetAngle(0, AngleNeed2Be);

	
	// physics::ModelState CurrentModelState(model);
	string TopicName = "~/" + model->GetName() + "_world";
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init(model->GetName());
	this->CommandSub = node->Subscribe(TopicName,&ModelController::CommandDecoding, this);
	CollisionPubAndSubInitialization();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be called in each iteration of simulation
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::OnSystemRunning(const common::UpdateInfo & /*_info*/)
{
	// Do something useful after simulation begins

}

//-------------------------------------------------------------------
// Functions used below are for magnetic connection simulation
//-------------------------------------------------------------------

//------------------- Start Region ----------------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is the initalization function of collision message
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::CollisionPubAndSubInitialization(void)
{
	gazebo::transport::NodePtr node1(new gazebo::transport::Node());
	// Initialize the node with the model name
	node1->Init(model->GetName());
	// cout<<"Mode: node name is '"<<model->GetName()<<"'"<<endl;
	string TopicName = "~/" + model->GetName() + "::FrontWheel::front_contact";
	// cout<<"Mode: node topic is '"<<TopicName<<"'"<<endl;
	this->LinkCollisonSub[0] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
	TopicName = "~/" + model->GetName() + "::UHolderBody::UHolder_contact";
	// cout<<"Mode: node topic is '"<<TopicName<<"'"<<endl;
	this->LinkCollisonSub[1] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
	TopicName = "~/" + model->GetName() + "::LeftWheel::LeftWheel_contact";
	this->LinkCollisonSub[2] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);
	TopicName = "~/" + model->GetName() + "::RightWheel::RightWheel_contact";
	this->LinkCollisonSub[3] = node1->Subscribe(TopicName,&ModelController::CollisionReceiverProcessor,this);

	string ColPubName = "~/"+model->GetName()+"_Collision";
	CollisionInfoToServer = node1->Advertise<collision_message_plus::msgs::CollisionMessage>(ColPubName);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be called when recieved collision message
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::CollisionReceiverProcessor(GzStringPtr &msg)
{
	// cout<<"Message: "<<msg->data()<<endl;
	string MsgsInfo = msg->data();
	string Collison1 = MsgsInfo.substr(0,MsgsInfo.find(","));
	string Collison2 = MsgsInfo.substr(MsgsInfo.find(",")+1,-1);
	// cout<<"Model: Collision 1: "<<Collison1<<endl;
	// cout<<"Model: Collision 2: "<<Collison2<<endl;
	collision_message_plus::msgs::CollisionMessage CollisionMsgsPush;

	string LinkName;
	// cout<<"Model: Information came from model: "<<model->GetName()<<endl;
	if(Collison1.substr(0,Collison1.find("::")).compare(model->GetName())==0)
	{
		// cout<<"Model: Collision 1 position "
		CollisionMsgsPush.set_collision1(Collison1);
		CollisionMsgsPush.set_collision2(Collison2);
		LinkName = Collison1.substr((model->GetName()).length()+2,Collison1.find(":",(model->GetName()).length()+2)-2-(model->GetName()).length());
	}else{
		CollisionMsgsPush.set_collision1(Collison2);
		CollisionMsgsPush.set_collision2(Collison1);
		LinkName = Collison2.substr((model->GetName()).length()+2,Collison2.find(":",(model->GetName()).length()+2)-2-(model->GetName()).length());
	}
	// cout<<"Model: The collision link name is :"<<LinkName<<endl;
	msgs::Pose PositionOfCollisionLink = msgs::Convert(model->GetLink(LinkName)->GetWorldPose());
	CollisionMsgsPush.mutable_positioncol1()->CopyFrom(PositionOfCollisionLink);

	CollisionInfoToServer->Publish(CollisionMsgsPush);
}

//------------------- End Region ----------------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will be called when recieved an command from world plugin
//            The format of the command might be a gait table
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::CommandDecoding(CommandMessagePtr &msg)
{
	int commandType = msg->messagetype();
	switch(commandType)
	{
		case 1:{NameOfConnectedModels.push_back(msg->whichmodelconnectedto());}break;
	}
}

//------------- Low level model control functions --------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to set the torque of a specified joint
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::SetJointAngleForce(physics::JointPtr CurrentJoint, int RotAxis, math::Angle AngleDesired)
{
	CurrentJoint->SetAngle(RotAxis, AngleDesired);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to control the joint angle by using a PID controller
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::JointPIDController(JointPlus &CurrentJoint, int RotAxis, math::Angle AngleDesired, double DesireSpeed)
{
	double AngleError, AngleDiffError;
	double SwingSpeed;
	AngleError = (AngleDesired - CurrentJoint.JointAngleNow).Radian();
	// cout<<"AngleError:"<<AngleError<<endl;
	AngleDiffError = AngleError - CurrentJoint.JointErrorHis;
	CurrentJoint.JointErrorAccu += AngleError;
	SwingSpeed = JointAngleKPID.x*AngleError + JointAngleKPID.y*CurrentJoint.JointErrorAccu + JointAngleKPID.z*AngleDiffError;
	// cout<<"SwingSpeed:"<<SwingSpeed<<endl;
	if (abs(SwingSpeed)> MaxiRotationRate*DesireSpeed)
	{
		SwingSpeed = SwingSpeed>0?MaxiRotationRate*DesireSpeed:(-MaxiRotationRate*DesireSpeed);
	}
	SetJointSpeed(CurrentJoint.JointX,RotAxis,SwingSpeed);

	CurrentJoint.JointErrorHis = AngleError;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to get a specified joint angle
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
math::Angle ModelController::GetJointAngle(physics::JointPtr CurrentJoint, int RotAxis)
{
	math::Angle CurrentJointAngle;
	CurrentJointAngle = CurrentJoint->GetAngle(RotAxis);
	return CurrentJointAngle;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to control the joint speed
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function will set the rotation rate of the joint
// The unit of this speed is rad/s
void ModelController::SetJointSpeed(physics::JointPtr CurrentJoint, int RotAxis, double SpeedDesired)
{
	CurrentJoint->SetVelocity(RotAxis,SpeedDesired);
}

//------------- Low level model control functions END ----------------

//----------------------- Utility functions --------------------------
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to calculate the angular velocity of a revolute joint
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double ModelController::RevolutionSpeedCal(physics::JointPtr JointNTBC, const int AxisIndex)
{
	double ResSpeedRef;

	// In the API instruction, fucntion GetVelocity() returns the "rotation rate"
	// The unit of this rotation rate is "rad/s"
	ResSpeedRef = JointNTBC->GetVelocity(AxisIndex);
	//cout<<"ResSpeedRef = "<<ResSpeedRef<<" rad/s"<<endl;

	return ResSpeedRef;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to get the coordinates and direction of the current model
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
math::Pose ModelController::GetModelCentralCoor(void)
{
	math::Pose ModelPosition;
	physics::ModelState CurrentModelState(model);
	ModelPosition = CurrentModelState.GetPose();
	return ModelPosition;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to get the coordinates and direction of the current model
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
math::Angle ModelController::AngleCalculation2Points(math::Vector2d StartPoint, math::Vector2d EndPoint)
{
	double displacementX, displacementY, angleC;
	displacementX = EndPoint.x - StartPoint.x;
	displacementY = EndPoint.y - StartPoint.y;
	// cout<<"direction vector is [ " << displacementX << "," <<displacementY<<"]"<<endl;
	angleC = atan2(displacementY,displacementX);
	math::Angle ReturnAngle(angleC+PI/2);
	return ReturnAngle;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to apply a complementary filter
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double ModelController::ComplementaryFilter(double FilteringValue, double ComplementFilterPar)
{
	static double ValueRecorder = 0;
	double FiltedValue;
	FiltedValue = (1 - ComplementFilterPar)*FilteringValue + ComplementFilterPar*ValueRecorder;

	ValueRecorder = FiltedValue;

	return FiltedValue;
}

//----------------------- Utility functions END ----------------------

//----------- Planar Motion Control functions ------------------------

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to control the planar motion direction of SMORES on the ground
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::AnglePIDController(math::Angle DesiredAngle, math::Angle CurrentAngle, math::Vector2d CurrentSpeed)
{
	double AngleError, AngleErrorDiff, DiffSpeedControl;
	static double AngleErrorHis = 0;
	static double AngleErrorAcu = 0;
	double LeftWheelSpeed, RightWheelSpeed;

	while (abs(DesiredAngle.Degree()) > 180)
	{
		double DegreeAngle;
		DegreeAngle = DesiredAngle.Degree()>0?DesiredAngle.Degree()-360:DesiredAngle.Degree()+360;
		DesiredAngle.SetFromDegree(DegreeAngle);
	}
	// cout<<"Desired Angle is "<< DesiredAngle.Degree()<<endl;

	if (abs((DesiredAngle-CurrentAngle).Degree())>180)
	{
		AngleError = (DesiredAngle-CurrentAngle).Degree()>0?(DesiredAngle-CurrentAngle).Radian()-2*PI:(DesiredAngle-CurrentAngle).Radian()+2*PI;
	}else{
		AngleError = (DesiredAngle-CurrentAngle).Radian();
	}
	AngleErrorAcu += AngleError;
	AngleErrorDiff = AngleError - AngleErrorHis;
	DiffSpeedControl = ModelAngleKPID.x*AngleError + ModelAngleKPID.y*AngleErrorAcu + ModelAngleKPID.z*AngleErrorDiff;
	LeftWheelSpeed = CurrentSpeed.x - DiffSpeedControl;
	RightWheelSpeed = CurrentSpeed.y + DiffSpeedControl;

	// Maximium speed checking
	// if (abs(RightWheelSpeed)>JointWRP.MaximiumRotRate)
	// {
	// 	RightWheelSpeed = RightWheelSpeed>0?JointWRP.MaximiumRotRate:-JointWRP.MaximiumRotRate;
	// 	LeftWheelSpeed = RightWheelSpeed - 2*DiffSpeedControl;
	// 	if (abs(LeftWheelSpeed)>JointWLP.MaximiumRotRate)
	// 	{
	// 		LeftWheelSpeed = LeftWheelSpeed>0?JointWLP.MaximiumRotRate:-JointWLP.MaximiumRotRate;
	// 	}
	// }
	// if (abs(LeftWheelSpeed)>JointWLP.MaximiumRotRate)
	// {
	// 	LeftWheelSpeed = LeftWheelSpeed>0?JointWLP.MaximiumRotRate:-JointWLP.MaximiumRotRate;
	// 	RightWheelSpeed = LeftWheelSpeed + 2*DiffSpeedControl;
	// 	if (abs(RightWheelSpeed)>JointWRP.MaximiumRotRate)
	// 	{
	// 		RightWheelSpeed = RightWheelSpeed>0?JointWRP.MaximiumRotRate:-JointWRP.MaximiumRotRate;
	// 	}
	// }

	SetJointSpeed(JointWR, 0, RightWheelSpeed);
	SetJointSpeed(JointWL, 0, LeftWheelSpeed);
	AngleErrorHis = AngleError;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is used to control the model drive to a specofc point on the ground plane
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ModelController::Move2Point(math::Vector2d DesiredPoint, math::Angle DesiredOrientation)
{
	math::Pose CurrentPosition;
	math::Vector2d startPoint;
	math::Vector2d CurrentSpeed;
	CurrentPosition = GetModelCentralCoor();
	startPoint.x = CurrentPosition.pos.x;
	startPoint.y = CurrentPosition.pos.y;
	math::Angle desireAngle = AngleCalculation2Points(startPoint, DesiredPoint);
	double CurrentDistance = startPoint.Distance(DesiredPoint);
	if (CurrentDistance > PlanarMotionStopThreshold)
	{
		CurrentSpeed.x = AccelerationRate*CurrentDistance;
		if (CurrentSpeed.x > MaxiRotationRate)
		{
			CurrentSpeed.x = MaxiRotationRate;
		}
		CurrentSpeed.y = CurrentSpeed.x;
		AnglePIDController(desireAngle, CurrentPosition.rot.GetYaw(), CurrentSpeed);
	}else{
		CurrentSpeed.x = 0;
		CurrentSpeed.y = 0;
		AnglePIDController(DesiredOrientation, CurrentPosition.rot.GetYaw(), CurrentSpeed);
	}
}

//----------- Planar Motion Control functions END --------------------