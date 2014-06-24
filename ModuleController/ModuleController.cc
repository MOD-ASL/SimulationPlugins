#include "ModuleController.hh"

using std::vector;
using std::string;
using std::cout;
using std::endl;

namespace gazebo{
ModuleController::ModuleController() 
    : ModelPlugin(), jointAngleKPID(5.5,0,0.75), modelAngleKPID(1,0,0)
{
  // Variables Initialization
  // Dimension Parameter Initialization
  // TODO: This parameter should be determined in configuration file
  maxiRotationRate = 2.4086;
  accelerationRate = 8;
  planarMotionStopThreshold = 0.016;
  wheelRadius =  0.045275;
  // Execution Parameter Initialization
  executionState = 0;
  for (unsigned i = 0; i < 4; ++i) jointAngleShouldBe[i] = 0;
  lftWheelSpeed = 0;
  rgtWheelSpeed = 0;
  startExecution = false;
  commandPriority = 0;
  // A hint of model been initialized
  cout<<"Model Initiated\n";
} // ModuleController::ModuleController
ModuleController::~ModuleController()
{
  this->model.reset();
  this->jointWR.reset();
  this->jointWL.reset();
  this->jointWF.reset();
  this->jointCB.reset();
} // ModuleController::~ModuleController
double ModuleController::RevolutionSpeedCal(physics::JointPtr joint, 
    const int axis_index)
{
  // The unit of this rotation rate is "rad/s"
  double rev_speed = joint->GetVelocity(axis_index);
  return rev_speed;
} // ModuleController::RevolutionSpeedCal
math::Pose ModuleController::GetModelCentralCoor(void)
{
  physics::ModelState current_model_state(model);
  math::Pose model_position = current_model_state.GetPose();
  return model_position;
} // ModuleController::GetModelCentralCoor
math::Angle ModuleController::AngleCalculation2Points(
    math::Vector2d start_point, math::Vector2d end_point)
{
  double displacement_X = end_point.x - start_point.x;
  double displacement_Y = end_point.y - start_point.y;
  double angle_C = atan2(displacement_Y,displacement_X);
  math::Angle return_angle(angle_C + PI/2);
  return return_angle;
} // ModuleController::AngleCalculation2Points
double ModuleController::ComplementaryFilter(double filtering_value, 
    double complement_filter_par, double *value_history)
{
  double filtered_value;
  filtered_value = (1 - complement_filter_par)*filtering_value 
      + complement_filter_par*(*value_history);
  *value_history = filtered_value;
  return filtered_value;
} // ModuleController::ComplementaryFilter
double ModuleController::ComplementaryFilter(double filtering_value, 
    double *value_history)
{
  return ComplementaryFilter(filtering_value, 0.9, value_history);
} // ModuleController::ComplementaryFilter
SmoresJoint & ModuleController::GetJointPlus(int node_ID)
{
  if (node_ID == 1){
    return jointWLP;
  }
  if (node_ID == 2){
    return jointWRP;
  }
  if (node_ID == 3){
    return jointCBP;
  }
  return jointWFP;
} // ModuleController::GetJointPlus
int ModuleController::GetJointAxis(int node_ID)
{
  int axis_idx = 0;
  switch(node_ID){
    case 0:axis_idx = 1;break;
    case 1:axis_idx = 0;break;
    case 2:axis_idx = 0;break;
    case 3:axis_idx = 0;break;
  }
  return axis_idx;
} // ModuleController::GetJointAxis
math::Angle ModuleController::GetJointAngle(
    physics::JointPtr current_joint, int rot_axis)
{
  math::Angle current_joint_angle = current_joint->GetAngle(rot_axis);
  return current_joint_angle;
} // ModuleController::GetJointAngle
void ModuleController::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Model plugin initialization, mainly for joints
  SystemInitialization(_parent);
  // Initialize the welcome information subscribers
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  this->welcomeInfoSub = node->Subscribe(
      "~/Welcome",&ModuleController::WelcomInfoProcessor, this);  
  // Bind function which will be executed in each iteration
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ModuleController::OnSystemRunning, this, _1));
} // ModuleController::Load
void ModuleController::WelcomInfoProcessor(GzStringPtr &msg)
{
  string info_received = msg->data();
  // TODO: currently there is no reaction for receiving welcome information
} // ModuleController::WelcomInfoProcessor
void ModuleController::SystemInitialization(physics::ModelPtr parent_model)
{
  // Get all the pointers set
  this->model = parent_model;
  this->jointWR = model->GetJoint("Right_wheel_hinge");
  this->jointWL = model->GetJoint("Left_wheel_hinge");
  this->jointWF = model->GetJoint("Front_wheel_hinge");
  this->jointCB = model->GetJoint("Center_hinge");
  // Joint plus initialization
  jointWRP.jointPtr = jointWR;
  jointWRP.needToBeSet = false;
  jointWRP.jointErrorHis = 0;
  jointWRP.jointErrorAccu = 0;
  jointWLP.jointPtr = jointWL;
  jointWLP.needToBeSet = false;
  jointWLP.jointErrorHis = 0;
  jointWLP.jointErrorAccu = 0;
  jointWFP.jointPtr = jointWF;
  jointWFP.needToBeSet = false;
  jointWFP.jointErrorHis = 0;
  jointWFP.jointErrorAccu = 0;
  jointCBP.jointPtr = jointCB;
  jointCBP.needToBeSet = false;
  jointCBP.jointErrorHis = 0;
  jointCBP.jointErrorAccu = 0;
  // Setting the model states through sdf elements
  // Setting the maximium torque of the two wheels
  this->jointWR->SetMaxForce(
      0,jointWR->GetSDF()->GetElement("physics")->GetElement("ode")
      ->GetElement("max_force")->Get<double>());
  this->jointWL->SetMaxForce(
      0,jointWL->GetSDF()->GetElement("physics")->GetElement("ode")
      ->GetElement("max_force")->Get<double>());
  // Setting the maximium torque of the front wheel
  this->jointWF->SetMaxForce(
      0,jointWF->GetSDF()->GetElement("physics")->GetElement("ode")
      ->GetElement("max_force")->Get<double>());
  // Setting the maximium torque of the body bending joint
  this->jointCB->SetMaxForce(
      0,jointCB->GetSDF()->GetElement("physics")->GetElement("ode")
      ->GetElement("max_force")->Get<double>());
  // Model Level publishers and subscribers setup
  string topic_name = "~/" + model->GetName() + "_world";
  string topic_name_pub = "~/" + model->GetName() + "_model";
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init(model->GetName());
  this->commandSub = node->Subscribe(
      topic_name,&ModuleController::CommandDecoding, this);
  this->commandPub = node->Advertise<command_message::msgs::CommandMessage>(
      topic_name_pub);
  CollisionPubAndSubInitialization();
  // Finishing initialization feedback  
  command_message::msgs::CommandMessage feed_back_message;
  feed_back_message.set_messagetype(5);
  feed_back_message.set_stringmessage(model->GetName());
  commandPub->Publish(feed_back_message);
} // ModuleController::SystemInitialization
void ModuleController::OnSystemRunning(const common::UpdateInfo & /*_info*/)
{
  // Update the states that are stored in joint plus
  // It is mainly for joint angle PID controller
  JointAngleUpdateInJointPlus();
  // Command execution
  if (executionState == 1){
    math::Vector2d final_position(targetPosition.pos.x,targetPosition.pos.y);
    math::Angle final_orientation(targetPosition.pos.z);
    Move2Point(final_position,final_orientation);
    if (startExecution){
      PositionTracking(); // A feedback message will be sent after acheiving the goal
    }
  }
  if (executionState == 2){
    for (int i = 0; i < 4; ++i){
      JointPIDController(jointAngleShouldBe[i], &GetJointPlus(i));
    }
    if (startExecution){
      JointAngleTracking(); // A feedback message will be sent after acheiving the goal
    }
  }
  if (executionState == 3){
    JointPIDController(jointAngleShouldBe[0], &GetJointPlus(0));
    JointPIDController(jointAngleShouldBe[3], &GetJointPlus(3));
    SetJointSpeed(jointWL, 0, lftWheelSpeed);
    SetJointSpeed(jointWR, 0, rgtWheelSpeed);
  }
} // ModuleController::OnSystemRunning
void ModuleController::CollisionPubAndSubInitialization(void)
{
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  // Initialize the node with the model name
  node->Init(model->GetName());
  string topic_name = "~/" + model->GetName() + "::FrontWheel::front_contact";
  this->linkCollisonSub[0] = node->Subscribe(
      topic_name,&ModuleController::CollisionReceivingCallback,this);
  topic_name = "~/" + model->GetName() + "::UHolderBody::UHolder_contact";
  this->linkCollisonSub[1] = node->Subscribe(
      topic_name,&ModuleController::CollisionReceivingCallback,this);
  topic_name = "~/" + model->GetName() + "::LeftWheel::LeftWheel_contact";
  this->linkCollisonSub[2] = node->Subscribe(
      topic_name,&ModuleController::CollisionReceivingCallback,this);
  topic_name = "~/" + model->GetName() + "::RightWheel::RightWheel_contact";
  this->linkCollisonSub[3] = node->Subscribe(
      topic_name,&ModuleController::CollisionReceivingCallback,this);

  string collision_pub_name = "~/"+model->GetName()+"_Collision";
  collisionInfoToServer = node
      ->Advertise<collision_message::msgs::CollisionMessage>(collision_pub_name);
} // ModuleController::CollisionPubAndSubInitialization
void ModuleController::CollisionReceivingCallback(GzStringPtr &msg)
{
  string msgs_info = msg->data();
  // Get the names of the links that involve in the collision
  string collison1 = msgs_info.substr(0,msgs_info.find(","));
  string collison2 = msgs_info.substr(msgs_info.find(",")+1,-1);
  collision_message::msgs::CollisionMessage forward_collision_msgs;
  string link_name;
  // Changed the order of reported collisions
  // Make the link of the current model at the first
  int model_name_len = model->GetName().length();
  if(collison1.substr(0,collison1.find("::")).compare(model->GetName())==0){
    forward_collision_msgs.set_collision1(collison1);
    forward_collision_msgs.set_collision2(collison2);
    link_name = collison1.substr(
        model_name_len+2,
        collison1.find(":",model_name_len+2)-2-model_name_len);
  }else{
    forward_collision_msgs.set_collision1(collison2);
    forward_collision_msgs.set_collision2(collison1);
    link_name = collison2.substr(
        model_name_len+2,
        collison2.find(":",model_name_len+2)-2-model_name_len);
  }
  msgs::Pose position_of_collision_link 
      = msgs::Convert(model->GetLink(link_name)->GetWorldPose());
  forward_collision_msgs.mutable_positioncol1()
      ->CopyFrom(position_of_collision_link);
  // Publish this information to world plugin
  collisionInfoToServer->Publish(forward_collision_msgs);
} // ModuleController::CollisionReceivingCallback
void ModuleController::CommandDecoding(CommandMessagePtr &msg)
{
  int command_type = msg->messagetype();
  command_message::msgs::CommandMessage feed_back_message;
  feed_back_message.set_messagetype(0);
  feed_back_message.set_stringmessage(model->GetName()+":");
  switch(command_type){
    // DEPRECATED: this information seems never to be used
    //             due to the new representation
    case 1:{nameOfConnectedModels.push_back(msg->stringmessage());break;}
    case 2:{
      this->executionState = 1;
      this->targetPosition = gazebo::msgs::Convert(msg->positionneedtobe());
      commandPub->Publish(feed_back_message);
      break;
    }
    case 3:{
      this->executionState = 2;
      for (int i = 0; i < 4; ++i){
        if (msg->jointgaittablestatus(i)){
          this->jointAngleShouldBe[i] = msg->jointgaittable(i);
        }
      }
      // Joint setting log
      cout<<"Model: "
          <<model->GetName()<<":joint0:"<<jointAngleShouldBe[0]<<endl;
      cout<<"Model: "
          <<model->GetName()<<":joint1:"<<jointAngleShouldBe[1]<<endl;
      cout<<"Model: "
          <<model->GetName()<<":joint2:"<<jointAngleShouldBe[2]<<endl;
      cout<<"Model: "
          <<model->GetName()<<":joint3:"<<jointAngleShouldBe[3]<<endl;
      commandPub->Publish(feed_back_message);
      break;
    }
    case 4:{
      this->executionState = 3;
      for (int i = 0; i < 4; ++i){
        if (i==0 || i==3){
          if (msg->jointgaittablestatus(i)){
            this->jointAngleShouldBe[i] = msg->jointgaittable(i);
          }
        }else{
          if (msg->jointgaittablestatus(i) && i == 1){
            lftWheelSpeed = msg->jointgaittable(i);
          }
          if (msg->jointgaittablestatus(i) && i == 2){
            rgtWheelSpeed = msg->jointgaittable(i);
          }
        }
      }
      commandPub->Publish(feed_back_message);
      break;
    }
  }
  // TODO: If we can confirm that command 5 will be abandoned
  //       Then modification will be needed for this line
  if (command_type == 0 || command_type == 5){
    startExecution = false;
  }else{
    startExecution = true;
    commandPriority = msg->priority();
  }
} // ModuleController::CommandDecoding
void ModuleController::SetJointAngleForce(physics::JointPtr current_joint, 
    int rot_axis, math::Angle angle_desired)
{
  current_joint->SetAngle(rot_axis, angle_desired);
} // ModuleController::SetJointAngleForce
void ModuleController::JointPIDController(double angle_desired_radian, 
    double desire_speed, SmoresJoint *current_joint) 
    // Desired Speed is a percentage of the maximum speed
{
  double angle_error = 0, angle_diff_error = 0;
  double speed = 0;
  int rot_axis = 0;
  math::Angle angle_desired(angle_desired_radian);
  angle_error = (angle_desired - current_joint->jointAngleNow).Radian();
  angle_diff_error = angle_error - current_joint->jointErrorHis;
  current_joint->jointErrorAccu += angle_error;
  speed = jointAngleKPID.x*angle_error 
      + jointAngleKPID.y*current_joint->jointErrorAccu 
      + jointAngleKPID.z*angle_diff_error;
  if (abs(speed)> maxiRotationRate*desire_speed){
    speed = 
        speed>0?maxiRotationRate*desire_speed:(-maxiRotationRate*desire_speed);
  }
  SetJointSpeed(current_joint->jointPtr,rot_axis,speed);
  current_joint->jointErrorHis = angle_error;
} // ModuleController::JointPIDController
void ModuleController::JointPIDController(double angle_desired_radian, 
    SmoresJoint *current_joint)
{
  JointPIDController(angle_desired_radian, 0.8, current_joint);
} // ModuleController::JointPIDController
void ModuleController::JointAngleUpdateInJointPlus(void)
{
  jointWRP.jointAngleNow = GetJointAngle(jointWR,0);
  jointWLP.jointAngleNow = GetJointAngle(jointWL,0);
  jointWFP.jointAngleNow = GetJointAngle(jointWF,0);
  jointCBP.jointAngleNow = GetJointAngle(jointCB,0);
} // ModuleController::JointAngleUpdateInJointPlus
void ModuleController::SetJointSpeed(physics::JointPtr current_joint, 
    int rot_axis, double speed_desired)
    // TODO: The first PID may not be necessary
{
  if (abs(speed_desired)>maxiRotationRate){
    speed_desired = speed_desired>0?maxiRotationRate:-maxiRotationRate;
  }
  current_joint->SetVelocity(rot_axis,speed_desired);
} // ModuleController::SetJointSpeed
void ModuleController::JointAngleTracking(void)
{
  bool execution_finished_flag = true;
  if (abs(GetJointAngle(jointWF,0).Radian()-jointAngleShouldBe[0])>EXECUTIONERROR){
    execution_finished_flag = false;
  }
  if (abs(GetJointAngle(jointWL,0).Radian()-jointAngleShouldBe[1])>EXECUTIONERROR){
    execution_finished_flag = false;
  }
  if (abs(GetJointAngle(jointWR,0).Radian()-jointAngleShouldBe[2])>EXECUTIONERROR){
    execution_finished_flag = false;
  }
  if (abs(GetJointAngle(jointCB,0).Radian()-jointAngleShouldBe[3])>EXECUTIONERROR){
    execution_finished_flag = false;
  }
  if (execution_finished_flag){
    command_message::msgs::CommandMessage feed_back_message;
    feed_back_message.set_messagetype(0);
    feed_back_message.set_priority(commandPriority);
    feed_back_message.set_stringmessage(model->GetName()+":finished");
    commandPub->Publish(feed_back_message);
  }
} // ModuleController::JointAngleTracking
void ModuleController::PositionTracking(void)
{
  bool execution_finished_flag = true;
  math::Vector2d module_pos(
      GetModelCentralCoor().pos.x,GetModelCentralCoor().pos.y);
  math::Vector2d desired_pos(targetPosition.pos.x,targetPosition.pos.y);
  if (module_pos.Distance(desired_pos)>2*EXECUTIONERROR){
    execution_finished_flag = false;
    cout<<"Model: distance is : "<<module_pos.Distance(desired_pos)<<endl;
  }
  if (abs(targetPosition.pos.z-GetModelCentralCoor().rot.GetYaw())
      >(EXECUTIONERROR+0.002)){
    execution_finished_flag = false;
    cout<<"Model: angle difference is : "
        <<abs(targetPosition.pos.z-GetModelCentralCoor().rot.GetYaw())<<endl;
  }
  if (execution_finished_flag){
    command_message::msgs::CommandMessage feed_back_message;
    feed_back_message.set_messagetype(0);
    feed_back_message.set_priority(commandPriority);
    feed_back_message.set_stringmessage(model->GetName()+":finished");
    commandPub->Publish(feed_back_message);
  }
} // ModuleController::PositionTracking
void ModuleController::AnglePIDController(math::Angle desired_angle, 
    math::Angle current_angle, math::Vector2d current_speed)
{
  double angle_error = 0, angle_error_diff = 0, diff_speed_control = 0;
  static double angle_error_his = 0;
  static double angle_error_acu = 0;
  double left_wheel_speed = 0, right_wheel_speed = 0;
  while (abs(desired_angle.Degree()) > 180){
    double degree_angle = desired_angle.Degree()>0?
        desired_angle.Degree()-360:desired_angle.Degree()+360;
    desired_angle.SetFromDegree(degree_angle);
  }
  if (abs((desired_angle-current_angle).Degree())>180){
    angle_error = (desired_angle-current_angle).Degree()>0?
        (desired_angle-current_angle).Radian()-2*PI:
        (desired_angle-current_angle).Radian()+2*PI;
  }else{
    angle_error = (desired_angle-current_angle).Radian();
  }
  angle_error_acu += angle_error;
  angle_error_diff = angle_error - angle_error_his;
  diff_speed_control = modelAngleKPID.x*angle_error 
      + modelAngleKPID.y*angle_error_acu + modelAngleKPID.z*angle_error_diff;
  left_wheel_speed = current_speed.x - diff_speed_control;
  right_wheel_speed = current_speed.y + diff_speed_control;
  SetJointSpeed(jointWR, 0, right_wheel_speed);
  SetJointSpeed(jointWL, 0, left_wheel_speed);
  angle_error_his = angle_error;
} // ModuleController::AnglePIDController
void ModuleController::Move2Point(math::Vector2d desired_point, 
    math::Angle desired_orientation)
{
  math::Pose current_position = GetModelCentralCoor();
  math::Vector2d start_point(current_position.pos.x, current_position.pos.y);
  math::Vector2d current_speed(0,0);
  math::Angle desire_angle = AngleCalculation2Points(start_point, desired_point);
  double current_distance = start_point.Distance(desired_point);
  if (current_distance > planarMotionStopThreshold){
    current_speed.x = accelerationRate*current_distance;
    if (current_speed.x > maxiRotationRate){
      current_speed.x = maxiRotationRate;
    }
    current_speed.y = current_speed.x;
    AnglePIDController(desire_angle, current_position.rot.GetYaw(), current_speed);
  }else{
    AnglePIDController(
        desired_orientation, current_position.rot.GetYaw(), current_speed);
  }
} // ModuleController::Move2Point
GZ_REGISTER_MODEL_PLUGIN(ModuleController)
} // namespace gazebo