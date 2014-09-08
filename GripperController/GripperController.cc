#include "GripperController.hh"

using std::vector;
using std::string;
using std::cout;
using std::endl;

namespace gazebo{
GripperController::GripperController() 
    : ModuleController()
{
} // GripperController::GripperController
GripperController::~GripperController()
{
} // GripperController::~GripperController
void GripperController::CollisionReceivingCallback(GzStringPtr &msg)
{
  string msgs_info = msg->data();
  // Get the names of the links that involve in the collision
  string collison1 = msgs_info.substr(0,msgs_info.find(","));
  string collison2 = msgs_info.substr(msgs_info.find(",")+1,-1);
  collision_message::msgs::CollisionMessage forward_collision_msgs;
  string link_name;
  // Magic gripper
  if (collison1.find("coke_can") != string::npos || 
      collison2.find("coke_can") != string::npos) {
    if (collison2.find("coke_can") != string::npos) {
      string tmp = collison1;
      collison1 = collison2;
      collison2 = tmp;
    }
    cout<<"Gripper: knocked the soda can"<<endl;
    physics::ModelPtr can_model = this->model->GetWorld()->GetModel(
        collison1.substr(0,collison1.find("::")));
    can_model->SetStatic(true);
    math::Pose a_offset;
    string link_str;
    math::Vector3 axis;
    if (collison2.find("CircuitHolder") != string::npos || 
        collison2.find("FrontWheel") != string::npos ) {
      a_offset.Set(0, -0.05, 0, 0, 0, 0);
      link_str = "FrontWheel";
      axis.Set(0,1,0);
    }
    if (collison2.find("UHolderBody") != string::npos ) {
      a_offset.Set(0, 0.05, 0, 0, 0, 0);
      link_str = "UHolderBody";
      axis.Set(0,1,0);
    }
    if (collison2.find("LeftWheel") != string::npos ) {
      a_offset.Set(0.05, 0, 0, 0, 0, 0);
      link_str = "LeftWheel";
      axis.Set(1,0,0);
    }
    if (collison2.find("RightWheel") != string::npos ) {
      a_offset.Set(-0.05, 0, 0, 0, 0, 0);
      link_str = "RightWheel";
      axis.Set(1,0,0);
    }
    // if (can_model->IsStatic()) {
    //   cout<<"Gripper: Can is static"<<endl;
    // }
    // this->model->AttachStaticModel(can_model,a_offset);
    if (!dynamicJoint)
    {
      dynamicJoint = this->model->GetWorld()->GetPhysicsEngine()->CreateJoint(
          "prismatic", this->model);
      dynamicJoint->Attach(this->model->GetLink(link_str), 
          can_model->GetLink("link"));
      dynamicJoint->Load(this->model->GetLink(link_str), can_model->GetLink("link"), 
          math::Pose(math::Vector3(0,-0.00,0),math::Quaternion()));
      dynamicJoint->SetAxis(0, axis);
      this->model->GetJointController()->AddJoint(dynamicJoint);
      dynamicJoint->SetHighStop(0,math::Angle(0.0000001));
      dynamicJoint->SetLowStop(0,math::Angle(-0.0000001));
      dynamicJoint->SetPosition(0,0.0);
    }
  }else{
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
  }
} // GripperController::CollisionReceivingCallback
void GripperController::ExtraOnload(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  cout<<"Gripper: Gripper: This is a gripper"<<endl;
} // GripperController::ExtraOnload
GZ_REGISTER_MODEL_PLUGIN(GripperController)
} // namespace gazebo
