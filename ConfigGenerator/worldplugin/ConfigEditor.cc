#include "ConfigEditor.hh"

namespace gazebo{
ConfigEditor::ConfigEditor()
		:WorldServer()
{}
ConfigEditor::~ConfigEditor(){}
void ConfigEditor::InsertModel(string name, math::Pose position)
{
  if (!currentWorld->GetModel(name)) {
    sdf::SDFPtr model_sdf;
    model_sdf.reset(new sdf::SDF);
    sdf::init(model_sdf);
    sdf::readFile(MODULEPATH, model_sdf);
    sdf::ElementPtr model_element = model_sdf->root->GetElement("model");
    math::Pose position_calibrate(
        math::Vector3(0, 0, -0.05), math::Quaternion(0, 0, 0));
    model_element->GetAttribute("name")->Set(name);
    model_element->GetElement("pose")->Set(position_calibrate*position);
    currentWorld->InsertModelSDF(*model_sdf);
    AddInitialPosition(position);
  }else{
    Color::Modifier red_log(Color::FG_RED);
    Color::Modifier def_log(Color::FG_DEFAULT);
    cout<<red_log<<"WARNING: World: Insertion failed: module name exists."
        <<def_log<<endl;
  }
}
void ConfigEditor::InsertModel(string name, math::Pose position, 
    string joint_angles)
{
  if (!currentWorld->GetModel(name))
  {
    sdf::SDFPtr model_sdf;
    model_sdf.reset(new sdf::SDF);
    sdf::init(model_sdf);
    sdf::readFile(MODULEPATH, model_sdf);
    sdf::ElementPtr model_element = model_sdf->root->GetElement("model");
    math::Pose position_calibrate(
        math::Vector3(0, 0, -0.05), math::Quaternion(0, 0, 0));
    model_element->GetAttribute("name")->Set(name);
    model_element->GetElement("pose")->Set(position_calibrate*position);
    currentWorld->InsertModelSDF(*model_sdf);
    AddInitialJoints(joint_angles);
    AddInitialPosition(position);
  }else{
    Color::Modifier red_log(Color::FG_RED);
    Color::Modifier def_log(Color::FG_DEFAULT);
    cout<<red_log<<"WARNING: World: Insertion failed: module name exists."
        <<def_log<<endl;
  }
}
void ConfigEditor::ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf)
{
	transport::NodePtr node_config(new transport::Node());
  node_config->Init("Configuration");
  string topic_name = "~/configSubscriber";
  this->configSub = node_config->Subscribe(
  		topic_name,&ConfigEditor::ConfigMessageDecoding, this);
  cout<<"World: subscriber topic: "<<this->configSub->GetTopic()<<endl;
  topic_name = "~/GUIconfigSubscriber";
  this->configPub = node_config->Advertise<
  		config_message::msgs::ConfigMessage>(topic_name);
}
void ConfigEditor::ExtraWorkWhenModelInserted(CommandMessagePtr &msg){}
void ConfigEditor::ConfigMessageDecoding(ConfigMessagePtr &msg)
{
  cout<<"World: Information received"<<endl;
  string module_name = msg->modelname();
  double coordinates[3] = {0,0,0};
  for (int i = 0; i < 3; ++i) {
    coordinates[i] = msg->modelposition(i);
  }
  double orientation[4] = {0,0,0,0};
  math::Quaternion orientation_pos;
  bool qua_pos_flag = false;
  if (msg->has_quaternionpos()) {
    if (msg->quaternionpos()) {
      qua_pos_flag = true;
    }
  }
  if (qua_pos_flag) {
    for (int i = 0; i < 4; ++i) {
      orientation[i] = msg->modelposition(i+3);
    }
    orientation_pos.Set(orientation[0], orientation[1], orientation[2],
        orientation[3]);
  }else{
    for (int i = 0; i < 3; ++i) {
      orientation[i] = msg->modelposition(i+3);
    }
    orientation_pos.SetFromEuler(orientation[0], orientation[1], orientation[2]);
  }
  math::Pose position_tmp(
  		math::Vector3(coordinates[0], coordinates[1], coordinates[2]), 
      orientation_pos);
  double joints_angles[4] = {0,0,0,0};
  for (int i = 0; i < 4; ++i) {
    joints_angles[i] = msg->jointangles(i);
  }
  if (GetModulePtrByName(module_name)) {
    if (msg->has_deleteflag()) {
      if (msg->deleteflag()) {
        DeleteModule(module_name);
      }
    }else{
      currentWorld->GetModel(module_name)->GetJoint("Front_wheel_hinge")
      		->SetAngle(0,joints_angles[0]);
      currentWorld->GetModel(module_name)->GetJoint("Left_wheel_hinge")
      		->SetAngle(0,joints_angles[1]);
      currentWorld->GetModel(module_name)->GetJoint("Right_wheel_hinge")
      		->SetAngle(0,joints_angles[2]);
      currentWorld->GetModel(module_name)->GetJoint("Center_hinge")
      		->SetAngle(0,joints_angles[3]);
      this->configPub->Publish(*msg);
    }
  }else{
    ostringstream strs;
    strs << joints_angles[0]<<" "<< joints_angles[1]<<" "<< joints_angles[2]<<" "
    		<< joints_angles[3];
    string joints_string = strs.str();
    cout<<"World: Joint angles: "<<joints_string<<endl;
    InsertModel(module_name, position_tmp, joints_string);
    this->configPub->Publish(*msg);
  }
}
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ConfigEditor)
} // namespace gazebo