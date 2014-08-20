#include "SimulationController.hh"

namespace gazebo{
SimulationController::SimulationController()
		:WorldServer()
{}
SimulationController::~SimulationController(){}
void SimulationController::ExtraInitializationInLoad(physics::WorldPtr _parent,
      sdf::ElementPtr _sdf)
{
  transport::NodePtr node_simControl(new transport::Node());
  node_simControl->Init("SimulationController");
  string topic_name = "~/simControlSubscriber";
  this->simControlSub = node_simControl->Subscribe(
  		topic_name,&SimulationController::SimControlMessageDecoding, this);
  cout<<"World: subscriber topic: "<<this->simControlSub->GetTopic()<<endl;
  topic_name = "~/GUIsimControlSubscriber";
  this->simControlPub = node_simControl->Advertise<
  		sim_control_message::msgs::SimControlMessage>(topic_name);

  need_to_load = false;
  need_to_execute = false;
  current_gait_file = "";
  current_configuration_file = "";
  delete_time = common::Timer();
}

void SimulationController::OnSystemRunningExtra(const common::UpdateInfo & _info)
{
  if (need_to_load)
  {
    if (!delete_time.GetRunning()){
      delete_time.Start();
      DeleteAllModules();
    }
    else if (delete_time.GetElapsed() > 5.0) {
      BuildConfigurationFromXML(current_configuration_file);
      delete_time.Stop();
      need_to_load = false;
    }
  }

  else if (need_to_execute)
  {
    ReadFileAndGenerateCommands(current_gait_file.c_str());
    need_to_execute = false;
  }

} // WorldServer::OnSystemRunningExtra

void SimulationController::ExtraWorkWhenModelInserted(CommandMessagePtr &msg){}
void SimulationController::SimControlMessageDecoding(SimControlMessagePtr &msg)
{
  cout<<"World: Information received"<<endl;
  if (msg->gaitname().size() != 0) {
    need_to_execute = true;
    current_gait_file = msg->gaitname();
  }
  else if (msg->configurationname().size() != 0) {
    need_to_load = true;
    current_configuration_file = msg->configurationname();
  }
}
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SimulationController)
} // namespace gazebo
