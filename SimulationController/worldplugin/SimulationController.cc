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
  math::Vector3 configuration_pose = math::Vector3(0,0,0);

  ReadLibraryPathFile(SMORES_LIBRARY_PATH_FILE);
}

void SimulationController::OnSystemRunningExtra(const common::UpdateInfo & _info)
{
  boost::filesystem::path SearchDir(smores_library_path);
  boost::filesystem::path DirContainingFile;

  if (need_to_load)
  // Loading a configuration
  {
    if (!delete_time.GetRunning()){
      UpdatePose();
      delete_time.Start();
      DeleteAllModules();
    }
    else if (delete_time.GetElapsed() > 3.0) {

      if ( FindFile( SearchDir, DirContainingFile, current_configuration_file ) )
      {
        BuildConfigurationFromXML(DirContainingFile.string(), configuration_pose);
      }
      else {
        std::cout << "Cannot find configuration file " << current_configuration_file << std::endl;
        std::cout << "\t in directory " << SearchDir << std::endl;
      }
      delete_time.Stop();
      need_to_load = false;

    }
  }

  else if (need_to_execute)
  // Executing a gait
  {
    if ( FindFile( SearchDir, DirContainingFile, current_gait_file) )
    {
      ReadFileAndGenerateCommands(DirContainingFile.string().c_str());
    }
    else {
      std::cout << "Cannot find gait file " << current_configuration_file << std::endl;
      std::cout << "\t in directory " << SearchDir << std::endl;
    }
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

void SimulationController::UpdatePose(void)
{
  cout<<"Updating Pose"<<endl;
  configuration_pose = GetCurrentConfigurationPose();
}

void SimulationController::ReadLibraryPathFile(const char* filename)
{
  ifstream infile;
  infile.open(filename);

  if (infile.is_open()) {
    while (!infile.eof()) {
      string path;
      getline(infile, path);
      if (path.find("/") == 0) {
        smores_library_path = path;
      }
      else {
        cout<<"File "<< filename<< " is not a valid SMORES library path file."<< endl;
      }
      break;
    }
    infile.close();
  }
  else {
    cout<<"File "<< filename<< " does not exist in SMORES model folder."<< endl;
  }
}

bool SimulationController::FindFile( const boost::filesystem::path& directory,
               boost::filesystem::path& path,
               const std::string& filename )
{
  bool found = false;

  const boost::filesystem::path file = filename;
  const boost::filesystem::recursive_directory_iterator end;
  const boost::filesystem::recursive_directory_iterator dir_iter( directory );

  const boost::filesystem::recursive_directory_iterator it =
        std::find_if( boost::filesystem::recursive_directory_iterator( directory ),
                      end,
                      FileEquals( filename ) );
  if ( it != end )
  {
    path = it->path();
    found = true;
  }

  return found;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(SimulationController)
} // namespace gazebo
