#include "WorldController.hh"

namespace gazebo{
WorldController::WorldController()
    :WorldServer()
{} // WorldController::WorldController
WorldController::~WorldController(){} // WorldController::~WorldController
void WorldController::ExtraInitializationInLoad(physics::WorldPtr _parent, 
    sdf::ElementPtr _sdf)
{
  // Build initial configuration from file
  BuildConfigurationFromXML(INTIALCONFIGURATION);

  // Here is the test for dynamic shared libraries
  // TODO: Need to be removed after testing
  // string lib_path 
  //     = "/home/edward/.gazebo/models/SMORES6Uriah/plugins/libSpiderController.so";
  // void *current_handle = NULL;
  // LibraryTemplate *spider = DynamicallyLoadedLibrary(
  //     lib_path.c_str(),current_handle);
  // spider->WhenRunning();
  // CloseLoadedLibrary(&current_handle);
} // WorldController::ExtraInitializationInLoad
void WorldController::ExtraWorkWhenModelInserted(CommandMessagePtr &msg)
{
  if (GetInitialJointSequenceSize() == 1) {
    // Confiuration connection initialized
    BuildConnectionFromXML(INTIALCONFIGURATION);
    cout<<"World: Build the connection"<<endl;
    ReadFileAndGenerateCommands("Commands");
    cout<<"World: Command been sent"<<endl;
  }
} // WorldController::ExtraWorkWhenModelInserted
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(WorldController)
} // namespace gazebo