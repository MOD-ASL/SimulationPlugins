#ifndef _GAZEBO_WORLD_CONTROLLER_HH_
#define _GAZEBO_WORLD_CONTROLLER_HH_

#include "WorldServer.hh"

#define INTIALCONFIGURATION "InitialConfiguration"

namespace gazebo{
/// Acts as the controller for normal simulation
//! Class which inherits WorldServer template
class WorldController : public WorldServer
{
 public:
  /// Constructor
  WorldController();
  /// Destructor
  ~WorldController();
  /// This function will perform extra initialization
  /*! 
    Need to read in initial configuration here
    \param _parent WorldPtr object from parent class
    \param _sdf ElementPtr object from parent class
  */
  virtual void ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf);
  /// This function will be called after all the models inserted
  /*! 
    Need to read in initial Gait table here
    \param msg A reference of the CommandMessagePtr object
  */
  virtual void ExtraWorkWhenModelInserted(CommandMessagePtr &msg);
 private:
  // transport::SubscriberPtr configSub;
  // transport::PublisherPtr configPub;
};
} // namespace gazebo
#endif
