#ifndef _GAZEBO_WORLD_CONTROLLER_HH_
#define _GAZEBO_WORLD_CONTROLLER_HH_

#include "WorldServer.hh"

namespace gazebo{
class WorldController : public WorldServer
{
 public:
  WorldController();
  ~WorldController();
  /// This function will perform extra initialization
  /// Need to read in initial configuration here
  virtual void ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf);
  /// Need to read in initial Gait table here
  virtual void ExtraWorkWhenModelInserted(CommandMessagePtr &msg);
 private:
  // transport::SubscriberPtr configSub;
  // transport::PublisherPtr configPub;
};
} // namespace gazebo
#endif
