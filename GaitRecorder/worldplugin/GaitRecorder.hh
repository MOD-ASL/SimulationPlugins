#ifndef _GAZEBO_GAIT_RECORDER_HH_
#define _GAZEBO_GAIT_RECORDER_HH_

#include "WorldServer.hh"
#include "gait_recorder_message.pb.h"
#include "util.hh"

#define INTIALCONFIGURATION "InitialConfiguration"

using std::string;
using std::vector;

typedef const boost::shared_ptr
    <const gait_recorder_message::msgs::GaitRecMessage> GaitRecMessagePtr;

namespace gazebo{
/// The class that will be used to record the position information of a module
/// at a specific point of simulation, so later simulator can be reset to this position
class PoseRecord
{
 public:
  PoseRecord();
  PoseRecord(double joint0, double joint1, double joint2, double joint3, 
      math::Pose pos);
  void UpdateJoints(double joint0, double joint1, double joint2, double joint3, 
      math::Pose pos);
 public:
  double JointAngles[4];
  math::Pose Position;
}; // class PoseRecord
/// This clase used to store the initial position information of the frame
class Frame
{
 public:
  Frame(string frame_name);
  void AddAPosition(PoseRecord pos_rec);
  unsigned int GetPositionSize(void) const;
  PoseRecord GetPosition(unsigned int idx) const;
 public:
  string frameName;
 private:
  vector<PoseRecord> modulePositions;
};
class GaitRecorder : public WorldServer
{
 public:
  /// Constructor
  GaitRecorder();
  /// Deconstructor
  ~GaitRecorder();
  /// The function that perform extra initialization
  /// Including register GaitMessage receiving callback
  void ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf);
  /// Need to be set so that command file will not be read
  void ExtraWorkWhenModelInserted(CommandMessagePtr &msg);
  void RecordCurrentPose(Frame &current_frame);
  void SetPose(const Frame *a_frame);
 private:
  /// Callback function when receiving gait recorder message
  void GaitRecorderMessageDecoding(GaitRecMessagePtr &msg);
 private:
  vector<Frame> frames;
  transport::SubscriberPtr gaitInfoSub;
}; // class GaitRecorder
} // PoseRecord

#endif