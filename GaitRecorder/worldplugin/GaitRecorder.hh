/*! \file Defined the GaitRecorder worlplugin, classes and types that used by 
this object */
#ifndef _GAZEBO_GAIT_RECORDER_HH_
#define _GAZEBO_GAIT_RECORDER_HH_

#include "WorldServer.hh"
#include "gait_recorder_message.pb.h"
#include "util.hh"
/// Initial configuration file path
#define INTIALCONFIGURATION "InitialConfiguration"

using std::string;
using std::vector;
/// Shared_ptr of the customized protobuf message: gait_recorder_message
typedef const boost::shared_ptr
    <const gait_recorder_message::msgs::GaitRecMessage> GaitRecMessagePtr;

namespace gazebo{
/// The class that will be used to record the position information of a module
/*! Record of the position of a single module at a specific point of simulation, 
so later the module can be reset to this position */
class PoseRecord
{
 public:
  /// Constructor
  PoseRecord();
  /// Constructor
  /*!
    \param joint0 Joint angle of joint 0
    \param joint1 Joint angle of joint 1
    \param joint2 Joint angle of joint 2
    \param joint3 Joint angle of joint 3
    \param pos Global position of the current module
  */
  PoseRecord(double joint0, double joint1, double joint2, double joint3, 
      math::Pose pos);
  /// Update the current record
  /*!
    \param joint0 Joint angle of joint 0
    \param joint1 Joint angle of joint 1
    \param joint2 Joint angle of joint 2
    \param joint3 Joint angle of joint 3
    \param pos Global position of the current module
  */
  void UpdateJoints(double joint0, double joint1, double joint2, double joint3, 
      math::Pose pos);
 public:
  /// Joint angles of the module
  double JointAngles[4];
  /// Global position of the module
  math::Pose Position;
}; // class PoseRecord
/// This clase used to store the initial position information of a section
/*!
  Name has been changed, still using the old name 'Frame' instead of 'Section'
*/
class Frame
{
 public:
  /// Constructor
  /*!
    \param frame_name Section identity string
  */
  Frame(string frame_name);
  /// Add a position record of a module
  /*!
    \param pos_rec PoseRecord object
  */
  void AddAPosition(PoseRecord pos_rec);
  /// Get the size of position records
  /*!
    \return Integer, size of the position records
  */
  unsigned int GetPositionSize(void) const;
  /// Get a position record with a position index
  /*!
    \param idx Position index
    \return A PoseRecord object
  */
  PoseRecord GetPosition(unsigned int idx) const;
 public:
  /// Section name string
  string frameName;
 private:
  /// Vector of PoseRecord objects
  vector<PoseRecord> modulePositions;
};
/// Gait recorder customized worldplugin
class GaitRecorder : public WorldServer
{
 public:
  /// Constructor
  GaitRecorder();
  /// Deconstructor
  ~GaitRecorder();
  /// The function that perform extra initialization
  /*! Including register GaitMessage receiving callback 
    \param _parent Current WorldPtr object
    \param _sdf ElementPtr object of the current world
  */
  void ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf);
  /// Callback after a model inserted
  /*! Need to be set so that command file will not be read 
    \param msg CommandMessagePtr object of the incoming message
  */
  void ExtraWorkWhenModelInserted(CommandMessagePtr &msg);
  /// Record the current position of all modules
  /*!
    \param current_frame Reference of a Section object
  */
  void RecordCurrentPose(Frame &current_frame);
  /// Set the position to the initial state of a section
  /*!
    \param a_frame Pointer of a Section object
  */
  void SetPose(const Frame *a_frame);
 private:
  /// Callback function when receiving gait recorder message
  /*!
    \param msg GaitRecMessagePtr object of incoming message
  */
  void GaitRecorderMessageDecoding(GaitRecMessagePtr &msg);
 private:
  /// A list of sections
  vector<Frame> frames;
  /// Subscriber of the gait python Gui program
  transport::SubscriberPtr gaitInfoSub;
}; // class GaitRecorder
} // PoseRecord

#endif