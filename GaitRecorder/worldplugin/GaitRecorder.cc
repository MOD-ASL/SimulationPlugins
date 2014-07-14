#include "GaitRecorder.hh"

using std::string;
using std::cout;

namespace gazebo{
PoseRecord::PoseRecord()
{
  for (int i = 0; i < 4; ++i) {
    JointAngles[i] = 0;
  }
} // PoseRecord::PoseRecord
PoseRecord::PoseRecord(double joint0, double joint1, double joint2, 
    double joint3, math::Pose pos)
{
  JointAngles[0] = joint0;
  JointAngles[1] = joint1;
  JointAngles[2] = joint2;
  JointAngles[3] = joint3;
  Position = pos;
} // PoseRecord::PoseRecord
void PoseRecord::UpdateJoints(double joint0, double joint1, double joint2, 
    double joint3, math::Pose pos)
{
  JointAngles[0] = joint0;
  JointAngles[1] = joint1;
  JointAngles[2] = joint2;
  JointAngles[3] = joint3;
  Position = pos;
} // PoseRecord::UpdateJoints

Frame::Frame(string frame_name)
{
  frameName = frame_name;
} // Frame::Frame
void Frame::AddAPosition(PoseRecord pos_rec)
{
  modulePositions.push_back(pos_rec);
} // Frame::AddAPosition
unsigned int Frame::GetPositionSize(void) const
{
  return modulePositions.size();
}
PoseRecord Frame::GetPosition(unsigned int idx) const
{
  return modulePositions.at(idx);
}

GaitRecorder::GaitRecorder()
    :WorldServer()
{} // GaitRecorder::GaitRecorder
GaitRecorder::~GaitRecorder(){}
void GaitRecorder::ExtraInitializationInLoad(physics::WorldPtr _parent, 
      sdf::ElementPtr _sdf)
{
  transport::NodePtr node_gait(new transport::Node());
  node_gait->Init("GaitRecorder");
  string topic_name = "~/gaitSubscriber";
  this->gaitInfoSub = node_gait->Subscribe(
      topic_name,&GaitRecorder::GaitRecorderMessageDecoding, this);
  cout<<"World: subscriber topic: "<<this->gaitInfoSub->GetTopic()<<endl;

  // // Build initial configuration from file
  // BuildConfigurationFromXML(INTIALCONFIGURATION);
} // GaitRecorder::ExtraInitializationInLoad
void GaitRecorder::ExtraWorkWhenModelInserted(CommandMessagePtr &msg)
{
  if (GetInitialJointSequenceSize() == 1) {
    string new_frame_name = "frame0";
    Frame new_frame(new_frame_name);
    RecordCurrentPose(new_frame);
    frames.push_back(new_frame);
  }
} // GaitRecorder::ExtraWorkWhenModelInserted
void GaitRecorder::GaitRecorderMessageDecoding(GaitRecMessagePtr &msg)
{
  string modelname = msg->modelname();
  bool begin_a_new_frame = msg->newframe();
  bool play_mode = msg->playstatus();
  if (begin_a_new_frame) {
    string new_frame_name = "frame"+util::to_string((int)frames.size());
    Frame new_frame(new_frame_name);
    RecordCurrentPose(new_frame);
    frames.push_back(new_frame);
    return;
  }
  // Reset the position
  if (msg->has_resetflag()) {
    // Reset the robot position
    if (msg->resetflag()) {
      SetPose(&frames.back());
    }
  }else{
    bool load_Configuration = false;
    if (msg->has_loadconfiguration()) {
      load_Configuration = msg->loadconfiguration();
    }
    if (load_Configuration) {
      // Build initial configuration from file
      BuildConfigurationFromXML(msg->extrinfo());
    }else{
      // Sending gait table commands
      if (play_mode) {
        // Disconnect Two Model
        if (msg->has_extrinfo()) {
          string extrainfo = msg->extrinfo();
          if (extrainfo.substr(2,1).compare("+")==0 || 
              extrainfo.substr(2,1).compare("-")==0) {
            bool connect = true;
            if (extrainfo.substr(2,1).compare("-")==0) {
              connect = false;
            }
            extrainfo = extrainfo.substr(4);
            string modulename1 = extrainfo.substr(1,extrainfo.find(" ")-1);
            extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
            int node1;
            int node2;
            string modulename2;
            if (extrainfo.substr(0,1).compare("&") == 0) {
              modulename2 = extrainfo.substr(1,extrainfo.find(" ")-1);
            }else{
              node1 = atoi(extrainfo.substr(1,extrainfo.find(" ")-1).c_str());
              extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
              modulename2 = extrainfo.substr(1,extrainfo.find(" ")-1);
              extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
              node2 = atoi(extrainfo.substr(1,extrainfo.find(" ")-1).c_str());
            }
            string condition = msg->condition();
            string dependency = msg->dependency();
            if (connect) {
              SendGaitTable(GetModulePtrByName(modulename1), modulename1, 
                  modulename2, node1, node2, 1,condition,dependency);
            }else{
              SendGaitTable(GetModulePtrByName(modulename1), modulename1, 
                  modulename2, 4, 4, 2,condition,dependency);
            }
            return;
          }
        }
        bool flags[4] = {true,true,true,true};
        double joints_values[4] = {0,0,0,0};
        joints_values[0] = msg->jointangles(0);
        joints_values[1] = msg->jointangles(1);
        joints_values[2] = msg->jointangles(2);
        joints_values[3] = msg->jointangles(3);
        string condition = msg->condition();
        string dependency = msg->dependency();
        unsigned int time_interval = msg->timer();
        if (time_interval == 0) {
          SendGaitTable(GetModulePtrByName(modelname), flags, joints_values, 3, 
              condition, dependency);
        }else{
          SendGaitTable(GetModulePtrByName(modelname), flags, joints_values, 3, 
              time_interval, condition, dependency);
        }
      }
      // User specify a joint angle
      if (!play_mode) {
        // Disconnect Two Model
        // TODO: Need an elegent way to deal with none play mode disconnection
        if (msg->has_extrinfo()) {
          string extrainfo = msg->extrinfo();
          if (extrainfo.substr(2,1).compare("+")==0 || 
              extrainfo.substr(2,1).compare("-")==0) {
            bool connect = true;
            if (extrainfo.substr(2,1).compare("-")==0) {
              connect = false;
            }
            extrainfo = extrainfo.substr(4);
            string modulename1 = extrainfo.substr(1,extrainfo.find(" ")-1);
            extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
            int node1;
            int node2;
            string modulename2;
            if (extrainfo.substr(0,1).compare("&") == 0) {
              modulename2 = extrainfo.substr(1,extrainfo.find(" ")-1);
            }else{
              node1 = atoi(extrainfo.substr(1,extrainfo.find(" ")-1).c_str());
              extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
              modulename2 = extrainfo.substr(1,extrainfo.find(" ")-1);
              extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
              node2 = atoi(extrainfo.substr(1,extrainfo.find(" ")-1).c_str());
            }

            if (connect) {
              ActiveConnect(GetModulePtrByName(modulename1), 
                  GetModulePtrByName(modulename2), node1, node2);
            }else{
              Disconnect(modulename1, modulename2);
            }
            return;
          }
        }
        bool flags[4] = {true,true,true,true};
        double joints_values[4] = {0,0,0,0};
        joints_values[0] = msg->jointangles(0);
        joints_values[1] = msg->jointangles(1);
        joints_values[2] = msg->jointangles(2);
        joints_values[3] = msg->jointangles(3);
        SendGaitTableInstance(GetModulePtrByName(modelname), flags, joints_values,3);
        cout<<"World: model problem: "<<modelname<<endl;
        currentWorld->GetModel(modelname)->GetJoint("Front_wheel_hinge")
            ->SetAngle(0,msg->jointangles(0));
        currentWorld->GetModel(modelname)->GetJoint("Left_wheel_hinge")
            ->SetAngle(0,msg->jointangles(1));
        currentWorld->GetModel(modelname)->GetJoint("Right_wheel_hinge")
            ->SetAngle(0,msg->jointangles(2));
        currentWorld->GetModel(modelname)->GetJoint("Center_hinge")
            ->SetAngle(0,msg->jointangles(3));
      }
    }
  }
} // GaitRecorder::GaitRecorderMessageDecoding
void GaitRecorder::RecordCurrentPose(Frame &current_frame)
{
  for (unsigned int i = 0; i < GetModuleListSize(); ++i) {
    double joint0 = GetModulePtrByIDX(i)->ModuleObject->GetJoint("Front_wheel_hinge")
        ->GetAngle(0).Radian();
    double joint1 = GetModulePtrByIDX(i)->ModuleObject->GetJoint("Left_wheel_hinge")
        ->GetAngle(0).Radian();
    double joint2 = GetModulePtrByIDX(i)->ModuleObject->GetJoint("Right_wheel_hinge")
        ->GetAngle(0).Radian();
    double joint3 = GetModulePtrByIDX(i)->ModuleObject->GetJoint("Center_hinge")
        ->GetAngle(0).Radian();
    math::Pose currentmodel = GetModulePtrByIDX(i)->ModuleObject->GetWorldPose();
    PoseRecord new_record(joint0,joint1,joint2,joint3,currentmodel);
    current_frame.AddAPosition(new_record);
  }
} // GaitRecorder::RecordCurrentPose
void GaitRecorder::SetPose(const Frame *a_frame)
{
  this->currentWorld->DisableAllModels();
  // common::Time::MSleep(1000);
  for (unsigned int i = 0; i < a_frame->GetPositionSize(); ++i) {
    bool flags[4] = {true,true,true,true};
    SendGaitTableInstance(
        GetModulePtrByIDX(i), flags, a_frame->GetPosition(i).JointAngles, 3);
    GetModulePtrByIDX(i)->ModuleObject->GetJoint("Front_wheel_hinge")->SetAngle(
        0,a_frame->GetPosition(i).JointAngles[0]);
    GetModulePtrByIDX(i)->ModuleObject->GetJoint("Left_wheel_hinge")->SetAngle(
        0,a_frame->GetPosition(i).JointAngles[1]);
    GetModulePtrByIDX(i)->ModuleObject->GetJoint("Right_wheel_hinge")->SetAngle(
        0,a_frame->GetPosition(i).JointAngles[2]);
    GetModulePtrByIDX(i)->ModuleObject->GetJoint("Center_hinge")->SetAngle(
        0,a_frame->GetPosition(i).JointAngles[3]);
    GetModulePtrByIDX(i)->ModuleObject->SetWorldPose(a_frame->GetPosition(i).Position);
  }
  this->currentWorld->EnableAllModels();
  // common::Time::MSleep(5000);
  // this->currentWorld->EnablePhysicsEngine(true);
} // GaitRecorder::SetPose
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GaitRecorder)
} // namespace gazebo