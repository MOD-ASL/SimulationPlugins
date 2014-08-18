/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
// Modified and updated by: Edward Yunkai Cui
// Description: This GUI plugin is used by configuration generator, to 
//              force the graphic update when pausing the simulator. This 
//              plugin will receive information from world plugin and to  
//              call the updating function in the rendering event.

// Include Rand.hh first due to compilation error on osx (boost #5010)
// https://svn.boost.org/trac/boost/ticket/5010
//! \file System plugin to force graphic update in simulator graphics
#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include "config_message.pb.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;
/// Shared_ptr of the customized protobuf message: config_message
typedef const boost::shared_ptr<
    const config_message::msgs::ConfigMessage> ConfigMessagePtr;

namespace gazebo
{
/// This class is used to store joint angle updating information
class JointUpdate
{
 public:
  /// Constructor
  /*!
    \param modelname Model name string
    \param jointangle Pointer of the joint angle array
  */
  JointUpdate(string modelname, double *jointangle)
  {
    this->modelName = modelname;
    for (int i = 0; i < 4; ++i) {
      this->jointAngle[i] = jointangle[i];
    }
  }
  /// Set joint angles
  /*!
    \param jointangle Pointer of the joint angle array
  */
  void SetJointAngle(double *jointangle)
  {
    for (int i = 0; i < 4; ++i) {
      this->jointAngle[i] = jointangle[i];
    }
  }
  string modelName;
  double jointAngle[4];
};
/// This is the system plugin that will start with gzclient
class SystemGUI : public SystemPlugin
{
  /// Destructor
  public: virtual ~SystemGUI()
  {
    this->connections.clear();
  }
  /// Called after the plugin has been constructed.
  public: void Load(int /*_argc*/, char ** /*_argv*/)
  {
    this->connections.push_back(
        event::Events::ConnectRender(
          boost::bind(&SystemGUI::Update, this)));
  }
  /// Called once after Load
  private: void Init()
  {
    transport::NodePtr nodeConfig(new transport::Node());
    nodeConfig->Init("Configuration");
    string topicName = "~/GUIconfigSubscriber";
    this->configSub = nodeConfig->Subscribe(
        topicName,&SystemGUI::ConfigMessageDecoding, this);
    cout<<"World: subscriber topic: "<<this->configSub->GetTopic()<<endl;
  }
  /// Called do extra work in the function Update()
  public: virtual void RunsInUpdate(void) {}
  /// Called every Render event. See the Load function.
  private: void Update()
  {
    // Get scene pointer
    rendering::ScenePtr scene = rendering::get_scene();
    // Wait until the scene is initialized.
    if (!scene || !scene->GetInitialized())
      return;
    RunsInUpdate();
    // Update joint angles
    for (unsigned int i = 0; i < jointUpdateList.size(); ++i) {
      string model_name = jointUpdateList.at(i).modelName;
      if (scene->GetVisual(model_name)) {
        double *jointangles = jointUpdateList.at(i).jointAngle;
        math::Pose linkpose1(0,0,0.05,jointangles[3],0,0);
        math::Pose linkpose2(0,0,0.05,0,jointangles[0],0);
        math::Pose linkpose3(0,0,0.05,jointangles[1],0,0);
        math::Pose linkpose4(0,0,0.05,-jointangles[2],0,3.141593);
        math::Pose linkpose5(0,0,0.05,0,0,0);
        string modelvisual1 = model_name + "::" + "UHolderBody";
        string modelvisual2 = model_name + "::" + "FrontWheel";
        string modelvisual3 = model_name + "::" + "LeftWheel";
        string modelvisual4 = model_name + "::" + "RightWheel";
        string modelvisual5 = model_name + "::" + "CircuitHolder";
        if (scene->GetVisual(modelvisual1)) {
          scene->GetVisual(modelvisual1)->SetPose(linkpose1);
        }
        if (scene->GetVisual(modelvisual2)) {
          scene->GetVisual(modelvisual2)->SetPose(linkpose2);
        }
        if (scene->GetVisual(modelvisual3)) {
          scene->GetVisual(modelvisual3)->SetPose(linkpose3);
        }
        if (scene->GetVisual(modelvisual4)) {
          scene->GetVisual(modelvisual4)->SetPose(linkpose4);
        }
        if (scene->GetVisual(modelvisual5)) {
          scene->GetVisual(modelvisual5)->SetPose(linkpose5);
        }
      }
    }
  }
  /// Call back when receiving configuration information
  /*!
    \param msg ConfigMessagePtr object for incoming message
  */
  private: void ConfigMessageDecoding(ConfigMessagePtr &msg)
  {
    cout<<"GUI: Information received"<<endl;
    string module_name = msg->modelname();
    double joints_angles[4] = {0,0,0,0};
    for (int i = 0; i < 4; ++i) {
      joints_angles[i] = msg->jointangles(i);
    }
    cout<<"GUI: 4 Joint angles: "<<joints_angles[0]<<","<<joints_angles[1]<<","
        <<joints_angles[2]<<","<<joints_angles[3]<<endl;
    bool exist = false;
    for (unsigned int i = 0; i < jointUpdateList.size(); ++i) {
      if (module_name.compare(jointUpdateList.at(i).modelName) == 0) {
        jointUpdateList.at(i).SetJointAngle(joints_angles);
        exist = true;
        break;
      }
    }
    if (!exist) {
      JointUpdate new_joint_update(module_name,joints_angles);
      jointUpdateList.push_back(new_joint_update);
    }
  }
  /// All the event connections.
  private: std::vector<event::ConnectionPtr> connections;
  /// The subscriber that subscribes the configuration information
  private: transport::SubscriberPtr configSub;
  /// A list that stores all the joint update information
  private: vector<JointUpdate> jointUpdateList;
}; // class JointUpdate
// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
} // namespace gazebo