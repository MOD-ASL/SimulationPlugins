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

// Include Rand.hh first due to compilation error on osx (boost #5010)
// https://svn.boost.org/trac/boost/ticket/5010
#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/transport/transport.hh"
#include "config_message.pb.h"

using namespace std;

namespace gazebo
{
  typedef const boost::shared_ptr<const config_message::msgs::ConfigMessage> ConfigMessagePtr;

  class JointUpdate
  {
  public:
    JointUpdate(string modelname, double jointangle[])
    {
      this->ModelName = modelname;
      for (int i = 0; i < 4; ++i)
      {
        this->JointAngle[i] = jointangle[i];
      }
    }

    void SetJointAngle(double jointangle[])
    {
      for (int i = 0; i < 4; ++i)
      {
        this->JointAngle[i] = jointangle[i];
      }
    }
  public:
    string ModelName;
    double JointAngle[4];

  };

  class SystemGUI : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~SystemGUI()
    {
      // this->connections.clear();
      // if (this->userCam)
      //   this->userCam->EnableSaveFrame(false);
      // this->userCam.reset();
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      this->connections.push_back(
          event::Events::ConnectRender(
            boost::bind(&SystemGUI::Update, this)));
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
      transport::NodePtr nodeConfig(new transport::Node());
      nodeConfig->Init("Configuration");
      string topicName = "~/GUIconfigSubscriber";
      this->configSub = nodeConfig->Subscribe(topicName,&SystemGUI::ConfigMessageDecoding, this);
      cout<<"World: subscriber topic: "<<this->configSub->GetTopic()<<endl;
      // Get a pointer to the active user camera
      // this->userCam = gui::get_active_camera();

      // Enable saving frames
      // this->userCam->EnableSaveFrame(true);

      // Specify the path to save frames into
      // this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");
    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    private: void Update()
    {
      // cout<<"GUI: here is before rendering"<<endl;
      // Get scene pointer
      rendering::ScenePtr scene = rendering::get_scene();

      // Wait until the scene is initialized.
      if (!scene || !scene->GetInitialized())
        return;

      // Print out the total number of visuals in the scene
      // std::cout << "Visual Count:" << scene->GetVisualCount() << std::endl;

      // Look for a specific visual by name.
      // if (scene->GetVisual("Module_0"))
      // {
      //   // std::cout << "GUI: visual: "<<scene->GetVisual("Module_0")->GetChild(3)->GetName()<<endl;
      //   math::Pose position = scene->GetVisual("Module_0")->GetChild(3)->GetPose();
      //   // cout<<"Gui:Current Position: "<<position.rot.GetRoll()<<","<<position.rot.GetPitch()<<","<<position.rot.GetYaw()<<endl;
      //   math::Pose newposition(0,0,0.05,-1.5708,0,0); 
      //   scene->GetVisual("Module_0")->GetChild(3)->SetPose(newposition);
      // }
      for (unsigned int i = 0; i < JointUpdateList.size(); ++i)
      {
        string modelname = JointUpdateList.at(i).ModelName;
        // cout<<"Gui: breakpoint 1"<<endl;
        // SmallDelay(10);
        if (scene->GetVisual(modelname))
        {
          double *jointangles = JointUpdateList.at(i).JointAngle;
          math::Pose linkpose1(0,0,0.05,jointangles[3],0,0);
          math::Pose linkpose2(0,0,0.05,0,jointangles[0],0);
          math::Pose linkpose3(0,0,0.05,jointangles[1],0,0);
          math::Pose linkpose4(0,0,0.05,-jointangles[2],0,3.141593);
          math::Pose linkpose5(0,0,0.05,0,0,0);
          // cout<<"Gui: breakpoint 2"<<endl;
          // SmallDelay(10);
          string modelvisual1 = modelname + "::" + "UHolderBody";
          string modelvisual2 = modelname + "::" + "FrontWheel";
          string modelvisual3 = modelname + "::" + "LeftWheel";
          string modelvisual4 = modelname + "::" + "RightWheel";
          string modelvisual5 = modelname + "::" + "CircuitHolder";
          if (scene->GetVisual(modelvisual1))
          {
            scene->GetVisual(modelvisual1)->SetPose(linkpose1);
          }
          if (scene->GetVisual(modelvisual2))
          {
            scene->GetVisual(modelvisual2)->SetPose(linkpose2);
          }
          if (scene->GetVisual(modelvisual3))
          {
            scene->GetVisual(modelvisual3)->SetPose(linkpose3);
          }
          if (scene->GetVisual(modelvisual4))
          {
            scene->GetVisual(modelvisual4)->SetPose(linkpose4);
          }
          if (scene->GetVisual(modelvisual5))
          {
            scene->GetVisual(modelvisual5)->SetPose(linkpose5);
          }
          // if (scene->GetVisual(modelname)->GetChild(4))
          // // if (scene->GetVisual(modelname)->GetChild(0) && scene->GetVisual(modelname)->GetChild(1) && scene->GetVisual(modelname)->GetChild(2) && scene->GetVisual(modelname)->GetChild(3) && scene->GetVisual(modelname)->GetChild(4))
          // {
          //   scene->GetVisual(modelname)->GetChild(3)->SetPose(linkpose1);
          //   scene->GetVisual(modelname)->GetChild(4)->SetPose(linkpose5);
          //   scene->GetVisual(modelname)->GetChild(2)->SetPose(linkpose4);
          //   scene->GetVisual(modelname)->GetChild(1)->SetPose(linkpose2);
          //   scene->GetVisual(modelname)->GetChild(0)->SetPose(linkpose3);
          // }
          // cout << "GUI: visual 0: "<<scene->GetVisual(modelname)->GetChild(0)->GetName()<<endl;  //left wheel
          // cout << "GUI: visual 1: "<<scene->GetVisual(modelname)->GetChild(1)->GetName()<<endl;  //front wheel
          // cout << "GUI: visual 2: "<<scene->GetVisual(modelname)->GetChild(2)->GetName()<<endl;  //right wheel
          // cout << "GUI: visual 4: "<<scene->GetVisual(modelname)->GetChild(4)->GetName()<<endl;  //circuit holder
          // cout << "GUI: visual 5: "<<scene->GetVisual(modelname)->GetChild(3)->GetName()<<endl; 
          // JointUpdateList.erase(JointUpdateList.begin());
        }
        // else
        // {
        //   SmallDelay(500);
        // }
      }
    }

    private: void ConfigMessageDecoding(ConfigMessagePtr &msg)
    {
      cout<<"GUI: Information received"<<endl;
      string module_name = msg->modelname();

    //   double coordinates[3] = {0,0,0};
    //   for (int i = 0; i < 3; ++i)
    //   {
    //     coordinates[i] = msg->modelposition(i);
    //   }
    //   cout<<"World: z coor is: "<<coordinates[2]<<endl;
    //   double orientation[3] = {0,0,0};
    //   for (int i = 0; i < 3; ++i)
    //   {
    //     orientation[i] = msg->modelposition(i+3);
    //   }
    //   math::Pose positionTMP(math::Vector3(coordinates[0], coordinates[1], coordinates[2]), math::Quaternion(orientation[0], orientation[1], orientation[2]));

      double joints_angles[4] = {0,0,0,0};
      for (int i = 0; i < 4; ++i)
      {
        joints_angles[i] = msg->jointangles(i);
      }
      cout<<"GUI: 4 Joint angles: "<<joints_angles[0]<<","<<joints_angles[1]<<","<<joints_angles[2]<<","<<joints_angles[3]<<endl;
      bool exist = false;
      for (unsigned int i = 0; i < JointUpdateList.size(); ++i)
      {
        if (module_name.compare(JointUpdateList.at(i).ModelName) == 0)
        {
          JointUpdateList.at(i).SetJointAngle(joints_angles);
          exist = true;
          break;
        }
      }
      if (!exist)
      {
        JointUpdate NewJointUpdate(module_name,joints_angles);
        JointUpdateList.push_back(NewJointUpdate);
      }
    }

    private: void SmallDelay(int counts)
    {
      for (int i = 0; i < counts; ++i)
      {
      }
    }

    /// Pointer the user camera.
    private: rendering::UserCameraPtr userCam;

    /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;

    private: transport::SubscriberPtr configSub;
    private: vector<JointUpdate> JointUpdateList;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}