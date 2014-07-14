#include "SuperServer.hh"

using namespace std;
using namespace rapidxml;
using namespace gazebo;

string Int2String(int number)
{
  stringstream ss; //create a stringstream
  ss << number;    //add number to the stream
  return ss.str(); //return a string with the contents of the stream
}

CollisionInformation::CollisionInformation(string collision1, string collision2, string link_collision1, string link_collision2)
{
  Model1 = collision1;
  Model2 = collision2;
  LinkOfModel1 = link_collision1;
  LinkofModel2 = link_collision2;
}

bool CollisionInformation::SameCollision(string collision1, string collision2, string link_collision1, string link_collision2)
{
  bool same_collision = false;
  if (collision1.compare(Model1)==0 && collision2.compare(Model2)==0)
  {
    if (link_collision1.compare(LinkOfModel1)==0 && link_collision2.compare(LinkofModel2)==0)
    {
      same_collision = true;
    }
  }else{
    if (collision1.compare(Model2)==0 && collision2.compare(Model1)==0)
    {
      if (link_collision1.compare(LinkofModel2)==0 && link_collision2.compare(LinkOfModel1)==0)
      {
        same_collision = true;
      }
    }
  }
  return same_collision;
}

Condition::Condition(string conditionID)
{
  this->condition_id = conditionID;
  this->total_count = 1;
  this->finished_count = 0;
  this->achieved = false;
}

PoseRecord::PoseRecord()
{
  for (int i = 0; i < 4; ++i)
  {
    JointAngles[i] = 0;
  }
}

PoseRecord::PoseRecord(double joint0, double joint1, double joint2, double joint3, math::Pose pos)
{
  JointAngles[0] = joint0;
  JointAngles[1] = joint1;
  JointAngles[2] = joint2;
  JointAngles[3] = joint3;
  Position = pos;
}

void PoseRecord::UpdateJoints(double joint0, double joint1, double joint2, double joint3, math::Pose pos)
{
  JointAngles[0] = joint0;
  JointAngles[1] = joint1;
  JointAngles[2] = joint2;
  JointAngles[3] = joint3;
  Position = pos;
}

ControlCenter::ControlCenter()
{
  numOfModules = 0;
  infoCounter = 0;
  NeedToSetPtr = 0;
  test_count = 0;
  insertModuleFlag = true;
  // this->FinishFlag = false;
  AlreadyBuild = false;
  // LastGaitMode = false;

  // All code below this line is for testing
}

void ControlCenter::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->currentWorld = _parent;
  // Create a new transport node
  transport::NodePtr node(new transport::Node());

  // Initialize the node with the world name
  node->Init(_parent->GetName());

  // Create a publisher on the ~/physics topic
  this->statePub = node->Advertise<msgs::GzString>("~/Welcome");

  transport::NodePtr nodeGait(new transport::Node());
  nodeGait->Init("GaitRecorder");
  string topicName = "~/gaitSubscriber";
  this->GaitToolSub = nodeGait->Subscribe(topicName,&ControlCenter::GaitRecorderMessageDecoding, this);
  cout<<"World: subscriber topic: "<<this->GaitToolSub->GetTopic()<<endl;
  // /gazebo/GaitRecorder/gaitSubscriber

  // Set the step time
  // stateMsg.set_max_step_size(0.01);

  // Change gravity
  // msgs::Set(physicsMsg.mutable_gravity(), math::Vector3(0.01, 0, 0.1));
  // statePub->Publish(stateMsg);
  this->addEntityConnection = event::Events::ConnectAddEntity(boost::bind(&ControlCenter::addEntity2World, this, _1));
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControlCenter::OnSystemRunning, this, _1));

  // math::Pose positionTMP(math::Vector3(0, 0, 0), math::Quaternion(1.57, 0, 0));
  // InsertModel("Module0", positionTMP);
  cout<<"World: Begin to build initial configuration"<<endl;
  BuildConfigurationFromXML();

  // Here is the test for dynamic shared libraries
  // void *lib_handle;
  // char *error;
  // LibraryTemplate * (*mkr)();

  // lib_handle = dlopen("/home/edward/.gazebo/models/SMORES6Uriah/plugins/libSpiderController.so", RTLD_LAZY);
  // if (!lib_handle) 
  // {
  //   fprintf(stderr, "%s\n", dlerror());
  //   exit(1);
  // }

  // // void *mkr = dlsym(lib_handle, "maker");
  // mkr = (LibraryTemplate * (*)())dlsym(lib_handle, "maker");
  // if ((error = dlerror()) != NULL)  
  // {
  //   fprintf(stderr, "%s\n", error);
  //   exit(1);
  // }

  // LibraryTemplate *Spider = mkr();
  // Spider->WhenRunning();
  // this->currentWorld->EnablePhysicsEngine(false);
}

void ControlCenter::addEntity2World(std::string & _info)
{
  // transport::NodePtr node(new transport::Node());
  // node->Init();
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Welcome message generation each time a new model has been added
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  msgs::GzString welcomeMsg;
  string CurrentMessage;

  int modelnumber = this->currentWorld->GetModelCount();
  numOfModules = modelnumber-1;
  cout<<"World: Number of models: "<<modelnumber<<endl;
  // cout<<"Default information: "<<_info<<endl;
  // this->statePub = node->Advertise<msgs::GzString>("~/Welcome");
  CurrentMessage = "Model"+Int2String(modelnumber);
  welcomeMsg.set_data(CurrentMessage);

  // statePub->Publish(welcomeMsg);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Dynamic publisher generation
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  transport::NodePtr node1(new transport::Node());
  node1->Init(_info);
  string NewPubName = "~/" + _info + "_world";
  transport::PublisherPtr newModulePub = node1->Advertise<command_message::msgs::CommandMessage>(NewPubName);
  NewPubName = "~/" + _info + "_model";
  transport::SubscriberPtr newModuleSub = node1->Subscribe(NewPubName,&ControlCenter::FeedBackMessageDecoding, this);
  // WorldPublisher.push_back(newWorldPub);

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Store the pointers of model into vectors
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // modelGroup.push_back(currentWorld->GetModel(_info));
  unsigned int howManyModules = moduleList.size();
  SmoresModulePtr newModule(new SmoresModule(_info, true, newModulePub, newModuleSub, howManyModules));
  newModule->ManuallyNodeInitial(newModule);
  moduleList.push_back(newModule);
  // newModule->ManuallyNodeInitial(newModule);
  // moduleList.at(howManyModules)->SaySomthing();
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Initiate the joint values
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // if (InitalJointValue.size()>0)
  // {
  //   // cout<<"World: InitalJointValue size is "<<InitalJointValue.size()<<endl;
  //   bool flags[4] = {true};
  //   double joint_angles[4] = {0};
  //   string joint_values_string = InitalJointValue.at(0);
  //   for (int i = 0; i < 4; ++i)
  //   {
  //     if (i<3)
  //     {
  //       joint_angles[i] = atof(joint_values_string.substr(0,joint_values_string.find(" ")).c_str());
  //       joint_values_string = joint_values_string.substr(joint_values_string.find(" ")+1);
  //     }else
  //     {
  //       joint_angles[i] = atof(joint_values_string.substr(0).c_str());
  //     }
  //   }
  //   cout<<"World: "<<newModule->ModuleID<<":joint0:"<<joint_angles[0]<<endl;
  //   cout<<"World: "<<newModule->ModuleID<<":joint1:"<<joint_angles[1]<<endl;
  //   cout<<"World: "<<newModule->ModuleID<<":joint2:"<<joint_angles[2]<<endl;
  //   cout<<"World: "<<newModule->ModuleID<<":joint3:"<<joint_angles[3]<<endl;
  //   SendGaitTable(newModule, flags, joint_angles);
  //   // InitalJointValue.erase(InitalJointValue.begin());
  // }

  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Dynamic subscriber of collision topic
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // string NewSubName = "~/"+_info+"_Collision";
  // transport::SubscriberPtr newWorldSub = node1->Subscribe(NewSubName,&ControlCenter::AutomaticMagneticConnectionManagement, this);
  // WorldColSubscriber.push_back(newWorldSub);
}

void ControlCenter::OnSystemRunning(const common::UpdateInfo & /*_info*/)
{
  // Main command execution procedure
  CommandManager();

  // dlclose(lib_handle);
} 

void ControlCenter::BuildConfigurationFromXML(void)
{
  file<> xmlFile(INTIALCONFIGURATION);
  // file<> xmlFile(file_name);
  xml_document<> doc;    // character type defaults to char
  doc.parse<0>(xmlFile.data());
  // cout<<"World: first node is "<<doc.first_node()->name()<<endl;
  xml_node<> *modlue_node = doc.first_node("configuration")->first_node("modules")->first_node("module");
  cout<<"World: modlue_node is "<<modlue_node->first_node("name")->value()<<endl;
  while (modlue_node)
  {
    string module_name = modlue_node->first_node("name")->value();
    cout<<"World: XML: Module name: "<<module_name<<endl;
    string position_string = modlue_node->first_node("position")->value();
    cout<<"World: XML: position: "<<position_string<<endl;
    double coordinates[3] = {0};
    for (int i = 0; i < 3; ++i)
    {
      coordinates[i] = atof(position_string.substr(0,position_string.find(" ")).c_str());
      position_string = position_string.substr(position_string.find(" ")+1);
    }
    double orientation[3] = {0};
    for (int i = 0; i < 3; ++i)
    {
      if (i==2)
      {
        orientation[i] = atof(position_string.substr(0).c_str());
      }else
      {
        orientation[i] = atof(position_string.substr(0,position_string.find(" ")).c_str());
        position_string = position_string.substr(position_string.find(" ")+1);
      }
    }
    math::Pose positionTMP(math::Vector3(coordinates[0], coordinates[1], coordinates[2]), math::Quaternion(orientation[0], orientation[1], orientation[2]));

    string joints_string = modlue_node->first_node("joints")->value();
    InsertModel(module_name, positionTMP, joints_string);
    modlue_node = modlue_node->next_sibling();
  }

}

void ControlCenter::BuildConnectionFromXML(void)
{
  file<> xmlFile(INTIALCONFIGURATION);
  // file<> xmlFile(file_name);
  xml_document<> doc;    // character type defaults to char
  doc.parse<0>(xmlFile.data());

  xml_node<> *connection_node = doc.first_node("configuration")->first_node("connections")->first_node("connection");
//       cout<<"World: connection_node is "<<connection_node->first_node("module1")->value()<<endl;
  while (connection_node)
  {
    string module1_name = connection_node->first_node("module1")->value();
    string module2_name = connection_node->first_node("module2")->value();

    string node1_ID_string = connection_node->first_node("node1")->value();
    string node2_ID_string = connection_node->first_node("node2")->value();

    int node1_ID = atoi(node1_ID_string.c_str());
    int node2_ID = atoi(node2_ID_string.c_str());

    string distance_string = connection_node->first_node("distance")->value();
    string angle_string = connection_node->first_node("angle")->value();

    double distance = atof(distance_string.c_str());
    double angle = atof(angle_string.c_str());

    SmoresModulePtr Model1Ptr = GetModulePtrByName(module1_name);
    SmoresModulePtr Model2Ptr = GetModulePtrByName(module2_name);

    ActiveConnection(Model1Ptr,Model2Ptr,node1_ID,node2_ID, angle, distance);

    connection_node = connection_node->next_sibling();
  }
}

void ControlCenter::GaitRecorderMessageDecoding(GaitRecMessagePtr &msg)
{
  string modelname = msg->modelname();
  bool beginanewframe = msg->newframe();
  bool playmode = msg->playstatus();
  if (beginanewframe)
  {
    // FrameInitialPosition = referenceModule->ModuleObject->GetLink("CircuitHolder")->GetWorldPose();
    RecordCurrentPose(FrameInitialJoints);
    return;
  }
  // Reset the position
  if (msg->has_resetflag())
  {
    // Reset the robot position
    if (msg->resetflag())
    {
      SetPose(FrameInitialJoints);
    }
    // bool flags[4] = {true,true,true,true};
    // double joints_values[4] = {0,0,0,0};
    // joints_values[0] = msg->jointangles(0);
    // joints_values[1] = msg->jointangles(1);
    // joints_values[2] = msg->jointangles(2);
    // joints_values[3] = msg->jointangles(3);
    // int group_num = msg->groupincr();
    // unsigned int time_interval = msg->timer();
    // SendGaitTable(GetModulePtrByName(modelname), flags, joints_values, group_num, time_interval);
    // LastGaitMode = true;
  }else{
    // Sending gait table commands
    if (playmode)
    {
      // Disconnect Two Model
      if (msg->has_extrinfo())
      {
        string extrainfo = msg->extrinfo();
        if (extrainfo.substr(2,1).compare("+")==0 || extrainfo.substr(2,1).compare("-")==0)
        {
          bool connect = true;
          if (extrainfo.substr(2,1).compare("-")==0)
          {
            connect = false;
          }
          extrainfo = extrainfo.substr(4);
          string modulename1 = extrainfo.substr(1,extrainfo.find(" ")-1);
          extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
          int node1;
          int node2;
          string modulename2;
          if (extrainfo.substr(0,1).compare("&") == 0)
          {
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
          if (connect)
          {
            // ActiveConnection(GetModulePtrByName(modulename1), GetModulePtrByName(modulename2), node1, node2);
            SendGaitTable(GetModulePtrByName(modulename1), modulename1, modulename2, node1, node2, 1,condition,dependency);
          }else{
            // Deconnection(modulename1, modulename2);
            SendGaitTable(GetModulePtrByName(modulename1), modulename1, modulename2, 4, 4, 2,condition,dependency);
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
      if (time_interval == 0)
      {
        SendGaitTable(GetModulePtrByName(modelname), flags, joints_values, 3, condition, dependency);
      }else
      {
        SendGaitTable(GetModulePtrByName(modelname), flags, joints_values, 3, time_interval, condition, dependency);
      }
    }
    // User specify a joint angle
    if (!playmode)
    {
      // Disconnect Two Model
      if (msg->has_extrinfo())
      {
        string extrainfo = msg->extrinfo();
        if (extrainfo.substr(2,1).compare("+")==0 || extrainfo.substr(2,1).compare("-")==0)
        {
          bool connect = true;
          if (extrainfo.substr(2,1).compare("-")==0)
          {
            connect = false;
          }
          extrainfo = extrainfo.substr(4);
          string modulename1 = extrainfo.substr(1,extrainfo.find(" ")-1);
          extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
          int node1;
          int node2;
          string modulename2;
          if (extrainfo.substr(0,1).compare("&") == 0)
          {
            modulename2 = extrainfo.substr(1,extrainfo.find(" ")-1);
          }else{
            node1 = atoi(extrainfo.substr(1,extrainfo.find(" ")-1).c_str());
            extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
            modulename2 = extrainfo.substr(1,extrainfo.find(" ")-1);
            extrainfo = extrainfo.substr(extrainfo.find(" ")+1);
            node2 = atoi(extrainfo.substr(1,extrainfo.find(" ")-1).c_str());
          }

          if (connect)
          {
            ActiveConnection(GetModulePtrByName(modulename1), GetModulePtrByName(modulename2), node1, node2);
          }else{
            Deconnection(modulename1, modulename2);
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
      currentWorld->GetModel(modelname)->GetJoint("Front_wheel_hinge")->SetAngle(0,msg->jointangles(0));
      currentWorld->GetModel(modelname)->GetJoint("Left_wheel_hinge")->SetAngle(0,msg->jointangles(1));
      currentWorld->GetModel(modelname)->GetJoint("Right_wheel_hinge")->SetAngle(0,msg->jointangles(2));
      currentWorld->GetModel(modelname)->GetJoint("Center_hinge")->SetAngle(0,msg->jointangles(3));
      // SendGaitTableInstance(GetModulePtrByName(modelname), flags, joints_values,3);
      // LastGaitMode = false;
    }
  }
}

void ControlCenter::FeedBackMessageDecoding(CommandMessagePtr &msg)
{
  if (msg->messagetype()==0)
  {
    // this->FinishFlag = true;
    // cout<<"World: Message confirmed"<<endl;
    // if (msg->stringmessage().compare("finished")==0)
    // {
    //   cout<<"World: Execution finished"<<endl;
    // }
    // cout<<"World: string message is :"<<msg->stringmessage()<<endl;
    string moduleName = msg->stringmessage().substr(0,msg->stringmessage().find(":"));
    // cout<<"World: module name is : "<<moduleName<<endl;
    ModuleCommandsPtr command_for_current_module = GetModulePtrByName(moduleName)->ModuleCommandContainer;
    if (command_for_current_module)
    {
      if (command_for_current_module->ExecutionFlag)
      {
        command_for_current_module->ReceivedFlag = true;
        cout<<"World: "<<moduleName<<" set receive flag"<<endl;
      }
    }
    string secondField = msg->stringmessage().substr(msg->stringmessage().find(":")+1,string::npos);
    // cout<<"World: get the correct word : '"<<secondField<<"'"<<endl;
    if (secondField.compare("finished")==0)
    {
      cout<<"World: "<<moduleName<<" Execution finished"<<endl;
      if (command_for_current_module)
      {
        if (command_for_current_module->ExecutionFlag)
        {
          command_for_current_module->FinishedFlag = true;
          // command_for_current_module->CurrentPriority = msg->priority();
          command_for_current_module->ExecutionFlag = false;
        }
      }
      command_message::msgs::CommandMessage finish_confirm_message;
      finish_confirm_message.set_messagetype(0);
      GetModulePtrByName(moduleName)->ModulePublisher->Publish(finish_confirm_message);
    }
  }
  if (msg->messagetype()==5)
  {
    // Initalization functions
    moduleList.at(NeedToSetPtr)->SetModulePtr(currentWorld->GetModel(moduleList.at(NeedToSetPtr)->ModuleID));
    cout<<"World: Asign the pointer to module: "<<moduleList.at(NeedToSetPtr)->ModuleID<<endl;
    NeedToSetPtr += 1; // Need a function when delete a entity, this value needs to be decrease
    if (InitalJointValue.size()>0)
    {
      bool flags[4] = {true,true,true,true};
      double joint_angles[4] = {0};
      string joint_values_string = InitalJointValue.at(0);
      for (int i = 0; i < 4; ++i)
      {
        if (i<3)
        {
          joint_angles[i] = atof(joint_values_string.substr(0,joint_values_string.find(" ")).c_str());
          joint_values_string = joint_values_string.substr(joint_values_string.find(" ")+1);
        }else
        {
          joint_angles[i] = atof(joint_values_string.substr(0).c_str());
        }
      }
      currentWorld->GetModel(msg->stringmessage())->GetJoint("Front_wheel_hinge")->SetAngle(0,joint_angles[0]);
      currentWorld->GetModel(msg->stringmessage())->GetJoint("Left_wheel_hinge")->SetAngle(0,joint_angles[1]);
      currentWorld->GetModel(msg->stringmessage())->GetJoint("Right_wheel_hinge")->SetAngle(0,joint_angles[2]);
      currentWorld->GetModel(msg->stringmessage())->GetJoint("Center_hinge")->SetAngle(0,joint_angles[3]);
      currentWorld->GetModel(msg->stringmessage())->SetLinkWorldPose(InitialPosition.at(0),currentWorld->GetModel(msg->stringmessage())->GetLink("CircuitHolder"));

      SendGaitTableInstance(GetModulePtrByName(msg->stringmessage()), flags, joint_angles,3);
      if (InitalJointValue.size()==1)
      {
        // Confiuration connection initialized
        BuildConnectionFromXML();
        cout<<"World: Build the connection"<<endl;
        // Record the initialposition
        // GlobalInitialPosition = moduleList.at(0)->ModuleObject->GetLink("CircuitHolder")->GetWorldPose();
        RecordCurrentPose(GlobalInitialJoints);
        // This line is a test, please delete in the future
        // readFileAndGenerateCommands("Commands");
        // cout<<"World: Command been sent"<<endl;
        // Need one right after readcommands function
        // currentCommandGroupInitialization();
      }
      InitalJointValue.erase(InitalJointValue.begin());
      InitialPosition.erase(InitialPosition.begin());
    }
  }
}

void ControlCenter::AutomaticMagneticConnectionManagement(CollisionMessagePtr &msg)
{
  string ModelOfCollision1 = msg->collision1().substr(0,msg->collision1().find("::"));
  string ModelOfCollision2 = msg->collision2().substr(0,msg->collision2().find("::"));
  string LinkOfCollision1 = msg->collision1().substr(msg->collision1().find("::")+2,msg->collision1().rfind("::")-msg->collision1().find("::")-2);
  string LinkOfCollision2 = msg->collision2().substr(msg->collision2().find("::")+2,msg->collision2().rfind("::")-msg->collision2().find("::")-2);
  SmoresModulePtr Model1Ptr = GetModulePtrByName(ModelOfCollision1);
  SmoresModulePtr Model2Ptr = GetModulePtrByName(ModelOfCollision2);
  int NodeOfModel1 = GetNodeIDByName(LinkOfCollision1);
  int NodeOfModel2 = GetNodeIDByName(LinkOfCollision2);

  math::Pose ContactLinkPos = msgs::Convert(msg->positioncol1());
  //---------------------- Find the pending connection request ------------------------
  int FoundPendingOne = 0;
  for (unsigned int i = 0; i < PendingRequest.size(); ++i)
  {
    if (PendingRequest.at(i).SameCollision(ModelOfCollision1,ModelOfCollision2,LinkOfCollision1,LinkOfCollision2))
    {
      PendingRequest.erase(PendingRequest.begin()+i);
      FoundPendingOne = 1;
      break;
    }
  }
  if(NodeOfModel1<4 && NodeOfModel2<4)
  {
    if (FoundPendingOne==0)
    {
      //---------------------- Add new pending connection request ------------------------
      if ((!AlreadyConnected(Model1Ptr,NodeOfModel1)) && (!AlreadyConnected(Model2Ptr,NodeOfModel2)) && (!AlreadyConnected(Model1Ptr,Model2Ptr)))
      {
        // This part is used to check the distance between robots
        math::Vector3 CenterModel1 = currentWorld->GetModel(ModelOfCollision1)->GetLink("CircuitHolder")->GetWorldPose().pos;
        math::Vector3 CenterModel2 = currentWorld->GetModel(ModelOfCollision2)->GetLink("CircuitHolder")->GetWorldPose().pos;
        cout<<"World: models distance"<<(CenterModel1-CenterModel2).GetLength()<<endl;
        if((CenterModel1-CenterModel2).GetLength()<VALIDCONNECTIONDISUPPER && (CenterModel1-CenterModel2).GetLength()>VALIDCONNECTIONDISLOWER)
        {
          cout<<"World: Distance between centers: "<<(CenterModel1-CenterModel2).GetLength()<<endl;
          CollisionInformation NewConnectionRequest(ModelOfCollision1, ModelOfCollision2,LinkOfCollision1,LinkOfCollision2);
          PendingRequest.push_back(NewConnectionRequest);
          cout<<"World: An pending entry has been established: '"<< ModelOfCollision1+":"+LinkOfCollision1+"::"+ ModelOfCollision2 + ":"+LinkOfCollision2<<"'"<<endl;
        }
      }
    }else{
      // The lighter configuration connects to heavier configuration
      if (CountModules(Model1Ptr)<CountModules(Model2Ptr))
      {
        SmoresModulePtr tmp_module_ptr = Model1Ptr;
        Model1Ptr = Model2Ptr;
        Model2Ptr = tmp_module_ptr;
        int tmp_node = NodeOfModel1;
        NodeOfModel1 = NodeOfModel2;
        NodeOfModel2 = tmp_node;
      }
      //-------- Do the real connection (including generate dynamic joint) ----------------
      ActiveConnection(Model1Ptr,Model2Ptr,NodeOfModel1,NodeOfModel2);
    }
  }
}

void ControlCenter::ConnectAndDynamicJointGeneration(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, SmoresEdgePtr an_edge)
{
  math::Pose ContactLinkPos = module_1->GetLinkPtr(node1_ID)->GetWorldPose();
  math::Pose PosOfTheOtherModel = module_2->GetLinkPtr(node2_ID)->GetWorldPose();
  cout<<"World: Post position of the module 1: ("<<ContactLinkPos.pos.x<<","<<ContactLinkPos.pos.y<<","<<ContactLinkPos.pos.z<<")"<<endl;
  cout<<"World: Post position of the module 2: ("<<PosOfTheOtherModel.pos.x<<","<<PosOfTheOtherModel.pos.y<<","<<PosOfTheOtherModel.pos.z<<")"<<endl;
  physics::LinkPtr Link1, Link2;
  math::Vector3 axis;
  math::Vector3 ZDirectionOffset(0,0,0.000);  //0.008
  // math::Vector3 newCenterPoint = 0.5*(ContactLinkPos.pos + PendingRequestPos.at(i).pos)+ZDirectionOffset;
  math::Vector3 newPositionOfLink1;
  math::Vector3 newPositionOfLink2;
  math::Quaternion newDirectionofLink1;
  math::Vector3 newZAxis;
  double AngleBetweenZAxes;
  math::Quaternion FirstRotationOfLink2;
  math::Quaternion SecondRotationOfLink2;
  math::Quaternion newDirectionofLink2;
  if (node1_ID == 0)
  {
    newPositionOfLink1 = ContactLinkPos.pos;
    newPositionOfLink2 = ContactLinkPos.pos - (0.1+an_edge->Distance)*ContactLinkPos.rot.GetYAxis();
    newDirectionofLink1 = ContactLinkPos.rot;
    newZAxis = PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetXAxis())*newDirectionofLink1.GetXAxis();
    newZAxis = newZAxis.Normalize();
    double CosineValue = newZAxis.Dot(newDirectionofLink1.GetZAxis())>0?min(1.0,newZAxis.Dot(newDirectionofLink1.GetZAxis())):max(-1.0,newZAxis.Dot(newDirectionofLink1.GetZAxis()));
    AngleBetweenZAxes = acos(CosineValue); // + an_edge->Angle;
    FirstRotationOfLink2.SetFromEuler(0,0,PI);
    double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
    if (DirectionReference>0)
    {
      SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
    }else{
      SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
    }
    if (node2_ID==3)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,0);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
      }else{
        SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
      }
    }
    if (node2_ID==1)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
      }else{
        SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
      }
    }
    if (node2_ID==2)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
      }else{
        SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
      }
    }
    newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
    axis.Set(0,1,0);
  }
  if (node1_ID == 1)
  {
    newPositionOfLink1 = ContactLinkPos.pos;
    newPositionOfLink2 = ContactLinkPos.pos + (0.1+an_edge->Distance)*ContactLinkPos.rot.GetXAxis();
    newDirectionofLink1 = ContactLinkPos.rot;
    newZAxis = PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetYAxis())*newDirectionofLink1.GetYAxis();
    newZAxis = newZAxis.Normalize();
    double CosineValue = newZAxis.Dot(newDirectionofLink1.GetZAxis())>0?min(1.0,newZAxis.Dot(newDirectionofLink1.GetZAxis())):max(-1.0,newZAxis.Dot(newDirectionofLink1.GetZAxis()));
    AngleBetweenZAxes = acos(CosineValue); // + an_edge->Angle;
    cout<<"World: AngleBetweenZAxes is "<<AngleBetweenZAxes<<endl;
    FirstRotationOfLink2.SetFromEuler(0,0,PI);
    double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
    if (DirectionReference>0)
    {
      SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
    }else{
      SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
    }
    if (node2_ID==0)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
      }else{
        SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
      }
    }
    if (node2_ID==3)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
      }else{
        SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
      }
    }
    if (node2_ID==2)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,-PI);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
      }else{
        SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
      }
    }
    newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
    axis.Set(1,0,0);
  }
  if (node1_ID == 2)
  {
    newPositionOfLink1 = ContactLinkPos.pos;
    newPositionOfLink2 = ContactLinkPos.pos + (0.1+an_edge->Distance)*ContactLinkPos.rot.GetXAxis();
    newDirectionofLink1 = ContactLinkPos.rot;
    newZAxis = PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetYAxis())*newDirectionofLink1.GetYAxis();
    newZAxis = newZAxis.Normalize();
    double CosineValue = newZAxis.Dot(newDirectionofLink1.GetZAxis())>0?min(1.0,newZAxis.Dot(newDirectionofLink1.GetZAxis())):max(-1.0,newZAxis.Dot(newDirectionofLink1.GetZAxis()));
    AngleBetweenZAxes = acos(CosineValue); // + an_edge->Angle;
    FirstRotationOfLink2.SetFromEuler(0,0,PI);
    double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
    if (DirectionReference>0)
    {
      SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
    }else{
      SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
    }
    if (node2_ID==0)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
      }else{
        SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
      }
    }
    if (node2_ID==3)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,PI/2);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
      }else{
        SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
      }
    }
    if (node2_ID==1)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,-PI);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0 ,0);
      }else{
        SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0 ,0);
      }
    }
    newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
    axis.Set(1,0,0);
  }
  if (node1_ID == 3)
  {
    newPositionOfLink1 = ContactLinkPos.pos;
    newPositionOfLink2 = ContactLinkPos.pos + (0.1+an_edge->Distance)*ContactLinkPos.rot.GetYAxis();
    newDirectionofLink1 = ContactLinkPos.rot;
    newZAxis = PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetZAxis())*newDirectionofLink1.GetZAxis() + PosOfTheOtherModel.rot.GetZAxis().Dot(newDirectionofLink1.GetXAxis())*newDirectionofLink1.GetXAxis();
    newZAxis = newZAxis.Normalize();
    double CosineValue = newZAxis.Dot(newDirectionofLink1.GetZAxis())>0?min(1.0,newZAxis.Dot(newDirectionofLink1.GetZAxis())):max(-1.0,newZAxis.Dot(newDirectionofLink1.GetZAxis()));
    AngleBetweenZAxes = acos(CosineValue); // + an_edge->Angle;
    FirstRotationOfLink2.SetFromEuler(0,0,PI);
    double DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
    if (DirectionReference>0)
    {
      SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
    }else{
      SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
    }
    if (node2_ID==0)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,0);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetYAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(0, AngleBetweenZAxes ,0);
      }else{
        SecondRotationOfLink2.SetFromEuler(0, -AngleBetweenZAxes ,0);
      }
    }
    if (node2_ID==1)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
      }else{
        SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
      }
    }
    if (node2_ID==2)
    {
      FirstRotationOfLink2.SetFromEuler(0,0,-PI/2);
      DirectionReference = newDirectionofLink1.GetZAxis().Cross(newZAxis).Dot(PosOfTheOtherModel.rot.GetXAxis());
      if (DirectionReference>0)
      {
        SecondRotationOfLink2.SetFromEuler(AngleBetweenZAxes, 0, 0);
      }else{
        SecondRotationOfLink2.SetFromEuler(-AngleBetweenZAxes, 0, 0);
      }
    }
    newDirectionofLink2 = newDirectionofLink1*FirstRotationOfLink2*SecondRotationOfLink2;
    axis.Set(0,1,0);
  }
  //+++++++++++++++++ This part of the code set the correct position of the models +++++++++++++++++
  Link1 = module_1->GetLinkPtr(node1_ID);
  Link2 = module_2->GetLinkPtr(node2_ID);

  module_1->ModuleObject->SetLinkWorldPose(math::Pose(newPositionOfLink1,newDirectionofLink1),Link1);
  module_2->ModuleObject->SetLinkWorldPose(math::Pose(newPositionOfLink2,newDirectionofLink2),Link2);

  //++++++++++++++++++ This part of the code generate the dynamic joint ++++++++++++++++++++++++++++
  physics::JointPtr DynamicJoint;
  DynamicJoint = currentWorld->GetPhysicsEngine()->CreateJoint("revolute",  module_1->ModuleObject);
  DynamicJoint->Attach(Link1, Link2);
  DynamicJoint->Load(Link1, Link2, math::Pose(math::Vector3(0,-0.00,0),math::Quaternion()));
  DynamicJoint->SetAxis(0, axis);
  module_1->ModuleObject->GetJointController()->AddJoint(DynamicJoint);
  DynamicJoint->SetAngle(0,math::Angle(0));
  DynamicJoint->SetHighStop(0,math::Angle(0.01));
  DynamicJoint->SetLowStop(0,math::Angle(-0.01));
  DynamicConnections.push_back(DynamicJoint);
  // This is necessary for easy access of the dynamic joint
  // an_edge->DynamicJointPtr = DynamicJoint;
  an_edge->DynamicJointPtr = DynamicConnections.back();
}

void ControlCenter::DynamicJointDestroy(SmoresEdgePtr aEdge)
{
  aEdge->DynamicJointPtr->Detach();
  // cout<<"World: The crush is not because of dynamic joint detach"<<endl;
  for (unsigned int i = 0; i < DynamicConnections.size(); ++i)
  {
    if (aEdge->DynamicJointPtr==DynamicConnections.at(i))
    {
      // cout<<"World: The crush is before reset"<<endl;
      DynamicConnections.at(i).reset();
      // cout<<"World: The crush is after reset"<<endl;
      DynamicConnections.erase(DynamicConnections.begin()+i);
      break;
    }
  }
}

void ControlCenter::InsertModel(string name, math::Pose position)
{
  if (!currentWorld->GetModel(name))
  {
    sdf::SDFPtr modelSDF;
    modelSDF.reset(new sdf::SDF);  
    // sdf::initFile("gazebo.sdf", modelSDF);
    sdf::init(modelSDF);
    sdf::readFile(MODULEPATH, modelSDF);
    sdf::ElementPtr modelElem = modelSDF->root->GetElement("model");
    // std::string modelName = modelElem->GetValueString("name");
    math::Pose CalibrateShift(math::Vector3(0, 0, -0.05), math::Quaternion(0, 0, 0));
    modelElem->GetAttribute("name")->Set(name);
    modelElem->GetElement("pose")->Set(CalibrateShift);

    currentWorld->InsertModelSDF(*modelSDF);
    InitialPosition.push_back(position);
    // cout<<"World: Initial Joint Angle Set "<<endl;
  }
  // else{
  //   cout<<"World: Insertion failed: module name exists"
  // }
}  

void ControlCenter::InsertModel(string name, math::Pose position, string joint_angles)
{
  if (!currentWorld->GetModel(name))
  {
    sdf::SDFPtr modelSDF;
    modelSDF.reset(new sdf::SDF);  
    // sdf::initFile("gazebo.sdf", modelSDF);
    sdf::init(modelSDF);
    sdf::readFile(MODULEPATH, modelSDF);
    sdf::ElementPtr modelElem = modelSDF->root->GetElement("model");
    // std::string modelName = modelElem->GetValueString("name");
    math::Pose CalibrateShift(math::Vector3(0, 0, -0.05), math::Quaternion(0, 0, 0));
    modelElem->GetAttribute("name")->Set(name);
    modelElem->GetElement("pose")->Set(CalibrateShift);

    currentWorld->InsertModelSDF(*modelSDF);
    InitalJointValue.push_back(joint_angles);
    InitialPosition.push_back(position);
  }
  // else{
  //   cout<<"World: Insertion failed: module name exists"
  // }
} 

void ControlCenter::PassiveConnection(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, double node_angle, double node_distance)
{
  if (!AlreadyConnected(module_1, module_2, node1_ID, node2_ID))
  {
    SmoresEdgePtr new_connection(new SmoresEdge(module_1->GetNode(node1_ID),module_2->GetNode(node2_ID),node_distance,node_angle,module_1->GetNodeAxis(node1_ID),module_2->GetNodeAxis(node2_ID)));
    ConnectionEdges.push_back(new_connection);
    module_1->GetNode(node1_ID)->ConnectOnEdge(new_connection);
    module_2->GetNode(node2_ID)->ConnectOnEdge(new_connection);
    // Adding the dynamic joint after adding the new edge
    ConnectAndDynamicJointGeneration(module_1, module_2, node1_ID, node2_ID, new_connection);
  }
}

void ControlCenter::ActiveConnection(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID, double node_angle, double node_distance)
{
  if (!AlreadyConnected(module_1, module_2, node1_ID, node2_ID))
  {
    SmoresEdgePtr new_connection(new SmoresEdge(module_1->GetNode(node1_ID),module_2->GetNode(node2_ID),node_distance,node_angle,module_1->GetNodeAxis(node1_ID),module_2->GetNodeAxis(node2_ID)));
    ConnectionEdges.push_back(new_connection);
    module_1->GetNode(node1_ID)->ConnectOnEdge(new_connection);
    module_2->GetNode(node2_ID)->ConnectOnEdge(new_connection);
    // Adding the dynamic joint after adding the new edge
    ConnectAndDynamicJointGeneration(module_1, module_2, node1_ID, node2_ID, new_connection);
  }
}

void ControlCenter::Deconnection(SmoresEdgePtr aEdge)  // This pointer must point to an element in the vector
{
  DynamicJointDestroy(aEdge);
  aEdge->model_1->Edge.reset();
  aEdge->model_2->Edge.reset();
  cout<<"World: The crush is after reset of edge in node"<<endl;
  for (unsigned int i = 0; i < ConnectionEdges.size(); ++i)
  {
    if (ConnectionEdges.at(i)==aEdge)
    {
      ConnectionEdges.at(i).reset();
      ConnectionEdges.erase(ConnectionEdges.begin()+i);
      break;
    }
  }
}

void ControlCenter::Deconnection(SmoresModulePtr aModule, int node_ID)
{
  SmoresEdgePtr aEdge = aModule->GetNode(node_ID)->GetEdge();
  this->Deconnection(aEdge);

}

void ControlCenter::Deconnection(string moduleName, int node_ID)
{

}

void ControlCenter::Deconnection(string moduleName1, string moduleName2)
{
  for (unsigned int i = 0; i < ConnectionEdges.size(); ++i)
  {
    if ((ConnectionEdges.at(i)->model_1->Parent->ModuleID.compare(moduleName1)==0 && ConnectionEdges.at(i)->model_2->Parent->ModuleID.compare(moduleName2)==0) || (ConnectionEdges.at(i)->model_1->Parent->ModuleID.compare(moduleName2)==0 && ConnectionEdges.at(i)->model_2->Parent->ModuleID.compare(moduleName1)==0))
    {
      DynamicJointDestroy(ConnectionEdges.at(i));
      ConnectionEdges.at(i)->model_1->Edge.reset();
      ConnectionEdges.at(i)->model_2->Edge.reset();
      ConnectionEdges.at(i).reset();
      ConnectionEdges.erase(ConnectionEdges.begin()+i);
      break;
    }
  }
}

void ControlCenter::CommandManager(void)
{
  // CurrentMinimalGroup = MAX_COMMANDGROUP_LENGTH + 1;
  for (unsigned int i = 0; i < ModuleCommandContainer.size(); ++i)
  {
    if (ModuleCommandContainer.at(i)->CommandSquence.size()>0)
    {
      if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeBased)
      {
        if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeInterval == 0)
        {
          ModuleCommandContainer.at(i)->FinishedFlag = true;
        }else{
          ModuleCommandContainer.at(i)->FinishedFlag = false;
        }
      }
      // -------------------------------------------------------------------------------
      if (!ModuleCommandContainer.at(i)->FinishedFlag)
      {
        if (!ModuleCommandContainer.at(i)->ReceivedFlag)
        {
          if (ModuleCommandContainer.at(i)->CommandSquence.at(0).ConditionOnOtherCommand)
          {
            if (CheckCondition(ModuleCommandContainer.at(i)->CommandSquence.at(0).Dependency))
            {
              // cout<<"World: Still can get in here"<<endl;
              // if (ModuleCommandContainer.at(i)->CommandSquence.at(0).ConditionID.compare("g"))
              // {
              //   cout<<"World: Enter the condition: g"<<endl;
              // }
              if (ModuleCommandContainer.at(i)->CommandSquence.at(0).SpecialCommandFlag)
              {
                cout<<"World: Here is the special command"<<endl;
                if (ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.CommandType == 1)
                {
                  ActiveConnection(ModuleCommandContainer.at(i)->WhichModule, GetModulePtrByName(ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Module2), ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Node1, ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Node2);
                  if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeBased)
                  {
                    ModuleCommandContainer.at(i)->ReceivedFlag = true;
                  }else
                  {
                    ModuleCommandContainer.at(i)->FinishedFlag = true;
                    ModuleCommandContainer.at(i)->ReceivedFlag = true;
                  }
                }
                if (ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.CommandType == 2)
                {
                  cout<<"World: Disconnection command has been send"<<endl;
                  Deconnection(ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Module1, ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Module2);
                  if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeBased)
                  {
                    ModuleCommandContainer.at(i)->ReceivedFlag = true;
                  }else
                  {
                    ModuleCommandContainer.at(i)->FinishedFlag = true;
                    ModuleCommandContainer.at(i)->ReceivedFlag = true;
                  }
                }
              }else
              {
                ModuleCommandContainer.at(i)->WhichModule->ModulePublisher->Publish(*(ModuleCommandContainer.at(i)->CommandSquence.at(0).ActualCommandMessage));
              }
              // cout<<"World: Message to '"<<ModuleCommandContainer.at(i)->WhichModule->ModuleID<<"' set joint angle 3 to : "<<ModuleCommandContainer.at(i)->CommandSquence.at(0)->jointgaittable(3)<<endl;
              cout<<"World: Model: "<<ModuleCommandContainer.at(i)->WhichModule->ModuleID<<": command sent"<<endl;
              cout<<"World: Size of the message is "<<ModuleCommandContainer.at(i)->CommandSquence.size()<<endl;
              ModuleCommandContainer.at(i)->ExecutionFlag = true;
            }
            // else
            // {
            //   // ModuleCommandContainer.at(i)->FinishedFlag = false;
            //   // ModuleCommandContainer.at(i)->ReceivedFlag = false;
            //   cout<<"World: is waiting for condition: "<< ModuleCommandContainer.at(i)->CommandSquence.at(0).Dependency<<endl;
            // }
          }else
          {
            // cout<<"World: Still can get in here"<<endl;
            if (ModuleCommandContainer.at(i)->CommandSquence.at(0).SpecialCommandFlag)
            {
              if (ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.CommandType == 1)
              {
                ActiveConnection(ModuleCommandContainer.at(i)->WhichModule, GetModulePtrByName(ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Module2), ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Node1, ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Node2);
                if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeBased)
                {
                  ModuleCommandContainer.at(i)->ReceivedFlag = true;
                }else
                {
                  ModuleCommandContainer.at(i)->FinishedFlag = true;
                  ModuleCommandContainer.at(i)->ReceivedFlag = true;
                }
              }
              if (ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.CommandType == 2)
              {
                Deconnection(ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Module1, ModuleCommandContainer.at(i)->CommandSquence.at(0).Command.Module2);
                if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeBased)
                {
                  ModuleCommandContainer.at(i)->ReceivedFlag = true;
                }else
                {
                  ModuleCommandContainer.at(i)->FinishedFlag = true;
                  ModuleCommandContainer.at(i)->ReceivedFlag = true;
                }
              }
            }else
            {
              ModuleCommandContainer.at(i)->WhichModule->ModulePublisher->Publish(*(ModuleCommandContainer.at(i)->CommandSquence.at(0).ActualCommandMessage));
            }
            // cout<<"World: Message to '"<<ModuleCommandContainer.at(i)->WhichModule->ModuleID<<"' set joint angle 3 to : "<<ModuleCommandContainer.at(i)->CommandSquence.at(0)->jointgaittable(3)<<endl;
            cout<<"World: Model: "<<ModuleCommandContainer.at(i)->WhichModule->ModuleID<<": command sent"<<endl;
            cout<<"World: Size of the message is "<<ModuleCommandContainer.at(i)->CommandSquence.size()<<endl;
            ModuleCommandContainer.at(i)->ExecutionFlag = true;
          }
        }else
        {
          // ----------------------- Time Count Down -------------------------
          if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeBased)
          {
            ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeInterval -= 1;
          }
        }
      }else
      {
        if (ModuleCommandContainer.at(i)->CommandSquence.at(0).ConditionCommand)
        {
          FinishOneConditionCommand(ModuleCommandContainer.at(i)->CommandSquence.at(0).ConditionID);
        }
        // This part will delete the correct priority level command
        ModuleCommandContainer.at(i)->CommandSquence.erase(ModuleCommandContainer.at(i)->CommandSquence.begin());
        cout<<"World: second place:"<<ModuleCommandContainer.at(i)->WhichModule->ModuleID<<" Size of the message is "<<ModuleCommandContainer.at(i)->CommandSquence.size()<<endl;

        ModuleCommandContainer.at(i)->ReceivedFlag = false;
        ModuleCommandContainer.at(i)->FinishedFlag = false;
        // command_message::msgs::CommandMessage finish_confirm_message;
        // finish_confirm_message.set_messagetype(0);
        // ModuleCommandContainer.at(i)->WhichModule->ModulePublisher->Publish(finish_confirm_message);
        if (ModuleCommandContainer.at(i)->CommandSquence.size() == 0)
        {
          cout<<"World: Erase the command sequence of "<<ModuleCommandContainer.at(i)->WhichModule->ModuleID<<endl;
          ModuleCommandContainer.at(i)->WhichModule->ModuleCommandContainer.reset();
          ModuleCommandContainer.erase(ModuleCommandContainer.begin()+i);
        }
      }
    }
  }
}
// group: 0: same as the last command; -n: n step before the current group, if no smaller group, then set to the current group; +n: n steps after the current group. Total length is 5000
void ControlCenter::SendGaitTable(SmoresModulePtr module, bool flag[4], double gait_value[4], int msg_type, unsigned int time_stamp, string condition_str, string dependency_str) // unit of time_stamp is millisecond
{
  CommandPtr ConnectionMessage(new command_message::msgs::CommandMessage());
  // command_message::msgs::CommandMessage ConnectionMessage;
  ConnectionMessage->set_messagetype(msg_type);
  ConnectionMessage->set_priority(0);
  for (int i = 0; i < 4; ++i)
  {
    ConnectionMessage->add_jointgaittablestatus(flag[i]);
    ConnectionMessage->add_jointgaittable(gait_value[i]);
  }
  // Here is a patch for instantly executed command
  if (time_stamp == 0)
  {
    time_stamp = 1;
  }
  CommandPro aNewMessage(ConnectionMessage,time_stamp);
  if (condition_str.size()>0)
  {
    aNewMessage.SetCondition(condition_str);
    AddCondition(condition_str);
  }
  if (dependency_str.size()>0)
  {
    aNewMessage.SetDependency(dependency_str);
  }

  if (!module->ModuleCommandContainer)
  {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));
    
    new_command_message->CommandSquence.push_back(aNewMessage);
    ModuleCommandContainer.push_back(new_command_message);
    module->ModuleCommandContainer = new_command_message;
    cout<<"World: "<<module->ModuleID<<" : command length: "<<module->ModuleCommandContainer->CommandSquence.size()<<endl;
  }else
  {
    module->ModuleCommandContainer->CommandSquence.push_back(aNewMessage);
    cout<<"World: "<<module->ModuleID<<" : command length: "<<module->ModuleCommandContainer->CommandSquence.size()<<endl;
  }
}

void ControlCenter::SendGaitTable(SmoresModulePtr module, bool flag[4], double gait_value[4], int msg_type, string condition_str, string dependency_str) // unit of time_stamp is millisecond
{
  CommandPtr ConnectionMessage(new command_message::msgs::CommandMessage());
  // command_message::msgs::CommandMessage ConnectionMessage;
  ConnectionMessage->set_messagetype(msg_type);
  ConnectionMessage->set_priority(0);
  for (int i = 0; i < 4; ++i)
  {
    ConnectionMessage->add_jointgaittablestatus(flag[i]);
    ConnectionMessage->add_jointgaittable(gait_value[i]);
  }
  CommandPro aNewMessage(ConnectionMessage);
  if (condition_str.size()>0)
  {
    aNewMessage.SetCondition(condition_str);
    AddCondition(condition_str);
  }
  if (dependency_str.size()>0)
  {
    aNewMessage.SetDependency(dependency_str);
  }

  if (!module->ModuleCommandContainer)
  {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));
    
    new_command_message->CommandSquence.push_back(aNewMessage);
    ModuleCommandContainer.push_back(new_command_message);
    module->ModuleCommandContainer = new_command_message;
    cout<<"World: "<<module->ModuleID<<" : command length: "<<module->ModuleCommandContainer->CommandSquence.size()<<endl;
  }else
  {
    module->ModuleCommandContainer->CommandSquence.push_back(aNewMessage);
    cout<<"World: "<<module->ModuleID<<" : command length: "<<module->ModuleCommandContainer->CommandSquence.size()<<endl;
  }
}

void ControlCenter::SendGaitTable(SmoresModulePtr module, int joint_ID, double gait_value, int msg_type, unsigned int time_stamp, string condition_str, string dependency_str)
{
  bool flag[4] = {false,false,false,false};
  double gait_values[4] = {0,0,0,0};
  flag[joint_ID] = true;
  gait_values[joint_ID] = gait_value;
  SendGaitTable(module, flag, gait_values, msg_type, time_stamp, condition_str, dependency_str);
}

void ControlCenter::SendGaitTable(SmoresModulePtr module, int joint_ID, double gait_value, int msg_type, string condition_str, string dependency_str)
{
  bool flag[4] = {false,false,false,false};
  double gait_values[4] = {0,0,0,0};
  flag[joint_ID] = true;
  gait_values[joint_ID] = gait_value;
  SendGaitTable(module, flag, gait_values, msg_type, condition_str, dependency_str);
}

// This command used to invoke disconnect command
void ControlCenter::SendGaitTable(SmoresModulePtr module, string module1, string module2, int node1, int node2, int commandtype, unsigned int time_stamp, string condition_str, string dependency_str)
{
  CommandPro aNewMessage;
  // Here is a patch for instantly executed command
  if (time_stamp == 0)
  {
    time_stamp = 1;
  }
  aNewMessage.SetTimer(time_stamp);
  SpecialCommand newSpecialCommand(commandtype, module1, module2, node1, node2);
  aNewMessage.SetSpecialCommand(newSpecialCommand);
  cout<<"World: Special Command: condition: "<<condition_str<<"; dependency: "<<dependency_str<<endl;
  if (condition_str.size()>0)
  {
    aNewMessage.SetCondition(condition_str);
    AddCondition(condition_str);
  }
  if (dependency_str.size()>0)
  {
    aNewMessage.SetDependency(dependency_str);
  }

  if (!module->ModuleCommandContainer)
  {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));
    
    new_command_message->CommandSquence.push_back(aNewMessage);
    ModuleCommandContainer.push_back(new_command_message);
    module->ModuleCommandContainer = new_command_message;
  }else
  {
    module->ModuleCommandContainer->CommandSquence.push_back(aNewMessage);
  }
}

void ControlCenter::SendGaitTable(SmoresModulePtr module, string module1, string module2, int node1, int node2, int commandtype, string condition_str, string dependency_str)
{
  CommandPro aNewMessage;
  SpecialCommand newSpecialCommand(commandtype, module1, module2, node1, node2);
  aNewMessage.SetSpecialCommand(newSpecialCommand);
  cout<<"World: Special Command: condition: "<<condition_str<<"; dependency: "<<dependency_str<<endl;
  if (condition_str.size()>0)
  {
    aNewMessage.SetCondition(condition_str);
    AddCondition(condition_str);
  }
  if (dependency_str.size()>0)
  {
    aNewMessage.SetDependency(dependency_str);
  }

  if (!module->ModuleCommandContainer)
  {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));
    
    new_command_message->CommandSquence.push_back(aNewMessage);
    ModuleCommandContainer.push_back(new_command_message);
    module->ModuleCommandContainer = new_command_message;
  }else
  {
    module->ModuleCommandContainer->CommandSquence.push_back(aNewMessage);
  }
}

void ControlCenter::SendGaitTableInstance(SmoresModulePtr module, bool flag[4], double gait_value[4], int msg_type)
{
  CommandPtr ConnectionMessage(new command_message::msgs::CommandMessage());
  // command_message::msgs::CommandMessage ConnectionMessage;
  ConnectionMessage->set_messagetype(msg_type);
  ConnectionMessage->set_priority(-1);
  for (int i = 0; i < 4; ++i)
  {
    ConnectionMessage->add_jointgaittablestatus(flag[i]);
    ConnectionMessage->add_jointgaittable(gait_value[i]);
  }

  module->ModulePublisher->Publish(*ConnectionMessage);
}

void ControlCenter::SendPositionInstance(SmoresModulePtr module, double x, double y, double orientation_angle)
{
  CommandPtr ConnectionMessage(new command_message::msgs::CommandMessage());
  // command_message::msgs::CommandMessage ConnectionMessage;
  ConnectionMessage->set_messagetype(2);
  ConnectionMessage->set_priority(-1);
  ConnectionMessage->mutable_positionneedtobe()->mutable_position()->set_x(x);
  ConnectionMessage->mutable_positionneedtobe()->mutable_position()->set_y(y);
  ConnectionMessage->mutable_positionneedtobe()->mutable_position()->set_z(orientation_angle);
  ConnectionMessage->mutable_positionneedtobe()->mutable_orientation()->set_x(0);
  ConnectionMessage->mutable_positionneedtobe()->mutable_orientation()->set_y(0);
  ConnectionMessage->mutable_positionneedtobe()->mutable_orientation()->set_z(0);
  ConnectionMessage->mutable_positionneedtobe()->mutable_orientation()->set_w(0);

  module->ModulePublisher->Publish(*ConnectionMessage);
}

void ControlCenter::EraseComaands(SmoresModulePtr module)  // Need to be tested
{
  if (module->ModuleCommandContainer)
  {
    EraseCommandPtrByModule(module);
    module->ModuleCommandContainer.reset();
  }
}

int ControlCenter::GetNodeIDByName(string node_name)
{
  if (node_name.rfind("FrontWheel")!=string::npos)
  {
    return 0;
  }
  if (node_name.rfind("LeftWheel")!=string::npos)
  {
    return 1;
  }
  if (node_name.rfind("RightWheel")!=string::npos)
  {
    return 2;
  }
  if (node_name.rfind("UHolderBody")!=string::npos)
  {
    return 3;
  }
  return 4; // When return 4, then there is no match found
}

SmoresModulePtr ControlCenter::GetModulePtrByName(string module_name)
{
  SmoresModulePtr ExistModule;
  for (unsigned int i = 0; i < moduleList.size(); ++i)
  {
    if (module_name.compare(moduleList.at(i)->ModuleID)==0)
    {
      ExistModule = moduleList.at(i);
      break;
    }
  }
  return ExistModule;
}

void ControlCenter::EraseCommandPtrByModule(SmoresModulePtr module_ptr)
{
  for (unsigned int i = 0; i < ModuleCommandContainer.size(); ++i)
  {
    if (module_ptr == ModuleCommandContainer.at(i)->WhichModule)
    {
      ModuleCommandContainer.erase(ModuleCommandContainer.begin()+i);
      break;
    }
  }
} 

int ControlCenter::GetModuleIDXByName(string module_name)
{
  int ExistModule = -1;
  for (unsigned int i = 0; i < moduleList.size(); ++i)
  {
    if (module_name.compare(moduleList.at(i)->ModuleID)==0)
    {
      ExistModule = i;
      break;
    }
  }
  return ExistModule;
}

bool ControlCenter::AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2, int node1_ID, int node2_ID)
{
  bool HavingAConnection;
  if((bool)module_1->GetNode(node1_ID)->Edge && (bool)module_2->GetNode(node2_ID)->Edge)
  {
    HavingAConnection = true;
  }else
  {
    HavingAConnection = false;
  }
  return HavingAConnection;
}

bool ControlCenter::AlreadyConnected(SmoresModulePtr module_1, SmoresModulePtr module_2)
{
  bool HavingAConnection = false;
  for (int j = 0; j < 4; ++j)
  {
    if (module_1->GetNode(j)->Edge)
    {
      SmoresModulePtr ConnectedModule = module_1->GetNode(j)->Edge->FindMatchingNode(module_1->GetNode(j))->Parent;
      if (ConnectedModule == module_2)
      {
        HavingAConnection = true;
        break;
      }
    }
  }
  return HavingAConnection;
}

bool ControlCenter::AlreadyConnected(SmoresModulePtr module, int node_ID)
{
  bool HavingAConnection = false;
  if (module->GetNode(node_ID)->Edge)
  {
    HavingAConnection = true;
  }
  return HavingAConnection;
}

unsigned int ControlCenter::CountModules(SmoresModulePtr module)
{
  vector<SmoresModulePtr> vector1;
  vector<SmoresModulePtr> vector2;
  vector<SmoresModulePtr> vector3;
  vector1.push_back(module);
  vector3.push_back(module);
  unsigned int module_count = 0;
  while(1)
  {
    if (vector1.size()>0)
    {
      module_count += vector1.size();
      for (unsigned int i = 0; i < vector1.size(); ++i)
      {
        for (int j = 0; j < 4; ++j)
        {
          if(vector1.at(i)->GetNode(j)->Edge)
          {
            SmoresModulePtr ConnectedModule = vector1.at(i)->GetNode(j)->Edge->FindMatchingNode(vector1.at(i)->GetNode(j))->Parent;
            vector2.push_back(ConnectedModule);
          }
        }
      }
      vector1.clear();
      for (unsigned int i = 0; i < vector2.size(); ++i)
      {
        bool have_it = true;
        for (unsigned int j = 0; j < vector3.size(); ++j)
        {
          if (vector2.at(i) == vector3.at(j))
          {
            have_it = false;
            break;
          }
        }
        if (have_it)
        {
          vector1.push_back(vector2.at(i));
          vector3.push_back(vector2.at(i));
        }
      }
      vector2.clear();
    }else{
      break;
    }
  }
  return module_count;
}

void ControlCenter::RecordCurrentPose(vector<PoseRecord>& JointRecSeq)
{
  for (unsigned int i = 0; i < moduleList.size(); ++i)
  {
    double joint0 = moduleList.at(i)->ModuleObject->GetJoint("Front_wheel_hinge")->GetAngle(0).Radian();
    double joint1 = moduleList.at(i)->ModuleObject->GetJoint("Left_wheel_hinge")->GetAngle(0).Radian();
    double joint2 = moduleList.at(i)->ModuleObject->GetJoint("Right_wheel_hinge")->GetAngle(0).Radian();
    double joint3 = moduleList.at(i)->ModuleObject->GetJoint("Center_hinge")->GetAngle(0).Radian();
    math::Pose currentmodel = moduleList.at(i)->ModuleObject->GetWorldPose();
    if (i < JointRecSeq.size())
    {
      JointRecSeq.at(i).UpdateJoints(joint0,joint1,joint2,joint3,currentmodel);
    }else{
      PoseRecord newRecord(joint0,joint1,joint2,joint3,currentmodel);
      JointRecSeq.push_back(newRecord);
    }
  }
}

void ControlCenter::SetPose(vector<PoseRecord>& JointRecSeq)
{
  // moduleList.at(0)->ModuleObject->SetLinkWorldPose(referencepose, moduleList.at(0)->ModuleObject->GetLink("CircuitHolder"));
  for (unsigned int i = 0; i < JointRecSeq.size(); ++i)
  {
    bool flags[4] = {true,true,true,true};
    SendGaitTableInstance(moduleList.at(i), flags,JointRecSeq.at(i).JointAngles, 3);
    // SendGaitTable(moduleList.at(i), flags, JointRecSeq.at(i).JointAngles);
    moduleList.at(i)->ModuleObject->GetJoint("Front_wheel_hinge")->SetAngle(0,JointRecSeq.at(i).JointAngles[0]);
    moduleList.at(i)->ModuleObject->GetJoint("Left_wheel_hinge")->SetAngle(0,JointRecSeq.at(i).JointAngles[1]);
    moduleList.at(i)->ModuleObject->GetJoint("Right_wheel_hinge")->SetAngle(0,JointRecSeq.at(i).JointAngles[2]);
    moduleList.at(i)->ModuleObject->GetJoint("Center_hinge")->SetAngle(0,JointRecSeq.at(i).JointAngles[3]);
    moduleList.at(i)->ModuleObject->SetWorldPose(JointRecSeq.at(i).Position);
  }
}

void ControlCenter::AddCondition(string conditionid)
{
  bool notExist = true;
  for (unsigned int i = 0; i < CommandConditions.size(); ++i)
  {
    if (CommandConditions.at(i)->condition_id.compare(conditionid) == 0)
    {
      CommandConditions.at(i)->total_count += 1;
      notExist = false;
      break;
    }
  }
  if (notExist)
  {
    ConditionPtr newcondition(new Condition(conditionid));
    CommandConditions.push_back(newcondition);
  }
}

void ControlCenter::FinishOneConditionCommand(string conditionid)
{
  for (unsigned int i = 0; i < CommandConditions.size(); ++i)
  {
    if (CommandConditions.at(i)->condition_id.compare(conditionid) == 0)
    {
      CommandConditions.at(i)->finished_count += 1;
      if (CommandConditions.at(i)->finished_count >= CommandConditions.at(i)->total_count)
      {
        CommandConditions.at(i)->achieved = true;
        cout<<"Wprld: condition: "<<CommandConditions.at(i)->condition_id<<" :achieved"<<endl;
      }
      break;
    }
  }
}

bool ControlCenter::CheckCondition(string conditionid)
{
  for (unsigned int i = 0; i < CommandConditions.size(); ++i)
  {
    if (CommandConditions.at(i)->condition_id.compare(conditionid) == 0)
    {
      return CommandConditions.at(i)->achieved;
    }
  }
  cout<<"World: didn't find the condition"<<endl;
  return true;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ControlCenter)