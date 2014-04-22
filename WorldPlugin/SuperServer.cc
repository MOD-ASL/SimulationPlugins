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

ControlCenter::ControlCenter()
{
  numOfModules = 0;
  infoCounter = 0;
  NeedToSetPtr = 0;
  test_count = 0;
  insertModuleFlag = true;
  // this->FinishFlag = false;
  AlreadyBuild = false;

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

  // Set the step time
  // stateMsg.set_max_step_size(0.01);

  // Change gravity
  // msgs::Set(physicsMsg.mutable_gravity(), math::Vector3(0.01, 0, 0.1));
  // statePub->Publish(stateMsg);
  this->addEntityConnection = event::Events::ConnectAddEntity(boost::bind(&ControlCenter::addEntity2World, this, _1));
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&ControlCenter::OnSystemRunning, this, _1));

  // math::Pose positionTMP(math::Vector3(0, 0, 0), math::Quaternion(1.57, 0, 0));
  // InsertModel("Module0", positionTMP);
  BuildConfigurationFromXML();

  // Here is the test for dynamic shared libraries
  void *lib_handle;
  char *error;
  LibraryTemplate * (*mkr)();

  lib_handle = dlopen("/home/edward/.gazebo/models/SMORES6Uriah/plugins/libSpiderController.so", RTLD_LAZY);
  if (!lib_handle) 
  {
    fprintf(stderr, "%s\n", dlerror());
    exit(1);
  }

  // void *mkr = dlsym(lib_handle, "maker");
  mkr = (LibraryTemplate * (*)())dlsym(lib_handle, "maker");
  if ((error = dlerror()) != NULL)  
  {
    fprintf(stderr, "%s\n", error);
    exit(1);
  }

  LibraryTemplate *Spider = mkr();
  Spider->WhenRunning();
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
  string NewSubName = "~/"+_info+"_Collision";
  transport::SubscriberPtr newWorldSub = node1->Subscribe(NewSubName,&ControlCenter::AutomaticMagneticConnectionManagement, this);
  WorldColSubscriber.push_back(newWorldSub);
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
  // cout<<"World: modlue_node is "<<modlue_node->first_node("name")->value()<<endl;
  while (modlue_node)
  {
    string module_name = modlue_node->first_node("name")->value();
    // cout<<"World: XML: Module name: "<<module_name<<endl;
    string position_string = modlue_node->first_node("position")->value();
    // cout<<"World: XML: position: "<<position_string<<endl;
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
          command_for_current_module->CurrentPriority = msg->priority();
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

      SendGaitTable(GetModulePtrByName(msg->stringmessage()), flags, joint_angles);
      if (InitalJointValue.size()==1)
      {
        // Confiuration connection initialized
        BuildConnectionFromXML();
        cout<<"World: Build the connection"<<endl;
        // This line is a test, please delete in the future
        readFileAndGenerateCommands("Commands");
        cout<<"World: Command been sent"<<endl;
        // Need one right after readcommands function
        currentCommandGroupInitialization();
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

void ControlCenter::CommandManager(void)
{
  CurrentMinimalGroup = MAX_COMMANDGROUP_LENGTH + 1;
  for (unsigned int i = 0; i < ModuleCommandContainer.size(); ++i)
  {
    if (ModuleCommandContainer.at(i)->CommandSquence.size()>0)
    {
      // ---------------- This is a patch for time based gait table --------------------
      if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeInterval > 0)
      {
        ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeInterval -= 1;
        if (ModuleCommandContainer.at(i)->CommandSquence.at(0).TimeInterval == 0)
        {
          if (ModuleCommandContainer.at(i)->CommandSquence.size()>=2 && ModuleCommandContainer.at(i)->CommandSquence.at(0).CommandGroup == ModuleCommandContainer.at(i)->CommandSquence.at(1).CommandGroup)
          {
            ModuleCommandContainer.at(i)->FinishedFlag = true;
            ModuleCommandContainer.at(i)->CurrentPriority = ModuleCommandContainer.at(i)->CommandSquence.at(0).ActualCommandMessage->priority();
          }
        }
      }
      // -------------------------------------------------------------------------------
      if (!ModuleCommandContainer.at(i)->FinishedFlag)
      {
        if (!ModuleCommandContainer.at(i)->ReceivedFlag)
        {
          if (ModuleCommandContainer.at(i)->CommandSquence.at(0).CommandGroup == CurrentCommandGroup)
          {
            // cout<<"World: Still can get in here"<<endl;
            ModuleCommandContainer.at(i)->WhichModule->ModulePublisher->Publish(*(ModuleCommandContainer.at(i)->CommandSquence.at(0).ActualCommandMessage));
            // cout<<"World: Message to '"<<ModuleCommandContainer.at(i)->WhichModule->ModuleID<<"' set joint angle 3 to : "<<ModuleCommandContainer.at(i)->CommandSquence.at(0)->jointgaittable(3)<<endl;
            // cout<<"World: Size of the message is "<<ModuleCommandContainer.at(i)->CommandSquence.size()<<endl;
            ModuleCommandContainer.at(i)->ExecutionFlag = true;
          } 
        }
        if (CurrentMinimalGroup > ModuleCommandContainer.at(i)->CommandSquence.at(0).CommandGroup)
        {
          CurrentMinimalGroup = ModuleCommandContainer.at(i)->CommandSquence.at(0).CommandGroup;
          // cout<<"Update of minimal command here first place, group number: "<<CurrentMinimalGroup<<endl;
        }
      }else{
        // This part will delete the correct priority level command
        for (unsigned int j = 0; j < ModuleCommandContainer.at(i)->CommandSquence.size(); ++j)
        {
          if (ModuleCommandContainer.at(i)->CommandSquence.at(j).ActualCommandMessage->priority()==ModuleCommandContainer.at(i)->CurrentPriority && ModuleCommandContainer.at(i)->CommandSquence.at(j).CommandGroup == CurrentCommandGroup)
          {
            ModuleCommandContainer.at(i)->CommandSquence.erase(ModuleCommandContainer.at(i)->CommandSquence.begin()+j);
            cout<<"World: second place:"<<ModuleCommandContainer.at(i)->WhichModule->ModuleID<<" Size of the message is "<<ModuleCommandContainer.at(i)->CommandSquence.size()<<endl;
            break;
          }
        }
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
        }else{
          // cout<<"Update of minimal command here, group number: "<<CurrentMinimalGroup<<endl;
          // ------------------- Patch for ordered group gait table --------------------
          if (CurrentMinimalGroup > ModuleCommandContainer.at(i)->CommandSquence.at(0).CommandGroup)
          {
            CurrentMinimalGroup = ModuleCommandContainer.at(i)->CommandSquence.at(0).CommandGroup;
          }
          cout<<"Update of minimal command here, group number: "<<CurrentMinimalGroup<<endl;
          // ---------------------------------------------------------------------------
        }
      }
    }
  }
  CurrentCommandGroup = CurrentMinimalGroup;
}
// group: 0: same as the last command; -n: n step before the current group, if no smaller group, then set to the current group; +n: n steps after the current group. Total length is 5000
void ControlCenter::SendGaitTable(SmoresModulePtr module, bool flag[4], double gait_value[4], int group, unsigned int time_stamp, int priority, int msg_type) // unit of time_stamp is millisecond
{
  CommandPtr ConnectionMessage(new command_message::msgs::CommandMessage());
  // command_message::msgs::CommandMessage ConnectionMessage;
  ConnectionMessage->set_messagetype(msg_type);
  ConnectionMessage->set_priority(priority);
  for (int i = 0; i < 4; ++i)
  {
    ConnectionMessage->add_jointgaittablestatus(flag[i]);
    ConnectionMessage->add_jointgaittable(gait_value[i]);
  }
  // --------------- Patch for time based gait table ---------------------------
  CommandPro aNewMessage;
  aNewMessage.ActualCommandMessage = ConnectionMessage;
  aNewMessage.TimeInterval = time_stamp;
  //----------------------------------------------------------------------------

  // -------------- Patch for ordered group gait table ------------------------
  int min_group = MAX_COMMANDGROUP_LENGTH+1;
  for (unsigned int i = 0; i <moduleList.size();++i)
  {
    if (moduleList.at(i)->ModuleCommandContainer)
    {
      if (moduleList.at(i)->ModuleCommandContainer->CommandSquence.at(0).CommandGroup < min_group)
      {
        min_group = moduleList.at(i)->ModuleCommandContainer->CommandSquence.at(0).CommandGroup;
      }
    }
  }
  if (min_group == MAX_COMMANDGROUP_LENGTH+1)
  {
    min_group = 0;
  }  

  if (!module->ModuleCommandContainer)
  {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));
    if (group <= 0)
    {
      aNewMessage.CommandGroup = min_group;
    }else{
      aNewMessage.CommandGroup = min_group + group;
      if (aNewMessage.CommandGroup>MAX_COMMANDGROUP_LENGTH)
      {
        aNewMessage.CommandGroup = aNewMessage.CommandGroup-MAX_COMMANDGROUP_LENGTH-1;
        if (group != 1)
        {
          CommandPtr ConnectionMessageTmp(new command_message::msgs::CommandMessage());
          ConnectionMessageTmp->set_messagetype(3);
          ConnectionMessageTmp->set_priority(0);
          for (int i = 0; i < 4; ++i)
          {
            ConnectionMessageTmp->add_jointgaittablestatus(0);
            ConnectionMessageTmp->add_jointgaittable(0);
          }
          CommandPro TmpNewMessage;
          TmpNewMessage.ActualCommandMessage = ConnectionMessageTmp;
          TmpNewMessage.TimeInterval = 0;
          TmpNewMessage.CommandGroup = MAX_COMMANDGROUP_LENGTH;
          new_command_message->CommandSquence.push_back(TmpNewMessage);
        }
      }
    }
    // ModuleCommandsPtr new_command_message(new ModuleCommands(module));
    new_command_message->CommandSquence.push_back(aNewMessage);
    ModuleCommandContainer.push_back(new_command_message);
    module->ModuleCommandContainer = new_command_message;
  }else{
    if (group == 0)
    {
      aNewMessage.CommandGroup = module->ModuleCommandContainer->CommandSquence.back().CommandGroup;
      if (priority == 0)
      {
        module->ModuleCommandContainer->CommandSquence.push_back(aNewMessage);
      }else{
        for (unsigned int i = module->ModuleCommandContainer->CommandSquence.size()-1; i >= 0; --i)
        {
          if (priority > module->ModuleCommandContainer->CommandSquence.at(i).ActualCommandMessage->priority() && module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup == aNewMessage.CommandGroup)
          {
            module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin()+i,aNewMessage);
            break;
          }
        }
      }
    }
    if (group > 0)
    {
      aNewMessage.CommandGroup = module->ModuleCommandContainer->CommandSquence.back().CommandGroup + group;
      if (aNewMessage.CommandGroup>MAX_COMMANDGROUP_LENGTH)
      {
        aNewMessage.CommandGroup -= MAX_COMMANDGROUP_LENGTH - 1;
      }
      module->ModuleCommandContainer->CommandSquence.push_back(aNewMessage);
    }
    if (group < 0)
    {
      aNewMessage.CommandGroup = module->ModuleCommandContainer->CommandSquence.back().CommandGroup + group;
      if(aNewMessage.CommandGroup<0)
      {
        aNewMessage.CommandGroup += MAX_COMMANDGROUP_LENGTH + 1;
      }
      if (aNewMessage.CommandGroup < min_group)
      {
        aNewMessage.CommandGroup = min_group;
      }
      if (module->ModuleCommandContainer->CommandSquence.front().CommandGroup>aNewMessage.CommandGroup)
      {
        module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin(),aNewMessage);
      }else{
        for (unsigned int i = module->ModuleCommandContainer->CommandSquence.size()-1; i >= 0; --i)
        {
          // Haven't implement priority
          if (module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup <= aNewMessage.CommandGroup && module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup <= module->ModuleCommandContainer->CommandSquence.back().CommandGroup)
          {
            module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin()+i+1,aNewMessage);
            break;
          }
          if (module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup > aNewMessage.CommandGroup && module->ModuleCommandContainer->CommandSquence.at(i-1).CommandGroup > aNewMessage.CommandGroup && module->ModuleCommandContainer->CommandSquence.at(i-1).CommandGroup > module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup)
          {
            module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin()+i+1,aNewMessage);
            break;
          }
        }
      }
    }
    // --------------------------------------------------------------------------
  }
  // ModuleCommandsPtr new_command_message(new ModuleCommands(module));
  // new_command_message->CommandSquence.push_back(ConnectionMessage);
  // ModuleCommandContainer.push_back(new_command_message);
  // module->ModulePublisher->Publish(ConnectionMessage);

}

void ControlCenter::SendGaitTable(SmoresModulePtr module, int joint_ID, double gait_value, int group, unsigned int time_stamp, int priority, int msg_type)
{
  bool flag[4] = {false};
  double gait_values[4] = {0};
  flag[joint_ID] = true;
  gait_values[joint_ID] = gait_value;
  SendGaitTable(module, flag, gait_values, priority, msg_type);
}

void ControlCenter::SendPosition(SmoresModulePtr module, double x, double y, double orientation_angle, int group, unsigned int time_stamp, int priority)
{
  CommandPtr ConnectionMessage(new command_message::msgs::CommandMessage());
  // command_message::msgs::CommandMessage ConnectionMessage;
  ConnectionMessage->set_messagetype(2);
  ConnectionMessage->set_priority(priority);
  ConnectionMessage->mutable_positionneedtobe()->mutable_position()->set_x(x);
  ConnectionMessage->mutable_positionneedtobe()->mutable_position()->set_y(y);
  ConnectionMessage->mutable_positionneedtobe()->mutable_position()->set_z(orientation_angle);
  ConnectionMessage->mutable_positionneedtobe()->mutable_orientation()->set_x(0);
  ConnectionMessage->mutable_positionneedtobe()->mutable_orientation()->set_y(0);
  ConnectionMessage->mutable_positionneedtobe()->mutable_orientation()->set_z(0);
  ConnectionMessage->mutable_positionneedtobe()->mutable_orientation()->set_w(0);

  // if (!module->ModuleCommandContainer)
  // {
  //   ModuleCommandsPtr new_command_message(new ModuleCommands(module));
  //   new_command_message->CommandSquence.push_back(ConnectionMessage);
  //   ModuleCommandContainer.push_back(new_command_message);
  //   module->ModuleCommandContainer = new_command_message;
  // }else{
  //   if (priority == 0)
  //   {
  //     module->ModuleCommandContainer->CommandSquence.push_back(ConnectionMessage);
  //   }else
  //   {
  //     for (unsigned int i = 0; i < module->ModuleCommandContainer->CommandSquence.size(); ++i)
  //     {
  //       if (priority > module->ModuleCommandContainer->CommandSquence.at(i)->priority())
  //       {
  //         module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin()+i,ConnectionMessage);
  //         break;
  //       }
  //     }
  //   }
  // }

  // --------------- Patch for time based gait table ---------------------------
  CommandPro aNewMessage;
  aNewMessage.ActualCommandMessage = ConnectionMessage;
  aNewMessage.TimeInterval = time_stamp;
  //----------------------------------------------------------------------------

  // -------------- Patch for ordered group gait table ------------------------
  int min_group = MAX_COMMANDGROUP_LENGTH+1;
  for (unsigned int i = 0; i <moduleList.size();++i)
  {
    if (moduleList.at(i)->ModuleCommandContainer)
    {
      if (moduleList.at(i)->ModuleCommandContainer->CommandSquence.at(0).CommandGroup < min_group)
      {
        min_group = moduleList.at(i)->ModuleCommandContainer->CommandSquence.at(0).CommandGroup;
      }
    }
  }
  if (min_group == MAX_COMMANDGROUP_LENGTH+1)
  {
    min_group = 0;
  } 

  if (!module->ModuleCommandContainer)
  {
    
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));
    if (group <= 0)
    {
      aNewMessage.CommandGroup = min_group;
    }else{
      aNewMessage.CommandGroup = min_group + group;
      if (aNewMessage.CommandGroup>MAX_COMMANDGROUP_LENGTH)
      {
        aNewMessage.CommandGroup = aNewMessage.CommandGroup-MAX_COMMANDGROUP_LENGTH-1;
        CommandPtr ConnectionMessageTmp(new command_message::msgs::CommandMessage());
        ConnectionMessageTmp->set_messagetype(3);
        ConnectionMessageTmp->set_priority(0);
        for (int i = 0; i < 4; ++i)
        {
          ConnectionMessageTmp->add_jointgaittablestatus(0);
          ConnectionMessageTmp->add_jointgaittable(0);
        }
        CommandPro TmpNewMessage;
        TmpNewMessage.ActualCommandMessage = ConnectionMessageTmp;
        TmpNewMessage.TimeInterval = 0;
        TmpNewMessage.CommandGroup = MAX_COMMANDGROUP_LENGTH;
        new_command_message->CommandSquence.push_back(TmpNewMessage);
      }
    }
    // ModuleCommandsPtr new_command_message(new ModuleCommands(module));
    new_command_message->CommandSquence.push_back(aNewMessage);
    ModuleCommandContainer.push_back(new_command_message);
    module->ModuleCommandContainer = new_command_message;
  }else{
    if (group == 0)
    {
      aNewMessage.CommandGroup = module->ModuleCommandContainer->CommandSquence.back().CommandGroup;
      if (priority == 0)
      {
        module->ModuleCommandContainer->CommandSquence.push_back(aNewMessage);
      }else{
        for (unsigned int i = module->ModuleCommandContainer->CommandSquence.size()-1; i >= 0; --i)
        {
          if (priority > module->ModuleCommandContainer->CommandSquence.at(i).ActualCommandMessage->priority() && module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup == aNewMessage.CommandGroup)
          {
            module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin()+i,aNewMessage);
            break;
          }
        }
      }
    }
    if (group > 0)
    {
      aNewMessage.CommandGroup = module->ModuleCommandContainer->CommandSquence.back().CommandGroup + group;
      if (aNewMessage.CommandGroup>MAX_COMMANDGROUP_LENGTH)
      {
        aNewMessage.CommandGroup -= MAX_COMMANDGROUP_LENGTH - 1;
      }
      module->ModuleCommandContainer->CommandSquence.push_back(aNewMessage);
    }
    if (group < 0)
    {
      aNewMessage.CommandGroup = module->ModuleCommandContainer->CommandSquence.back().CommandGroup + group;
      if(aNewMessage.CommandGroup<0)
      {
        aNewMessage.CommandGroup += MAX_COMMANDGROUP_LENGTH + 1;
      }
      if (aNewMessage.CommandGroup < min_group)
      {
        aNewMessage.CommandGroup = min_group;
      }
      if (module->ModuleCommandContainer->CommandSquence.front().CommandGroup>aNewMessage.CommandGroup)
      {
        module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin(),aNewMessage);
      }else{
        for (unsigned int i = module->ModuleCommandContainer->CommandSquence.size()-1; i >= 0; --i)
        {
          // Haven't implement priority
          if (module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup <= aNewMessage.CommandGroup && module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup <= module->ModuleCommandContainer->CommandSquence.back().CommandGroup)
          {
            module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin()+i+1,aNewMessage);
            break;
          }
          if (module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup > aNewMessage.CommandGroup && module->ModuleCommandContainer->CommandSquence.at(i-1).CommandGroup > aNewMessage.CommandGroup && module->ModuleCommandContainer->CommandSquence.at(i-1).CommandGroup > module->ModuleCommandContainer->CommandSquence.at(i).CommandGroup)
          {
            module->ModuleCommandContainer->CommandSquence.insert(module->ModuleCommandContainer->CommandSquence.begin()+i+1,aNewMessage);
            break;
          }
        }
      }
    }
    // --------------------------------------------------------------------------
  }
  // ModuleCommandsPtr new_command_message(new ModuleCommands(module));
  // new_command_message->CommandSquence.push_back(ConnectionMessage);
  // ModuleCommandContainer.push_back(new_command_message);
  // module->ModulePublisher->Publish(ConnectionMessage);
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

void ControlCenter::readFileAndGenerateCommands(const char* fileName)
{
  string output;
  ifstream infile;
  infile.open(fileName);
  int smallcount = 0;
  double joints_values[4] = {0,0,0,0};
  bool flags[4] = {true,true,true,true};
  int model_number;
  int group_num = 0;
  unsigned int time_interval = 0;
  if (infile.is_open()) {
    while (!infile.eof()) {
      infile >> output;
      // cout<<output<<endl;
      switch(smallcount){
        case 0:model_number = atoi(output.c_str());break;
        case 1:joints_values[0] = atof(output.c_str());break;
        case 2:joints_values[1] = atof(output.c_str());break;
        case 3:joints_values[2] = atof(output.c_str());break;
        case 4:joints_values[3] = atof(output.c_str());break;
        case 5:group_num = atoi(output.c_str());break;
        case 6:time_interval = atoi(output.c_str());break;
      }
      smallcount++;
      if (smallcount == 7)
      {
        smallcount=0;
        SendGaitTable(moduleList.at(model_number), flags, joints_values, group_num, time_interval);
      }
    }
  }
  infile.close();
}

void ControlCenter::currentCommandGroupInitialization(void)
{
  int min_group = MAX_COMMANDGROUP_LENGTH+1;
  for (unsigned int i = 0; i <moduleList.size();++i)
  {
    if (moduleList.at(i)->ModuleCommandContainer)
    {
      if (moduleList.at(i)->ModuleCommandContainer->CommandSquence.at(0).CommandGroup < min_group)
      {
        min_group = moduleList.at(i)->ModuleCommandContainer->CommandSquence.at(0).CommandGroup;
      }
    }
  }
  CurrentCommandGroup = min_group;
}
    
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ControlCenter)