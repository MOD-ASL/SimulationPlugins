#include "WorldServer.hh"

using std::string;
using std::vector;
using std::cout;

using rapidxml::file;
using rapidxml::xml_document;
using rapidxml::xml_node;

namespace gazebo
{
WorldServer::WorldServer()
{
  needToSetPtr = 0;
  autoMagneticConnectionFlag = false;
  configurationFile = "";
  // All code below this line is for testing
} // WorldServer::WorldServer
WorldServer::~WorldServer(){}
void WorldServer::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->currentWorld = _parent;
  // Create a new transport node
  transport::NodePtr node(new transport::Node());
  // Initialize the node with the world name
  node->Init(_parent->GetName());
  // Create a publisher to publish welcome information
  this->welcomePub = node->Advertise<msgs::GzString>("~/Welcome");
  // Event binding functions
  this->addEntityConnection = event::Events::ConnectAddEntity(
      boost::bind(&WorldServer::AddEntityToWorld, this, _1));
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&WorldServer::OnSystemRunning, this, _1));
  // Perform extra initializations
  ExtraInitializationInLoad(_parent,_sdf);
} // WorldServer::Load
void WorldServer::ExtraInitializationInLoad(
    physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{} // WorldServer::ExtraInitializationInLoad
LibraryTemplate *WorldServer::DynamicallyLoadedLibrary(
   const char* library_path, void *lib_handle)
{
  char *error;
  LibraryTemplate * (*mkr)();
  lib_handle = dlopen(library_path, RTLD_LAZY);
  if (!lib_handle) {
    fprintf(stderr, "%s\n", dlerror());
    exit(1);
  }
  mkr = (LibraryTemplate * (*)())dlsym(lib_handle, "maker");
  if ((error = dlerror()) != NULL) {
    fprintf(stderr, "%s\n", error);
    exit(1);
  }
  return mkr();
} // WorldServer::DynamicallyLoadedLibrary
void WorldServer::CloseLoadedLibrary(void **lib_handle)
{
  dlclose(lib_handle);
} // WorldServer::CloseLoadedLibrary
void WorldServer::EnableAutoMagneticConnection(void)
{
  this->autoMagneticConnectionFlag = true;
}
void WorldServer::AddEntityToWorld(std::string & _info)
{
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Welcome message generation each time a new model has been added
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  int model_number = this->currentWorld->GetModelCount();
  string current_message = "Model"+Int2String(model_number);
  // A log line, which can be deleted in the future
  // cout<<"World: Number of models: "<<model_number<<endl;
  msgs::GzString welcome_msgs;
  welcome_msgs.set_data(current_message);
  welcomePub->Publish(welcome_msgs);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Dynamic publisher generation for Commands between world and modules
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  transport::NodePtr node(new transport::Node());
  node->Init(_info);
  // Topic name for publisher
  string topic_name = "~/" + _info + "_world";
  transport::PublisherPtr new_module_pub 
      = node->Advertise<command_message::msgs::CommandMessage>(topic_name);
  // Topic name for subscriber
  topic_name = "~/" + _info + "_model";
  transport::SubscriberPtr new_module_sub 
      = node->Subscribe(topic_name,&WorldServer::FeedBackMessageDecoding, this);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Initialize module object and store the pointers into the module vector
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  unsigned int how_many_modules = moduleList.size();
  SmoresModulePtr new_module(new SmoresModule(
      _info, true, new_module_pub, new_module_sub, how_many_modules));
  // TODO: find a elegent way for the following line
  new_module->ManuallyNodeInitial(new_module);
  moduleList.push_back(new_module);
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Dynamic subscriber of collision topic
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if (autoMagneticConnectionFlag) {
    string new_sub_name = "~/"+_info+"_Collision";
    transport::SubscriberPtr new_world_sub = node->Subscribe(
        new_sub_name,&WorldServer::AutomaticMagneticConnectionManagement, this);
    WorldColSubscriber.push_back(new_world_sub);
  }
} // WorldServer::AddEntityToWorld
void WorldServer::OnSystemRunning(const common::UpdateInfo & _info)
{
  // Main command execution procedure
  CommandManager();
  // Extra Code that needs to be run in each iteration
  OnSystemRunningExtra(_info);
} // WorldServer::OnSystemRunning
void WorldServer::OnSystemRunningExtra(
    const common::UpdateInfo & _info){} // WorldServer::OnSystemRunningExtra
void WorldServer::BuildConfigurationFromXML(string file_name)
{
  file<> xmlFile(file_name.c_str());
  xml_document<> doc;    // character type defaults to char
  doc.parse<0>(xmlFile.data());
  xml_node<> *modlue_node = doc.first_node("configuration")
      ->first_node("modules")->first_node("module");
  while (modlue_node) {
    string module_name = modlue_node->first_node("name")->value();
    string position_string = modlue_node->first_node("position")->value();
    double coordinates[3] = {0,0,0};
    for (int i = 0; i < 3; ++i) {
      coordinates[i] 
          = atof(position_string.substr(0,position_string.find(" ")).c_str());
      position_string = position_string.substr(position_string.find(" ")+1);
    }
    vector<double> orientation;
    while(position_string.length()>0){
      string::size_type next_space = position_string.find_first_of(" ");
      orientation.push_back(atof(position_string.substr(0,next_space).c_str()));
      if (next_space == string::npos) {
        position_string = position_string.substr(position_string.length());
      }else{
        position_string = position_string.substr(next_space+1);
      }
    }
    math::Quaternion model_orientation;
    if (orientation.size() == 3) {
      model_orientation.SetFromEuler(orientation.at(0),orientation.at(1),
          orientation.at(2));
    }
    if (orientation.size() == 4) {
      model_orientation.Set(orientation.at(0),orientation.at(1),orientation.at(2),
          orientation.at(3));
    }
    math::Pose model_position(
        math::Vector3(coordinates[0], coordinates[1], coordinates[2]), 
        model_orientation);
    string joints_string = modlue_node->first_node("joints")->value();
    if (modlue_node->first_node("path"))
    {
      InsertModel(module_name, model_position, joints_string,modlue_node->
          first_node("path")->value());
    }else{
      InsertModel(module_name, model_position, joints_string);
    }
    modlue_node = modlue_node->next_sibling();
  }
  configurationFile = file_name;
} // WorldServer::BuildConfigurationFromXML
void WorldServer::BuildConnectionFromXML(string file_name)
{
  file<> xmlFile(file_name.c_str());
  xml_document<> doc;    // character type defaults to char
  doc.parse<0>(xmlFile.data());
  xml_node<> *connection_node = doc.first_node("configuration")
      ->first_node("connections")->first_node("connection");
  while (connection_node) {
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
    SmoresModulePtr model1_ptr = GetModulePtrByName(module1_name);
    SmoresModulePtr model2_ptr = GetModulePtrByName(module2_name);
    math::Pose module1_pose = model1_ptr->ModuleObject->GetWorldPose();
    math::Pose module2_pose = model2_ptr->ModuleObject->GetWorldPose();
    ActiveConnect(model1_ptr,model2_ptr,node1_ID,node2_ID, angle, distance);
    connection_node = connection_node->next_sibling();
  }
  cout<<"World: Finishing up."<<endl;
  configurationFile = "";
} // WorldServer::BuildConnectionFromXML
void WorldServer::FeedBackMessageDecoding(CommandMessagePtr &msg)
{
  // Command feedback handler
  if (msg->messagetype()==0) {
    string module_name 
        = msg->stringmessage().substr(0,msg->stringmessage().find(":"));
    ModuleCommandsPtr command_for_current_module 
        = GetModulePtrByName(module_name)->moduleCommandContainer;
    if (command_for_current_module) {
      if (command_for_current_module->ExecutionFlag) {
        command_for_current_module->ReceivedFlag = true;
        // cout<<"World: "<<module_name<<" set receive flag"<<endl;
      }
    }
    string second_field = msg->stringmessage().substr(
        msg->stringmessage().find(":")+1,string::npos);
    if (second_field.compare("finished")==0) {
      cout<<"World: "<<module_name<<" Execution finished"<<endl;
      if (command_for_current_module) {
        if (command_for_current_module->ExecutionFlag) {
          command_for_current_module->FinishedFlag = true;
          command_for_current_module->ExecutionFlag = false;
        }
      }
      command_message::msgs::CommandMessage finish_confirm_message;
      finish_confirm_message.set_messagetype(0);
      GetModulePtrByName(module_name)->ModulePublisher
          ->Publish(finish_confirm_message);
    }
  }
  // Inserted model initialization
  if (msg->messagetype()==5) {
    // Initalization functions
    moduleList.at(needToSetPtr)->SetModulePtr(
        currentWorld->GetModel(moduleList.at(needToSetPtr)->ModuleID));
    // cout<<"World: Asign the pointer to module: "
    //     <<moduleList.at(needToSetPtr)->ModuleID<<endl;
    // Need a function when delete a entity, this value needs to be decrease
    needToSetPtr += 1;
    if (initalJointValue.size()>0) {
      bool flags[4] = {true,true,true,true};
      double joint_angles[4] = {0};
      string joint_values_string = initalJointValue.at(0);
      for (int i = 0; i < 4; ++i) {
        if (i<3) {
          joint_angles[i] = atof(joint_values_string
              .substr(0,joint_values_string.find(" ")).c_str());
          joint_values_string = joint_values_string
              .substr(joint_values_string.find(" ")+1);
        }else {
          joint_angles[i] = atof(joint_values_string.substr(0).c_str());
        }
      }
      currentWorld->GetModel(msg->stringmessage())
          ->GetJoint("Front_wheel_hinge")->SetPosition(0,joint_angles[0]);
      currentWorld->GetModel(msg->stringmessage())
          ->GetJoint("Left_wheel_hinge")->SetPosition(0,joint_angles[1]);
      currentWorld->GetModel(msg->stringmessage())
          ->GetJoint("Right_wheel_hinge")->SetPosition(0,joint_angles[2]);
      currentWorld->GetModel(msg->stringmessage())
          ->GetJoint("Center_hinge")->SetPosition(0,joint_angles[3]);
      currentWorld->GetModel(msg->stringmessage())
          ->SetLinkWorldPose(initialPosition.at(0),
          currentWorld->GetModel(msg->stringmessage())->GetLink("CircuitHolder"));
      SendGaitTableInstance(
          GetModulePtrByName(msg->stringmessage()), flags, joint_angles,3);
      if (GetInitialJointSequenceSize() == 1) {
        if (configurationFile.size() > 0) {
          // Confiuration connection initialized
          BuildConnectionFromXML(configurationFile);
          cout<<"World: Build the connection"<<endl;
        }
      }
      ExtraWorkWhenModelInserted(msg);
      initalJointValue.erase(initalJointValue.begin());
      initialPosition.erase(initialPosition.begin());
    }
  }
} // WorldServer::FeedBackMessageDecoding
void WorldServer::ExtraWorkWhenModelInserted(CommandMessagePtr &msg)
{} // WorldServer::ExtraWorkWhenModelInserted
void WorldServer::AutomaticMagneticConnectionManagement(CollisionMessagePtr &msg)
{
  string model_of_collision1 
      = msg->collision1().substr(0,msg->collision1().find("::"));
  string model_of_collision2 
      = msg->collision2().substr(0,msg->collision2().find("::"));
  string link_of_collision1 = msg->collision1().substr(msg->collision1()
      .find("::")+2,msg->collision1().rfind("::")-msg->collision1().find("::")-2);
  string link_of_collision2 = msg->collision2().substr(msg->collision2()
      .find("::")+2,msg->collision2().rfind("::")-msg->collision2().find("::")-2);
  SmoresModulePtr model1_ptr = GetModulePtrByName(model_of_collision1);
  SmoresModulePtr model2_ptr = GetModulePtrByName(model_of_collision2);
  int node_of_model1 = GetNodeIDByName(link_of_collision1);
  int node_of_model2 = GetNodeIDByName(link_of_collision2);
  //---------------------- Find the pending connection request ----------------
  bool found_pending_one = false;
  for (unsigned int i = 0; i < pendingRequest.size(); ++i)
  {
    if (pendingRequest.at(i).SameCollision(model_of_collision1,
        model_of_collision2,link_of_collision1,link_of_collision2)) {
      pendingRequest.erase(pendingRequest.begin()+i);
      found_pending_one = true;
      break;
    }
  }
  if (node_of_model1<4 && node_of_model2<4) {
    if (!found_pending_one) {
  //---------------------- Add new pending connection request -----------------
      if ((!AlreadyConnected(model1_ptr,node_of_model1)) 
          && (!AlreadyConnected(model2_ptr,node_of_model2)) 
          && (!AlreadyConnected(model1_ptr,model2_ptr))) {
        // This part is used to check the distance between robots
        math::Vector3 center_of_model1 = currentWorld->GetModel(
            model_of_collision1)->GetLink("CircuitHolder")->GetWorldPose().pos;
        math::Vector3 center_of_model2 = currentWorld->GetModel(
            model_of_collision2)->GetLink("CircuitHolder")->GetWorldPose().pos;
        double distance_between_two_centers
            = (center_of_model1-center_of_model2).GetLength();
        if (distance_between_two_centers<VALIDCONNECTIONDISUPPER
            && distance_between_two_centers>VALIDCONNECTIONDISLOWER)
        {
          // cout<<"World: Distance between centers: "
          //     <<distance_between_two_centers<<endl;
          CollisionInformation new_connection_request(
              model_of_collision1, model_of_collision2,
              link_of_collision1, link_of_collision2);
          pendingRequest.push_back(new_connection_request);
          cout<<"World: An pending entry has been established: '"
              << model_of_collision1+":"+link_of_collision1+"::"
              + model_of_collision2 + ":"+link_of_collision2<<"'"<<endl;
        }
      }
    }else{
      // The lighter cluster connects to heavier cluster
      if (CountModules(model1_ptr) < CountModules(model2_ptr)) {
        SmoresModulePtr tmp_module_ptr = model1_ptr;
        model1_ptr = model2_ptr;
        model2_ptr = tmp_module_ptr;
        int tmp_node = node_of_model1;
        node_of_model1 = node_of_model2;
        node_of_model2 = tmp_node;
      }
  //-------- Do the real connection (including generate dynamic joint) --------
      ActiveConnect(model1_ptr,model2_ptr,node_of_model1,node_of_model2);
    }
  }
} // WorldServer::AutomaticMagneticConnectionManagement
// TODO: Since there is a huge change in this function, it should be tested
void WorldServer::ConnectAndDynamicJointGeneration(
    SmoresModulePtr module_1, SmoresModulePtr module_2, 
    int node1_ID, int node2_ID, SmoresEdgePtr an_edge)
{
  math::Pose ContactLinkPos = module_1->GetLinkPtr(node1_ID)->GetWorldPose();
  math::Pose PosOfTheOtherModel = module_2->GetLinkPtr(node2_ID)->GetWorldPose();
  physics::LinkPtr Link1, Link2;
  math::Vector3 axis;
  math::Pose position_of_module1(math::Vector3(0,0,0),math::Quaternion(0,0,0,0));
  math::Pose position_of_module2(math::Vector3(0,0,0),math::Quaternion(0,0,0,0));
  NewPositionCalculation(an_edge, ContactLinkPos, PosOfTheOtherModel, 
      node1_ID, node2_ID, &position_of_module1, &position_of_module2);
  if (node1_ID == 0 || node1_ID == 3) {
    axis.Set(0,1,0);
  }
  if (node1_ID == 1 || node1_ID == 2) {
    axis.Set(1,0,0);
  }
  //+++++ This part of the code set the correct position of the models ++++++++
  Link1 = module_1->GetLinkPtr(node1_ID);
  Link2 = module_2->GetLinkPtr(node2_ID);

  module_1->ModuleObject->SetLinkWorldPose(position_of_module1,Link1);
  module_2->ModuleObject->SetLinkWorldPose(position_of_module2,Link2);

  //++++ This part of the code generate the dynamic joint +++++++++++++++++++++
  physics::JointPtr DynamicJoint;
  DynamicJoint = currentWorld->GetPhysicsEngine()->CreateJoint(
      "prismatic", module_1->ModuleObject);
  DynamicJoint->Attach(Link1, Link2);
  DynamicJoint->Load(
      Link1, Link2, math::Pose(math::Vector3(0,-0.00,0),math::Quaternion()));
  DynamicJoint->SetAxis(0, axis);
  string joint_name = "DynamicJoint" + Int2String((int)dynamicConnections.size());
  DynamicJoint->SetName(joint_name);
  module_1->ModuleObject->GetJointController()->AddJoint(DynamicJoint);
  DynamicJoint->SetHighStop(0,math::Angle(0.0000001));
  DynamicJoint->SetLowStop(0,math::Angle(-0.0000001));
  DynamicJoint->SetPosition(0,0.0);
  dynamicConnections.push_back(DynamicJoint);
  // This is necessary for easy access of the dynamic joint
  an_edge->DynamicJointPtr = dynamicConnections.back();
} // WorldServer::ConnectAndDynamicJointGeneration
void WorldServer::RotationQuaternionCalculation(math::Vector3 normal_axis,
    math::Vector3 z_axis_of_link1, math::Vector3 z_axis_of_link2, 
    math::Vector3 first_rotation, math::Vector3 second_rotation,
    math::Quaternion *first_rotation_of_link2, 
    math::Quaternion *second_rotation_of_link2)
{
  first_rotation_of_link2->SetFromEuler(first_rotation);
  double direction_reference 
      = z_axis_of_link1.Cross(z_axis_of_link2).Dot(normal_axis);
  if (direction_reference>0)
  {
    second_rotation_of_link2->SetFromEuler(second_rotation);
  }else{
    second_rotation_of_link2->SetFromEuler(-second_rotation);
  }
} // WorldServer::RotationQuaternionCalculation
void WorldServer::NewPositionCalculation(SmoresEdgePtr an_edge,
    math::Pose old_pose_of_module1, math::Pose old_pose_of_module2, 
    int node1_ID, int node2_ID, 
    math::Pose *new_pose_of_module1, math::Pose *new_pose_of_module2)
{
  double z_rotation[3] = {PI,PI/2,0};
  math::Vector3 new_position_of_link1 = old_pose_of_module1.pos;
  math::Quaternion new_direction_of_link1 = old_pose_of_module1.rot;
  math::Vector3 new_position_of_link2 = old_pose_of_module1.pos 
      - (0.1+an_edge->Distance)*old_pose_of_module1.rot.GetYAxis();
  math::Vector3 z_axis_of_old_module2 = old_pose_of_module2.rot.GetZAxis();
  math::Vector3 z_axis_of_new_direction = new_direction_of_link1.GetZAxis();
  math::Vector3 x_axis_of_new_direction = new_direction_of_link1.GetXAxis();
  math::Vector3 new_z_axis = z_axis_of_old_module2
      .Dot(z_axis_of_new_direction)*z_axis_of_new_direction 
      + z_axis_of_old_module2
      .Dot(x_axis_of_new_direction)*x_axis_of_new_direction;
  if (node1_ID == 1 || node1_ID == 2) {
    for (int i = 0; i < 3; ++i) {
      z_rotation[i] = ConversionForAngleOverPi(z_rotation[i] - 3.0/2.0*PI);
    }
    new_position_of_link2 = old_pose_of_module1.pos 
        + (0.1+an_edge->Distance)*old_pose_of_module1.rot.GetXAxis();
    new_z_axis = z_axis_of_old_module2
        .Dot(z_axis_of_new_direction)*z_axis_of_new_direction 
        + z_axis_of_old_module2
        .Dot(new_direction_of_link1.GetYAxis())*new_direction_of_link1.GetYAxis();
  }
  if (node1_ID == 3) {
    for (int i = 0; i < 3; ++i) {
      z_rotation[i] = ConversionForAngleOverPi(z_rotation[i] - PI);
    }
    new_position_of_link2 = old_pose_of_module1.pos 
        + (0.1+an_edge->Distance)*old_pose_of_module1.rot.GetYAxis();
    new_z_axis = z_axis_of_old_module2
        .Dot(z_axis_of_new_direction)*z_axis_of_new_direction 
        + z_axis_of_old_module2
        .Dot(x_axis_of_new_direction)*x_axis_of_new_direction;
  }
  new_z_axis = new_z_axis.Normalize();
  double cosine_value = new_z_axis.Dot(z_axis_of_new_direction)>0?
      min(1.0,new_z_axis.Dot(z_axis_of_new_direction)):
      max(-1.0,new_z_axis.Dot(z_axis_of_new_direction));
  double angle_between_z_axes = acos(cosine_value); // + an_edge->Angle;
  math::Quaternion first_rotation_of_link2;
  math::Quaternion second_rotation_of_link2;
  math::Quaternion new_direction_of_link2;
  if (node2_ID==0)
  {
    RotationQuaternionCalculation(old_pose_of_module2.rot.GetYAxis(),
        z_axis_of_new_direction,new_z_axis, 
        math::Vector3(0,0,z_rotation[0]),math::Vector3(0, angle_between_z_axes ,0),
        &first_rotation_of_link2, &second_rotation_of_link2);
  }
  if (node2_ID==1)
  {
    RotationQuaternionCalculation(old_pose_of_module2.rot.GetXAxis(),
        z_axis_of_new_direction,new_z_axis, 
        math::Vector3(0,0,z_rotation[1]),math::Vector3(angle_between_z_axes, 0, 0),
        &first_rotation_of_link2, &second_rotation_of_link2);
  }
  if (node2_ID==2)
  {
    RotationQuaternionCalculation(old_pose_of_module2.rot.GetXAxis(),
        z_axis_of_new_direction,new_z_axis, 
        math::Vector3(0,0,z_rotation[1]),math::Vector3(angle_between_z_axes, 0, 0),
        &first_rotation_of_link2, &second_rotation_of_link2);
  }
  if (node2_ID==3)
  {
    RotationQuaternionCalculation(old_pose_of_module2.rot.GetYAxis(),
        z_axis_of_new_direction,new_z_axis, 
        math::Vector3(0,0,z_rotation[2]),math::Vector3(0, angle_between_z_axes ,0),
        &first_rotation_of_link2, &second_rotation_of_link2);
  }
  new_direction_of_link2 
      = new_direction_of_link1*first_rotation_of_link2*second_rotation_of_link2;
  new_pose_of_module1->Set(new_position_of_link1,new_direction_of_link1);
  new_pose_of_module2->Set(new_position_of_link2,new_direction_of_link2);
} // WorldServer::NewPositionCalculation
double WorldServer::ConversionForAngleOverPi(double angle)
{
  if (abs(angle) > PI+0.0000000001){
    return angle>0?angle-2*PI:2*PI+angle;
  }
  return angle;
} // WorldServer::ConversionForAngleOverPi
void WorldServer::DynamicJointDestroy(SmoresEdgePtr edge)
{
  edge->DynamicJointPtr->Detach();
  for (unsigned int i = 0; i < dynamicConnections.size(); ++i) {
    if (edge->DynamicJointPtr == dynamicConnections.at(i)) {
      dynamicConnections.at(i).reset();
      edge->DynamicJointPtr->Fini();
      edge->DynamicJointPtr.reset();
      dynamicConnections.erase(dynamicConnections.begin()+i);
      break;
    }
  }
} // WorldServer::DynamicJointDestroy
void WorldServer::InsertModel(string name, math::Pose position, 
    string joint_angles)
{
  if (!currentWorld->GetModel(name))
  {
    sdf::SDFPtr model_sdf;
    model_sdf.reset(new sdf::SDF);
    sdf::init(model_sdf);
    sdf::readFile(MODULEPATH, model_sdf);
    sdf::ElementPtr model_element = model_sdf->root->GetElement("model");
    math::Pose position_calibrate(
        math::Vector3(0, 0, -0.05), math::Quaternion(0, 0, 0));
    model_element->GetAttribute("name")->Set(name);
    model_element->GetElement("pose")->Set(position_calibrate);
    currentWorld->InsertModelSDF(*model_sdf);
    initalJointValue.push_back(joint_angles);
    initialPosition.push_back(position);
  }else{
    Color::Modifier red_log(Color::FG_RED);
    Color::Modifier def_log(Color::FG_DEFAULT);
    cout<<red_log<<"WARNING: World: Insertion failed: module name exists."
        <<def_log<<endl;
  }
} // WorldServer::InsertModel
void WorldServer::InsertModel(string name, math::Pose position, 
    string joint_angles, string model_path)
{
  if (!currentWorld->GetModel(name))
  {
    sdf::SDFPtr model_sdf;
    model_sdf.reset(new sdf::SDF);
    sdf::init(model_sdf);
    sdf::readFile(model_path.c_str(), model_sdf);
    sdf::ElementPtr model_element = model_sdf->root->GetElement("model");
    math::Pose position_calibrate(
        math::Vector3(0, 0, -0.05), math::Quaternion(0, 0, 0));
    model_element->GetAttribute("name")->Set(name);
    model_element->GetElement("pose")->Set(position_calibrate);
    currentWorld->InsertModelSDF(*model_sdf);
    initalJointValue.push_back(joint_angles);
    initialPosition.push_back(position);
  }else{
    Color::Modifier red_log(Color::FG_RED);
    Color::Modifier def_log(Color::FG_DEFAULT);
    cout<<red_log<<"WARNING: World: Insertion failed: module name exists."
        <<def_log<<endl;
  }
} // WorldServer::InsertModel
void WorldServer::AddInitialPosition(math::Pose position)
{
  initialPosition.push_back(position);
} // WorldServer::AddInitialPosition
void WorldServer::AddInitialJoints(string joint_angles)
{
  initalJointValue.push_back(joint_angles);
} // WorldServer::AddInitialJoints
void WorldServer::DeleteModule(string module_name)
{
  SmoresModulePtr currentModule = GetModulePtrByName(module_name);
  // ---------- Destroy all the edges -------------------
  if (currentModule->NodeFWPtr->Edge) {
    Disconnect(currentModule, 0);
  }
  if (currentModule->NodeLWPtr->Edge) {
    Disconnect(currentModule, 1);
  }
  if (currentModule->NodeRWPtr->Edge) {
    Disconnect(currentModule, 2);
  }
  if (currentModule->NodeUHPtr->Edge) {
    Disconnect(currentModule, 3);
  }
  // ---------- Destroy module in the module list -----
  for (unsigned int i = 0; i < moduleList.size(); ++i) {
    if (currentModule == moduleList.at(i)) {
      moduleList.at(i).reset();
      moduleList.erase(moduleList.begin()+i);
      break;
    }
  }
  currentWorld->GetModel(module_name)->Fini();
  needToSetPtr -= 1;
}
void WorldServer::PassiveConnect(SmoresModulePtr module_1, 
    SmoresModulePtr module_2, int node1_ID, int node2_ID, 
    double node_angle, double node_distance)
{
  if (!AlreadyConnected(module_1, module_2, node1_ID, node2_ID)) {
    SmoresEdgePtr new_connection(new SmoresEdge(module_1->GetNode(node1_ID),
        module_2->GetNode(node2_ID),node_distance,node_angle,
        module_1->GetNodeAxis(node1_ID),module_2->GetNodeAxis(node2_ID)));
    connectionEdges.push_back(new_connection);
    module_1->GetNode(node1_ID)->ConnectOnEdge(new_connection);
    module_2->GetNode(node2_ID)->ConnectOnEdge(new_connection);
    // Adding the dynamic joint after adding the new edge
    ConnectAndDynamicJointGeneration(module_1, module_2, node1_ID, node2_ID, 
        new_connection);
  }
} // WorldServer::PassiveConnect
void WorldServer::PassiveConnect(SmoresModulePtr module_1, 
    SmoresModulePtr module_2, int node1_ID, int node2_ID)
{
  PassiveConnect(module_1, module_2, node1_ID, node2_ID, 0, 0);
} // WorldServer::PassiveConnect
void WorldServer::ActiveConnect(SmoresModulePtr module_1, 
    SmoresModulePtr module_2, int node1_ID, int node2_ID, 
    double node_angle, double node_distance)
{
  if (!AlreadyConnected(module_1, module_2, node1_ID, node2_ID))
  {
    SmoresEdgePtr new_connection(new SmoresEdge(module_1->GetNode(node1_ID),
        module_2->GetNode(node2_ID),node_distance,node_angle,
        module_1->GetNodeAxis(node1_ID),module_2->GetNodeAxis(node2_ID)));
    connectionEdges.push_back(new_connection);
    module_1->GetNode(node1_ID)->ConnectOnEdge(new_connection);
    module_2->GetNode(node2_ID)->ConnectOnEdge(new_connection);
    // Adding the dynamic joint after adding the new edge
    ConnectAndDynamicJointGeneration(module_1, module_2, node1_ID, node2_ID,
        new_connection);
  }
} // WorldServer::ActiveConnect
void WorldServer::ActiveConnect(SmoresModulePtr module_1, 
    SmoresModulePtr module_2, int node1_ID, int node2_ID)
{
  ActiveConnect(module_1, module_2, node1_ID, node2_ID, 0, 0);
} // WorldServer::ActiveConnect
void WorldServer::Disconnect(SmoresEdgePtr edge)
{
  DynamicJointDestroy(edge);
  edge->model_1->Edge.reset();
  edge->model_2->Edge.reset();
  for (unsigned int i = 0; i < connectionEdges.size(); ++i) {
    if (connectionEdges.at(i)==edge) {
      connectionEdges.at(i).reset();
      connectionEdges.erase(connectionEdges.begin()+i);
      break;
    }
  }
} // WorldServer::Disconnect
void WorldServer::Disconnect(SmoresModulePtr module, int node_ID)
{
  SmoresEdgePtr edge = module->GetNode(node_ID)->GetEdge();
  this->Disconnect(edge);
} // WorldServer::Disconnect
void WorldServer::Disconnect(string module_name, int node_ID)
{
  SmoresEdgePtr edge = 
      GetModulePtrByName(module_name)->GetNode(node_ID)->GetEdge();
  this->Disconnect(edge);
} // WorldServer::Disconnect
void WorldServer::Disconnect(string module_name1, string module_name2)
{
  for (unsigned int i = 0; i < connectionEdges.size(); ++i) {
    string model1_in_edge = connectionEdges.at(i)->model_1->Parent->ModuleID;
    string model2_in_edge = connectionEdges.at(i)->model_2->Parent->ModuleID;
    if ((model1_in_edge.compare(module_name1)==0 
        && model2_in_edge.compare(module_name2)==0)
        || (model1_in_edge.compare(module_name2)==0
        && model2_in_edge.compare(module_name1)==0)) {
      DynamicJointDestroy(connectionEdges.at(i));
      connectionEdges.at(i)->model_1->Edge.reset();
      connectionEdges.at(i)->model_2->Edge.reset();
      connectionEdges.at(i).reset();
      connectionEdges.erase(connectionEdges.begin()+i);
      break;
    }
  }
} // WorldServer::Disconnect
void WorldServer::CommandManager(void)
{
  for (unsigned int i = 0; i < moduleCommandContainer.size(); ++i)
  {
    ModuleCommandsPtr current_command_container
        = moduleCommandContainer.at(i);
    // Time based gait-table checking
    if (current_command_container->CommandSquence.at(0).TimeBased) {
      if (current_command_container->CommandSquence.at(0).TimeInterval == 0) {
        current_command_container->FinishedFlag = true;
      }else{
        current_command_container->FinishedFlag = false;
      }
    }
    // Command execution processure
    if (!current_command_container->FinishedFlag) {
      if (!current_command_container->ReceivedFlag) {
        if (current_command_container->CommandSquence.at(0)
            .ConditionOnOtherCommand) {
          // Condition checking
          if (CheckCondition(
              current_command_container->CommandSquence.at(0).Dependency)) {
            CommandExecution(current_command_container);
          }
        }else{
          CommandExecution(current_command_container);
        }
      }else{
        // Timer Count Down
        if (current_command_container->CommandSquence.at(0).TimeBased) {
          current_command_container->CommandSquence.at(0).TimeInterval -= 1;
        }
      }
    }else{
      if (current_command_container->CommandSquence.at(0).ConditionCommand){
        FinishOneConditionCommand(
            current_command_container->CommandSquence.at(0).ConditionID);
      }
      current_command_container->CommandSquence.erase(
          current_command_container->CommandSquence.begin());
      current_command_container->ReceivedFlag = false;
      current_command_container->FinishedFlag = false;
      // Send confirm message back to model
      command_message::msgs::CommandMessage finish_confirm_message;
      finish_confirm_message.set_messagetype(0);
      current_command_container->WhichModule->ModulePublisher
          ->Publish(finish_confirm_message);
      // Erase empty container
      if (current_command_container->CommandSquence.size() == 0) {
        cout<<"World: Erase the command sequence of "
            <<current_command_container->WhichModule->ModuleID<<endl;
        current_command_container->WhichModule->moduleCommandContainer.reset();
        moduleCommandContainer.erase(moduleCommandContainer.begin()+i);
      }
    }
  }
} // WorldServer::CommandManager
void WorldServer::CommandExecution(ModuleCommandsPtr current_command_container)
{
  if (current_command_container->CommandSquence.at(0).SpecialCommandFlag) {
    if (current_command_container->CommandSquence.at(0).Command.CommandType == 1)
    {
      ActiveConnect(current_command_container->WhichModule, GetModulePtrByName(
          current_command_container->CommandSquence.at(0).Command.Module2), 
          current_command_container->CommandSquence.at(0).Command.Node1, 
          current_command_container->CommandSquence.at(0).Command.Node2);
      current_command_container->ReceivedFlag = true;
      if (!current_command_container->CommandSquence.at(0).TimeBased) {
        current_command_container->FinishedFlag = true;
      }
    }
    if (current_command_container->CommandSquence.at(0).Command.CommandType == 2)
    {
      Disconnect(current_command_container->CommandSquence.at(0).Command.Module1, 
          current_command_container->CommandSquence.at(0).Command.Module2);
      current_command_container->ReceivedFlag = true;
      if (!current_command_container->CommandSquence.at(0).TimeBased) {
        current_command_container->FinishedFlag = true;
      }
    }
  }else{
    current_command_container->WhichModule->ModulePublisher->Publish(
        *(current_command_container->CommandSquence.at(0).ActualCommandMessage));
  }
  // cout<<"World: Model: "<<current_command_container->WhichModule->ModuleID
  //     <<": command sent"<<endl;
  // cout<<"World: Size of the message is "
  //     <<current_command_container->CommandSquence.size()<<endl;
  current_command_container->ExecutionFlag = true;
} // WorldServer::CommandExecution
void WorldServer::SendGaitTable(SmoresModulePtr module, const bool *flag, 
    const double *gait_value, int msg_type, unsigned int time_stamp, 
    string condition_str, string dependency_str) 
    // unit of time_stamp is millisecond
{
  CommandPtr command_message(new command_message::msgs::CommandMessage());
  command_message->set_messagetype(msg_type);
  // TODO: Need to get rid of priority entirely
  command_message->set_priority(0);
  for (int i = 0; i < 4; ++i) {
    command_message->add_jointgaittablestatus(flag[i]);
    command_message->add_jointgaittable(gait_value[i]);
  }
  // Here is a patch for instantly executed command
  // TODO: Need an elegant way to implement it
  if (time_stamp == 0) {
    time_stamp = 1;
  }
  CommandPro new_message(command_message,time_stamp);
  if (condition_str.size()>0) {
    new_message.SetCondition(condition_str);
    AddCondition(condition_str);
  }
  if (dependency_str.size()>0) {
    new_message.SetDependency(dependency_str);
  }
  if (!module->moduleCommandContainer) {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));  
    new_command_message->CommandSquence.push_back(new_message);
    moduleCommandContainer.push_back(new_command_message);
    module->moduleCommandContainer = new_command_message;
    cout<<"World: "<<module->ModuleID<<" : command length: "
        <<module->moduleCommandContainer->CommandSquence.size()<<endl;
  }else{
    module->moduleCommandContainer->CommandSquence.push_back(new_message);
    cout<<"World: "<<module->ModuleID<<" : command length: "
        <<module->moduleCommandContainer->CommandSquence.size()<<endl;
  }
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, const bool *flag, 
    const double *gait_value, int msg_type, unsigned int time_stamp) 
    // unit of time_stamp is millisecond
{
  SendGaitTable(module, flag, gait_value, msg_type, time_stamp, "", "");
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, const bool *flag, 
    const double *gait_value, int msg_type, string condition_str, 
    string dependency_str)
{
  CommandPtr command_message(new command_message::msgs::CommandMessage());
  command_message->set_messagetype(msg_type);
  command_message->set_priority(0);
  for (int i = 0; i < 4; ++i) {
    command_message->add_jointgaittablestatus(flag[i]);
    command_message->add_jointgaittable(gait_value[i]);
  }
  CommandPro new_message(command_message);
  if (condition_str.size()>0) {
    new_message.SetCondition(condition_str);
    AddCondition(condition_str);
  }
  if (dependency_str.size()>0) {
    new_message.SetDependency(dependency_str);
  }

  if (!module->moduleCommandContainer) {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));   
    new_command_message->CommandSquence.push_back(new_message);
    moduleCommandContainer.push_back(new_command_message);
    module->moduleCommandContainer = new_command_message;
    cout<<"World: "<<module->ModuleID<<" : command length: "
        <<module->moduleCommandContainer->CommandSquence.size()<<endl;
  }else{
    module->moduleCommandContainer->CommandSquence.push_back(new_message);
    cout<<"World: "<<module->ModuleID<<" : command length: "
        <<module->moduleCommandContainer->CommandSquence.size()<<endl;
  }
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, const bool *flag, 
    const double *gait_value, int msg_type)
{
  SendGaitTable(module, flag, gait_value, msg_type, "", "");
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, int joint_ID, 
    double gait_value, int msg_type, unsigned int time_stamp, 
    string condition_str, string dependency_str)
{
  bool flag[4] = {false,false,false,false};
  double gait_values[4] = {0,0,0,0};
  flag[joint_ID] = true;
  gait_values[joint_ID] = gait_value;
  SendGaitTable(module, flag, gait_values, msg_type, time_stamp, 
      condition_str, dependency_str);
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, int joint_ID, 
    double gait_value, int msg_type, unsigned int time_stamp)
{
  SendGaitTable(module, joint_ID, gait_value, msg_type, time_stamp, "", "");
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, int joint_ID, 
    double gait_value, int msg_type, string condition_str, string dependency_str)
{
  bool flag[4] = {false,false,false,false};
  double gait_values[4] = {0,0,0,0};
  flag[joint_ID] = true;
  gait_values[joint_ID] = gait_value;
  SendGaitTable(module, flag, gait_values, msg_type, 
      condition_str, dependency_str);
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, int joint_ID, 
    double gait_value, int msg_type)
{
  SendGaitTable(module, joint_ID, gait_value, msg_type, "", "");
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, 
    string module1, string module2, int node1, int node2, 
    int command_type, unsigned int time_stamp, 
    string condition_str, string dependency_str)
{
  CommandPro new_message;
  // Here is a patch for instantly executed command
  if (time_stamp == 0) {
    time_stamp = 1;
  }
  new_message.SetTimer(time_stamp);
  SpecialCommand special_command(command_type, module1, module2, node1, node2);
  new_message.SetSpecialCommand(special_command);
  // cout<<"World: Special Command: condition: "<<condition_str
  //     <<"; dependency: "<<dependency_str<<endl;
  if (condition_str.size()>0) {
    new_message.SetCondition(condition_str);
    AddCondition(condition_str);
  }
  if (dependency_str.size()>0) {
    new_message.SetDependency(dependency_str);
  }
  if (!module->moduleCommandContainer) {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));    
    new_command_message->CommandSquence.push_back(new_message);
    moduleCommandContainer.push_back(new_command_message);
    module->moduleCommandContainer = new_command_message;
  }else{
    module->moduleCommandContainer->CommandSquence.push_back(new_message);
  }
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, 
  string module1, string module2, int node1, int node2, 
  int command_type, unsigned int time_stamp)
{
  SendGaitTable(module, module1, module2, node1, node2, 
      command_type, time_stamp, "", "");
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, 
    string module1, string module2, int node1, int node2, 
    int command_type, string condition_str, string dependency_str)
{
  CommandPro new_message;
  SpecialCommand special_command(command_type, module1, module2, node1, node2);
  new_message.SetSpecialCommand(special_command);
  // cout<<"World: Special Command: condition: "<<condition_str
  //     <<"; dependency: "<<dependency_str<<endl;
  if (condition_str.size()>0) {
    new_message.SetCondition(condition_str);
    AddCondition(condition_str);
  }
  if (dependency_str.size()>0) {
    new_message.SetDependency(dependency_str);
  }
  if (!module->moduleCommandContainer) {
    ModuleCommandsPtr new_command_message(new ModuleCommands(module));    
    new_command_message->CommandSquence.push_back(new_message);
    moduleCommandContainer.push_back(new_command_message);
    module->moduleCommandContainer = new_command_message;
  }else{
    module->moduleCommandContainer->CommandSquence.push_back(new_message);
  }
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTable(SmoresModulePtr module, 
    string module1, string module2, int node1, int node2, int command_type)
{
  SendGaitTable(module, module1, module2, node1, node2, command_type, "", "");
} // WorldServer::SendGaitTable
void WorldServer::SendGaitTableInstance(SmoresModulePtr module, 
    const bool *flag, const double *gait_value, int msg_type)
{
  CommandPtr connection_message(new command_message::msgs::CommandMessage());
  connection_message->set_messagetype(msg_type);
  connection_message->set_priority(-1);
  for (int i = 0; i < 4; ++i) {
    connection_message->add_jointgaittablestatus(flag[i]);
    connection_message->add_jointgaittable(gait_value[i]);
  }
  module->ModulePublisher->Publish(*connection_message);
} // WorldServer::SendGaitTableInstance
void WorldServer::SendGaitTableInstance(SmoresModulePtr module, 
    const bool *flag, const double *gait_value)
{
  SendGaitTableInstance(module, flag, gait_value, 4);
} // WorldServer::SendGaitTableInstance
void WorldServer::SendPositionInstance(SmoresModulePtr module, 
  double x, double y, double orientation_angle)
{
  CommandPtr connection_message(new command_message::msgs::CommandMessage());
  connection_message->set_messagetype(2);
  connection_message->set_priority(-1);
  connection_message->mutable_positionneedtobe()->mutable_position()->set_x(x);
  connection_message->mutable_positionneedtobe()->mutable_position()->set_y(y);
  connection_message->mutable_positionneedtobe()->mutable_position()
      ->set_z(orientation_angle);
  connection_message->mutable_positionneedtobe()->mutable_orientation()->set_x(0);
  connection_message->mutable_positionneedtobe()->mutable_orientation()->set_y(0);
  connection_message->mutable_positionneedtobe()->mutable_orientation()->set_z(0);
  connection_message->mutable_positionneedtobe()->mutable_orientation()->set_w(0);
  module->ModulePublisher->Publish(*connection_message);
} // WorldServer::SendPositionInstance
// TODO: Need to be tested
void WorldServer::EraseComaands(SmoresModulePtr module) 
{
  if (module->moduleCommandContainer) {
    EraseCommandPtrByModule(module);
    module->moduleCommandContainer.reset();
  }
} // WorldServer::EraseComaands
int WorldServer::GetNodeIDByName(string node_name)
{
  if (node_name.rfind("FrontWheel")!=string::npos) {
    return 0;
  }
  if (node_name.rfind("LeftWheel")!=string::npos) {
    return 1;
  }
  if (node_name.rfind("RightWheel")!=string::npos) {
    return 2;
  }
  if (node_name.rfind("UHolderBody")!=string::npos) {
    return 3;
  }
  Color::Modifier red_log(Color::FG_RED);
  Color::Modifier def_log(Color::FG_DEFAULT);
  cout<<red_log<<"WARNING: World: Wrong node number"<<def_log<<endl;
  return 4; // When return 4, then there is no match found
} // WorldServer::GetNodeIDByName
SmoresModulePtr WorldServer::GetModulePtrByName(string module_name)
{
  SmoresModulePtr exist_module;
  for (unsigned int i = 0; i < moduleList.size(); ++i) {
    if (module_name.compare(moduleList.at(i)->ModuleID)==0) {
      exist_module = moduleList.at(i);
      break;
    }
  }
  return exist_module;
} // WorldServer::GetModulePtrByName
SmoresModulePtr WorldServer::GetModulePtrByIDX(unsigned int idx)
{
  SmoresModulePtr exist_module;
  if (idx < moduleList.size()) {
    exist_module = moduleList.at(idx);
  }
  return exist_module;
} // WorldServer::GetModulePtrByIDX
unsigned int WorldServer::GetModuleListSize(void)
{
  return moduleList.size();
} // WorldServer::GetModuleListSize
void WorldServer::EraseCommandPtrByModule(SmoresModulePtr module_ptr)
{
  for (unsigned int i = 0; i < moduleCommandContainer.size(); ++i) {
    if (module_ptr == moduleCommandContainer.at(i)->WhichModule) {
      moduleCommandContainer.erase(moduleCommandContainer.begin()+i);
      break;
    }
  }
} // WorldServer::EraseCommandPtrByModule
int WorldServer::GetModuleIndexByName(string module_name)
{
  int exist_module = -1;
  for (unsigned int i = 0; i < moduleList.size(); ++i) {
    if (module_name.compare(moduleList.at(i)->ModuleID)==0) {
      exist_module = i;
      break;
    }
  }
  return exist_module;
} // WorldServer::GetModuleIndexByName
bool WorldServer::AlreadyConnected(SmoresModulePtr module_1, 
    SmoresModulePtr module_2, int node1_ID, int node2_ID)
{
  bool having_a_connection;
  if((bool)module_1->GetNode(node1_ID)->Edge 
      && (bool)module_2->GetNode(node2_ID)->Edge) {
    having_a_connection = true;
  }else{
    having_a_connection = false;
  }
  return having_a_connection;
} // WorldServer::AlreadyConnected
bool WorldServer::AlreadyConnected(SmoresModulePtr module_1, 
    SmoresModulePtr module_2)
{
  bool having_a_connection = false;
  for (int j = 0; j < 4; ++j) {
    if (module_1->GetNode(j)->Edge) {
      SmoresModulePtr connected_module = module_1->GetNode(j)->Edge
          ->FindMatchingNode(module_1->GetNode(j))->Parent;
      if (connected_module == module_2) {
        having_a_connection = true;
        break;
      }
    }
  }
  return having_a_connection;
} // WorldServer::AlreadyConnected
bool WorldServer::AlreadyConnected(SmoresModulePtr module, int node_ID)
{
  bool having_a_connection = false;
  if (module->GetNode(node_ID)->Edge) {
    having_a_connection = true;
  }
  return having_a_connection;
} // WorldServer::AlreadyConnected
// A replacement of the old function, a BFS (Breath First Search)
// TODO: Need to be tested
unsigned int WorldServer::CountModules(SmoresModulePtr module)
{
  std::queue<SmoresModulePtr> frontier;
  vector<SmoresModulePtr> cluster;
  frontier.push(module);
  while (frontier.size()>0) {
    if (std::find(cluster.begin(), cluster.end(), frontier.front())
        ==cluster.end()) {
      cluster.push_back(frontier.front());
      for (int i = 0; i < 4; ++i) {
        if (frontier.front()->GetNode(i)->Edge) {
          SmoresModulePtr connected_module = frontier.front()->GetNode(i)->Edge
              ->FindMatchingNode(frontier.front()->GetNode(i))->Parent;
          frontier.push(connected_module);
        }
      }
    }
    frontier.pop();
  }
  return cluster.size();
} // WorldServer::CountModules
unsigned int WorldServer::GetInitialJointSequenceSize(void)
{
  return initalJointValue.size();
} // WorldServer::GetInitialJointSequenceSize
void WorldServer::ReadFileAndGenerateCommands(const char* fileName)
{
  ifstream infile;
  infile.open(fileName);
  string output;
  string candidate_str;
  if (infile.is_open()) {
    while (!infile.eof()) {
      string a_line;
      getline(infile,a_line);
      if (a_line.find("//") != string::npos) {
        a_line = a_line.substr(0,a_line.find("//"));
      }
      if (a_line.size()>0) {
        while(a_line.find_first_of(";") != string::npos) {
          string candidate_str = a_line.substr(0, a_line.find_first_of(";"));
          a_line = a_line.substr(a_line.find_first_of(";")+1);
          if (candidate_str.find_first_of("$") != string::npos) {
            candidate_str = candidate_str.substr(
                candidate_str.find_first_of("$")+1);
            InterpretSpecialString(candidate_str);
          }else{
            InterpretCommonGaitString(candidate_str);
          }
        }
      }else{
        continue;
      }
    }
  }
} // WorldServer::ReadFileAndGenerateCommands
void WorldServer::InterpretCommonGaitString(string a_command_str)
{
  // Color::Modifier yellow_log(Color::FG_YELLOW,true);
  // Color::Modifier def_log(Color::FG_DEFAULT);
  // cout<<yellow_log<<"World: Command str: "<<a_command_str<<def_log<<";"<<endl;
  int time_interval = StripOffTimerInCommandString(a_command_str);
  string condition = StripOffCondition(a_command_str);
  string dependency = StripOffDependency(a_command_str);
  istringstream str_stream(a_command_str);
  vector<string> tokens;
  copy(istream_iterator<string>(str_stream),
     istream_iterator<string>(),
     back_inserter<vector<string> >(tokens));
  string model_name = tokens.at(0);
  // cout<<"World: Model name: "<<model_name<<";"<<endl;
  // cout<<"World: The command string is: "<<a_command_str<<endl;
  bool flags[4] = {false, false, false, false};
  double joints_values[4] = {0, 0, 0, 0};
  vector<string> joint_specs;
  for (int i = 1; i < 5; ++i) {
    joint_specs.push_back(tokens.at(i));
  }
  FigureInterpret(&joint_specs, flags, joints_values);
  if (time_interval >=0 ) {
    SendGaitTable(GetModulePtrByName(model_name), flags, joints_values, 
        3, time_interval, condition, dependency);
  }else{
    SendGaitTable(GetModulePtrByName(model_name), flags, joints_values, 
        3, condition, dependency);
  }
} // WorldServer::InterpretCommonGaitString
void WorldServer::InterpretSpecialString(string a_command_str)
{
  // Color::Modifier green_log(Color::FG_GREEN);
  // Color::Modifier def_log(Color::FG_DEFAULT);
  // cout<<green_log<<"World: Special command str: "
  //     <<a_command_str<<def_log<<";"<<endl;
  int time_interval = StripOffTimerInCommandString(a_command_str);
  string condition = StripOffCondition(a_command_str);
  string dependency = StripOffDependency(a_command_str);
  // Find all the Module names
  vector<string> module_names;
  a_command_str += " ";
  while(a_command_str.find_first_of('&') != string::npos)
  {
    unsigned int symbol_pos = a_command_str.find_first_of('&');
    unsigned int space_pos = a_command_str.find_first_of(' ', symbol_pos);
    module_names.push_back(a_command_str.substr(symbol_pos+1,
        space_pos-symbol_pos-1));
    a_command_str = a_command_str.substr(0,symbol_pos)
        +a_command_str.substr(space_pos+1);
  }
  // Find all the node number
  vector<int> node_ids;
  while(a_command_str.find_first_of('#') != string::npos)
  {
    unsigned int symbol_pos = a_command_str.find_first_of('#');
    unsigned int space_pos = a_command_str.find_first_of(' ', symbol_pos);
    node_ids.push_back(atoi(a_command_str.substr(symbol_pos+1,
        space_pos-symbol_pos-1).c_str()));
    a_command_str = a_command_str.substr(0,symbol_pos)
        +a_command_str.substr(space_pos+1);
  }

  istringstream str_stream(a_command_str);
  vector<string> tokens;
  copy(istream_iterator<string>(str_stream),
     istream_iterator<string>(),
     back_inserter<vector<string> >(tokens));

  int command_type = 0;
  if (tokens.at(0).at(0) == '+')
    command_type = 1;
  if (tokens.at(0).at(0) == '-')
    command_type = 2;
  // TODO: This is not an elegant solution
  while (node_ids.size()<2) {
    node_ids.push_back(0);
  }
  if (time_interval >=0 ) {
    SendGaitTable(GetModulePtrByName(module_names.at(0)), module_names.at(0), 
        module_names.at(1), node_ids.at(0), node_ids.at(1), command_type, 
        time_interval, condition, dependency);
  }else{
    SendGaitTable(GetModulePtrByName(module_names.at(0)), module_names.at(0), 
        module_names.at(1), node_ids.at(0), node_ids.at(1), command_type, 
        condition, dependency);
  }
} // WorldServer::InterpretSpecialString
/*! TODO: This is at a very low API level of SGST
          need to implement all the features in the future
*/
void WorldServer::FigureInterpret(const vector<string> *joints_spec, 
    bool *type_flags,  double *joint_values)
{
  for (unsigned int i = 0; i < 4; ++i)
  {
    string mini_unit =  joints_spec->at(i);
    if (mini_unit.at(0) == 'p')
      type_flags[i] = true;
    if (mini_unit.at(0) == 's')
      type_flags[i] = false;
    if (mini_unit.at(0) == 't')
      type_flags[i] = false;
    if (mini_unit.at(0) == 'i')
      type_flags[i] = false;
    if (mini_unit.at(0) == 'c')
      type_flags[i] = false;
    if (mini_unit.at(0) == 'd')
      type_flags[i] = false;
    if (mini_unit.substr(1).size()>0)
      joint_values[i] = atof(mini_unit.substr(1).c_str());
  }
} // WorldServer::FigureInterpret
int WorldServer::StripOffTimerInCommandString(string &command_string)
{
  if (command_string.find("[") != string::npos) {
    unsigned int left_brace_pos = command_string.find("[");
    unsigned int right_brace_pos = command_string.find("]");
    string time_str = command_string.substr(left_brace_pos+1,
        right_brace_pos-left_brace_pos-1);
    command_string = command_string.substr(0,left_brace_pos-1)
        + command_string.substr(right_brace_pos+1);
    return atoi(time_str.c_str());
  }
  return -1;
} // WorldServer::StripOffTimerInCommandString
string WorldServer::StripOffCondition(string &command_string)
{
  if (command_string.find("{") != string::npos) {
    unsigned int left_brace_pos = command_string.find("{");
    unsigned int right_brace_pos = command_string.find("}");
    string condition = command_string.substr(left_brace_pos+1,
        right_brace_pos-left_brace_pos-1);
    command_string = command_string.substr(0,left_brace_pos-1)
        + command_string.substr(right_brace_pos+1);
    return condition;
  }
  return "";
} // WorldServer::StripOffCondition
string WorldServer::StripOffDependency(string &command_string)
{
  if (command_string.find("(") != string::npos) {
    unsigned int left_brace_pos = command_string.find("(");
    unsigned int right_brace_pos = command_string.find(")");
    string dependency = command_string.substr(left_brace_pos+1,
        right_brace_pos-left_brace_pos-1);
    command_string = command_string.substr(0,left_brace_pos-1)
        + command_string.substr(right_brace_pos+1);
    return dependency;
  }
  return "";
} // WorldServer::StripOffDependency
void WorldServer::AddCondition(string condition_id)
{
  bool not_exist = true;
  for (unsigned int i = 0; i < commandConditions.size(); ++i) {
    if (commandConditions.at(i)->condition_id.compare(condition_id) == 0) {
      commandConditions.at(i)->total_count += 1;
      not_exist = false;
      break;
    }
  }
  if (not_exist) {
    ConditionPtr new_condition(new Condition(condition_id));
    commandConditions.push_back(new_condition);
  }
} // WorldServer::AddCondition
void WorldServer::FinishOneConditionCommand(string condition_id)
{
  for (unsigned int i = 0; i < commandConditions.size(); ++i) {
    if (commandConditions.at(i)->condition_id.compare(condition_id) == 0) {
      commandConditions.at(i)->finished_count += 1;
      if (commandConditions.at(i)->finished_count 
          >= commandConditions.at(i)->total_count) {
        commandConditions.at(i)->achieved = true;
        cout<<"World: condition: "<<commandConditions.at(i)->condition_id
            <<" :achieved"<<endl;
      }
      break;
    }
  }
} // WorldServer::FinishOneConditionCommand
bool WorldServer::CheckCondition(string condition_id)
{
  for (unsigned int i = 0; i < commandConditions.size(); ++i) {
    if (commandConditions.at(i)->condition_id.compare(condition_id) == 0) {
      return commandConditions.at(i)->achieved;
    }
  }
  return true;
} // WorldServer::CheckCondition
SmoresEdgePtr WorldServer::GetEdgePtrByIDX(unsigned int idx)
{
  return connectionEdges.at(idx);
} // WorldServer::GetEdgePtrByIDX
unsigned int WorldServer::GetEdgeCounts(void)
{
  return connectionEdges.size();
} // WorldServer::GetEdgeCounts
void WorldServer::ReBuildDynamicJoint(SmoresEdgePtr a_edge)
{
  ConnectAndDynamicJointGeneration(
        a_edge->model_1->Parent, a_edge->model_2->Parent, 
        a_edge->model_1->NodeID, a_edge->model_2->NodeID, 
        a_edge);
} // WorldServer::ReBuildDynamicJoint
} // namespace gazebo