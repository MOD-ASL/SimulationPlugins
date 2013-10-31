//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Filename:  ContactSensor.cc
// Author:  Edward Cui
// Contact: cyk1990995@gmail.com
// Last updated:  10/24/2013
// Description: This sensor pluginin is used to populate message when
//              two models are close enough to connect, this messgae 
//              will eventually pass to the world plugin, the system 
//              monitor.
// Commit info: git@github.com:princeedward/SimulationPlugins.git
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include "ContactSensor.hh"

using namespace gazebo;
using namespace std;
GZ_REGISTER_SENSOR_PLUGIN(ContactSensor)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contact sensor plugin constructor
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ContactSensor::ContactSensor() : SensorPlugin()
{
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contact sensor plugin deconstructor
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ContactSensor::~ContactSensor()
{ 
}   

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The function will be called when the model loaded in the world
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ContactSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{   
  // Get the parent sensor.
  this->parentSensor =
    boost::shared_dynamic_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  { 
    gzerr << "ContactSensor requires a ContactSensor.\n";
    return;
  } 

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      boost::bind(&ContactSensor::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  //+++++++++++++ Generate a sensor topic with name of sensor's name
  // and its parent name +++++++++++++++++++++++++++++++++++++++++++

  // Create a new transport node
  transport::NodePtr node(new transport::Node());

  // Get the parent model name
  string ParentModelName = (parentSensor->GetParentName()).substr(0,(parentSensor->GetParentName()).find("::"));
  // cout<<"Node name is "<<ParentModelName<<endl;

  // Initialize the node with the model name
  node->Init(ParentModelName);
  // cout<<"Sensor: node name is '"<<ParentModelName<<"'"<<endl;
  string TopicName = "~/" + parentSensor->GetParentName() + "::" + parentSensor->GetName();
  this->CollisionPub = node->Advertise<msgs::GzString>(TopicName);
  // cout<<"Sensor: node topic is '"<<TopicName<<"'"<<endl;
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// The function will be called everytime the sensor has been updated
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ContactSensor::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->GetContacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    if (contacts.contact(i).collision1().find("ground_plane")==string::npos && contacts.contact(i).collision2().find("ground_plane")==string::npos)
    {
      // std::cout << "Collision between[" << contacts.contact(i).collision1()
      //         << "] and [" << contacts.contact(i).collision2() << "]\n";

      //+++++++++++++++ Publish the topic to model +++++++++++++++++++++++++++++++++++++++
      msgs::GzString CollisionMessage;
      string CollisionMsgs = contacts.contact(i).collision1() + "," + contacts.contact(i).collision2();
      CollisionMessage.set_data(CollisionMsgs);
      CollisionPub->Publish(CollisionMessage);
      // cout<<"Topic Published"<<endl;
      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

      //+++++++++++++++ Showing the position of the contact points +++++++++++++++++++++++
      // for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
      // {
      //   std::cout << j << "  Position:"
      //             << contacts.contact(i).position(j).x() << " "
      //             << contacts.contact(i).position(j).y() << " "
      //             << contacts.contact(i).position(j).z() << "\n";
      //   std::cout << "   Normal:"
      //             << contacts.contact(i).normal(j).x() << " "
      //             << contacts.contact(i).normal(j).y() << " "
      //             << contacts.contact(i).normal(j).z() << "\n";
      //   std::cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      // }
      //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      break;
    }
  }
}