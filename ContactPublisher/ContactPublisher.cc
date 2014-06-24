#include "ContactPublisher.hh"
// Declaration of the objects used from standard libraries
using std::string;
using std::cout;

namespace gazebo{
ContactSensor::ContactSensor() : SensorPlugin()
{
} // ContactSensor::ContactSensor
ContactSensor::~ContactSensor()
{ 
  parentSensor.reset();
  updateConnection.reset();
  collisionPub.reset();
} // ContactSensor::~ContactSensor
void ContactSensor::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor. This code from gazebo wiki.
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
  // Create a new transport node
  transport::NodePtr node(new transport::Node());
  // Get the parent model name
  string parent_model_name = 
    parentSensor->GetParentName()
    .substr(0,(parentSensor->GetParentName()).find("::"));
  // Initialize the node with the model name
  node->Init(parent_model_name);
  // Setting up the publisher
  string topic_name = 
    "~/" + parentSensor->GetParentName() + "::" + parentSensor->GetName();
  this->collisionPub = node->Advertise<msgs::GzString>(topic_name);
} // ContactSensor::Load
void ContactSensor::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts = this->parentSensor->GetContacts();
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    if (contacts.contact(i).collision1().find("ground_plane") == string::npos 
        && contacts.contact(i).collision2().find("ground_plane") == string::npos)
        // Get rid of the collision with ground plane
    {
      // Publish the topic to model
      string collision_msgs = 
          contacts.contact(i).collision1() + "," + contacts.contact(i).collision2();
      msgs::GzString gzstr_collision_message;
      gzstr_collision_message.set_data(collision_msgs);
      collisionPub->Publish(gzstr_collision_message);

      //+++++++++++++++ Showing the position of the contact points +++++++++++++++
      // for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
      // {
      //   cout << j << "  Position:"
      //        << contacts.contact(i).position(j).x() << " "
      //        << contacts.contact(i).position(j).y() << " "
      //        << contacts.contact(i).position(j).z() << "\n";
      //   cout << "   Normal:"
      //        << contacts.contact(i).normal(j).x() << " "
      //        << contacts.contact(i).normal(j).y() << " "
      //        << contacts.contact(i).normal(j).z() << "\n";
      //   cout << "   Depth:" << contacts.contact(i).depth(j) << "\n";
      // }
      //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      break;
    }
  }
} // ContactSensor::OnUpdate

GZ_REGISTER_SENSOR_PLUGIN(ContactSensor)
} // namespace gazebo 