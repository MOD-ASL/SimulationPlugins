#include "PaintingPlugin.hh"

using namespace std;
using namespace octomap;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(PaintingPlugin)

PaintingPlugin::~PaintingPlugin()
{
  tree.writeBinary("LIDARMap.bt");
  std::cout << "\nnooooo!!!\n";
}

PaintingPlugin::PaintingPlugin() : tree(0.01)
{
    std::cout << "yerp??\n";
}


void PaintingPlugin::lidarCB(ConstLaserScanStampedPtr& _msg)
{
  for (int i = 0; i < _msg->scan().ranges_size(); i++)
  {
    if (_msg->scan().ranges(i) < 1)
    {
      double range = _msg->scan().ranges(i);
      double angle = _msg->scan().angle_min() + i*_msg->scan().angle_step();
      
      gazebo::math::Pose scanPose(range*cos(angle), range*sin(angle), 0, 0, 0, 0);
      
      gazebo::math::Pose targetPose = scanPose + gazebo::msgs::Convert(_msg->scan().world_pose());
      
      double x = targetPose.pos.x;
      double y = targetPose.pos.y;
      double z = targetPose.pos.z;
      
      if (z < 0.002)
      {
	break;
      }
      
      std::vector<gazebo::math::Pose>::const_iterator poseIndex;
      
      bool smoreHit = false;
      
      for (poseIndex = smorePose.begin(); poseIndex != smorePose.end(); poseIndex++)
      {
	gazebo::math::Pose centroid(0, 0, 0.05, 0, 0, 0);
	gazebo::math::Pose smoreCenter = centroid + *poseIndex;
	/*std::cout << fabs((smoreCenter).pos.x - x) << "\n";
	std::cout << fabs((smoreCenter).pos.y - y) << "\n";
	std::cout << fabs(((smoreCenter).pos.z+0.05) - z) << "\n\n";*/
	if (fabs(smoreCenter.pos.x - x) <= 0.07 && fabs(smoreCenter.pos.y - y) <= 0.07 && fabs(smoreCenter.pos.z - z) <= 0.07)
	{
	  smoreHit = true;
	  break;
	}
      }
      
      if (!smoreHit)
      {
	/*int i = (int) round(100*x + 500);
	int j = (int) round(100*y + 500);
	int k = (int) round(100*z + 500);*/
	
	point3d endpoint ((float) x, (float) y, (float) z);
	
	if (tree.search(endpoint) == NULL)
	{
	  line->AddPoint(math::Vector3(x, y, z));
	  //occupancy[i][j][k] = true;
	  this->infoScore++;
	  printf("Info Score: %d\n", this->infoScore);
	  msgs::Int score;
	  score.set_data(this->infoScore);
	  this->scorePub->Publish(score);	  
	  tree.updateNode(endpoint, true);
	}
	else
	{
	  /*std::cout << "It happens to be occupied already\n";
	  if (tree.search(endpoint) != NULL)
	  {
	    std::cout << "Happens to be OCTOPIED as well!!\n";
	  }*/
	}
      }
      else
      {
	std::cout << "WOAH THERE!!\n";
      }
    }
  }
}

void PaintingPlugin::poseCB(ConstPosesStampedPtr& _msg)
{
  smoreIndex.clear();
  smorePose.clear();
  
  for (int i = 0; i < _msg->pose_size(); i++)
  {
    if ((_msg->pose(i).name().substr(0, 6) == "Module" || _msg->pose(i).name().substr(0, 6) == "SMORES") && _msg->pose(i).name().find("::") == std::string::npos)
    {
      //std::cout << _msg->pose(i).name() << " at " << i << "\n";
      std::string smoreName = _msg->pose(i).name();
      //std::string lidarName = smoreName + "::SMORES_LIDAR::model";
      
      smoreIndex.push_back(i);
      smorePose.push_back(gazebo::msgs::Convert(_msg->pose(i)));
    }
    if (_msg->pose(i).name().find("::SMORES_LIDAR::model") != std::string::npos)
    {
      std::string lidarName = _msg->pose(i).name();
      
      size_t index = lidarName.find("::");
      while(index != std::string::npos)
      {
	lidarName.replace(index, 2, "/");
	index = lidarName.find("::");
      }
      lidarTopic.push_back("/gazebo/default/" + lidarName + "/range_sensor/scan");
    }
  }
}

void PaintingPlugin::paintCB(ConstVector3dPtr& _msg)
{
  math::Vector3 targetPoint(_msg->x(), _msg->y(), _msg->z());
  this->line->AddPoint(targetPoint);
}

void PaintingPlugin::print_query_info(point3d query, OcTreeNode* node) {
  if (node != NULL) {
    cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << endl;
  }
  else 
    cout << "occupancy probability at " << query << ":\t is unknown" << endl;    
}


void PaintingPlugin::Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    cout << endl;
    cout << "generating example map" << endl;

    std::cout << "yerp\n\n";
    
    this->infoScore = 0;
 
    this->model = _parent->GetRootVisual()->GetParent();
    
    // Listen to the update event. This event is broadcast pre-render update???
    this->updateConnection = event::Events::ConnectPreRender(boost::bind(&PaintingPlugin::OnUpdate, this));
    this->line = this->model->CreateDynamicLine(rendering::RENDERING_POINT_LIST);
    
    lidarTopic.clear();
  
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init();
    
    this->poseSub = this->node->Subscribe<gazebo::msgs::PosesStamped>("/gazebo/default/pose/info", &PaintingPlugin::poseCB, this);
    this->scorePub = this->node->Advertise<msgs::Int>("/gazebo/default/SensingScore");
    
    std::cout << "subscribed??\n\n";
    
    this->line->setMaterial("Gazebo/Red");
    this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
    this->model->SetVisible(true);
}
// Called by the world update start event
void PaintingPlugin::OnUpdate()
{
  if (lidarTopic.size() > 0 && this->listener.size() == 0)
  {
    for(std::vector<std::string>::const_iterator lidarIndex = lidarTopic.begin(); lidarIndex < lidarTopic.end(); lidarIndex++)
    {
      std::cout << *lidarIndex << "\n";
      listener.push_back(node->Subscribe<gazebo::msgs::LaserScanStamped>(*lidarIndex, &PaintingPlugin::lidarCB, this));
    }
  }
  else
  {
    //std::cout << "Hrrm\n";
  }
}
