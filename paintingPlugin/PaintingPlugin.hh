#ifndef PAINTING_PLUGIN_H
#define PAINTING_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>

#include <gazebo/common/common.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "gazebo/rendering/DynamicLines.hh"
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>

#include <gazebo/msgs/geometry.pb.h>
#include <gazebo/msgs/model_configuration.pb.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <cmath>


namespace gazebo
{
class PaintingPlugin : public VisualPlugin
{
private:
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
    // Called by the world update start event
    void OnUpdate();
    
    void lidarCB(ConstLaserScanStampedPtr &_msg);
    
    void poseCB(ConstPosesStampedPtr &_msg);
    
    void paintCB(ConstVector3dPtr &_msg);

private:
    // Pointer to the model
    rendering::VisualPtr model;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
    
    std::string moduleName;
    
    rendering::DynamicLines *line;
    
    transport::NodePtr node;
    transport::SubscriberPtr lidarSub;
    transport::SubscriberPtr poseSub;
    transport::SubscriberPtr paintSub;
    gazebo::transport::PublisherPtr scorePub;
    
    std::vector<gazebo::transport::SubscriberPtr> listener;
    
    math::Pose lidarPose;
    
    std::vector<int> smoreIndex;
    std::vector<math::Pose> smorePose;
    std::vector<std::string> lidarTopic;
    
    int infoScore;
    
    bool occupancy[2000][2000][1000];
};
}

#endif // PAINTING_PLUGIN_H