#include "CollisionInformation.hh"

using std::string;

namespace gazebo
{
CollisionInformation::CollisionInformation(string collision1, 
    string collision2, string link_collision1, string link_collision2)
{
  Model1 = collision1;
  Model2 = collision2;
  LinkOfModel1 = link_collision1;
  LinkofModel2 = link_collision2;
} // CollisionInformation::CollisionInformation
CollisionInformation::~CollisionInformation(){}
bool CollisionInformation::SameCollision(string collision1, 
    string collision2, string link_collision1, string link_collision2)
{
  bool same_collision = false;
  if (collision1.compare(Model1)==0 && collision2.compare(Model2)==0){
    if (link_collision1.compare(LinkOfModel1)==0 
        && link_collision2.compare(LinkofModel2)==0){
      same_collision = true;
    }
  }else{
    if (collision1.compare(Model2)==0 && collision2.compare(Model1)==0){
      if (link_collision1.compare(LinkofModel2)==0 
          && link_collision2.compare(LinkOfModel1)==0){
        same_collision = true;
      }
    }
  }
  return same_collision;
} // CollisionInformation::SameCollision
} // namespace gazebo