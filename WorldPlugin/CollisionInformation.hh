#ifndef _GAZEBO_COLLISION_INFORMATION_HH_
#define _GAZEBO_COLLISION_INFORMATION_HH_

#include <string>

using std::string;

namespace gazebo
{
class CollisionInformation
{
 public:
  CollisionInformation(string collision1, string collision2, 
      string link_collision1, string link_collision2);
  ~CollisionInformation();
  // Check whether the collision stored in two objects are the same
  bool SameCollision(string collision1, string collision2, 
      string link_collision1, string link_collision2);
 public:
  string Model1;
  string Model2;
  string LinkOfModel1;
  string LinkofModel2;
}; // class CollisionInformation
} // namespace gazebo
#endif