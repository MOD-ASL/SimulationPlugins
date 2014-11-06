#ifndef _GAZEBO_COLLISION_INFORMATION_HH_
#define _GAZEBO_COLLISION_INFORMATION_HH_
//! \file Defines CollisionInformation class used in WorldPlugin
#include <string>

using std::string;

namespace gazebo
{
/// A class used to store collision information for two models
/*!
  This class is mainly used in magnetic connection of two contact faces
*/
class CollisionInformation
{
 public:
  /// Constructor
  /*!
    \param collision1 Name of one of the collided models
    \param collision2 Name of the other collided models
    \param link_collision1 Name of the link of the first model
    \param link_collision2 Name of the link of the second model
  */
  CollisionInformation(string collision1, string collision2, 
      string link_collision1, string link_collision2);
  /// Destructor
  ~CollisionInformation();
  /// Check whether the collision stored in two objects are the same
  /*!
    Because the model name stored in the object have no order, so this function
    is useful to check whether two objects are the same
    \param collision1 Name of one of the collided models
    \param collision2 Name of the other collided models
    \param link_collision1 Name of the link of the first model
    \param link_collision2 Name of the link of the second model
    \return If they are the same, return true, else return false
  */
  bool SameCollision(string collision1, string collision2, 
      string link_collision1, string link_collision2);
 public:
  /// Name of the first model
  string Model1;
  /// Name of the second model
  string Model2;
  /// Name of the collided link of the first model
  string LinkOfModel1;
  /// Name of the collided link of the second model
  string LinkofModel2;
}; // class CollisionInformation
} // namespace gazebo
#endif