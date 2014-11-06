#ifndef _GAZEBO_CONDITION_HH_
#define _GAZEBO_CONDITION_HH_
//! \file Define Condition class used in WorldPlugin
#include <string>

using std::string;

namespace gazebo
{
/// A class used to store the condition information and mangement the work flow
class Condition
{
 public:
 	/// Constructor
 	/*!
 		\param condition_ID The unique condition id
 	*/
  Condition(string condition_ID);
  /// Destructor
  ~Condition();
 public:
 	/// Condition ID
  string condition_id;
  /// The total number of gait entries (commands) that has this trigger information
  int total_count;
  /// The number of commands that has been finished which has this trigger
  int finished_count;
  /// When finished_count euqals to total_count, achieved will be set to True
  /*! 
  	When this is true, all the commands depend on the accomplishment of this 
  	condition will be triggered
  */
  bool achieved;
}; // class Condition
} // namespace gazebo
#endif