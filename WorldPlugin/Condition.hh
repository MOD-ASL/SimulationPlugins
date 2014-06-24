#ifndef _GAZEBO_CONDITION_HH_
#define _GAZEBO_CONDITION_HH_

#include <string>

using std::string;

namespace gazebo
{
class Condition
{
 public:
  Condition(string condition_ID);
  ~Condition();
 public:
  string condition_id;
  int total_count;
  int finished_count;
  bool achieved;
}; // class Condition
} // namespace gazebo
#endif