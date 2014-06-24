#include "Condition.hh"

using std::string;

namespace gazebo{
Condition::Condition(string condition_ID)
{
  this->condition_id = condition_ID;
  this->total_count = 1;
  this->finished_count = 0;
  this->achieved = false;
} // Condition::Condition
Condition::~Condition(){}
}