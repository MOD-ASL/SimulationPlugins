//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author: Edward Yunkai Cui
// Description: Those classes defined in this file are used by the 
//              command manager in the world plugin
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef _GAZEBO_COMMAND_MANAGEMENT_HH_
#define _GAZEBO_COMMAND_MANAGEMENT_HH_

#include <boost/shared_ptr.hpp>

#include "command_message.pb.h"

using std::string;
using std::vector;

typedef boost::shared_ptr<command_message::msgs::CommandMessage> CommandPtr;

namespace gazebo{
class SmoresModule;
typedef boost::shared_ptr<SmoresModule> SmoresModulePtr;
// A data structure that stores command message
class SpecialCommand
{
 public:
  SpecialCommand(int commandtype, string modelname1, string modelname2, 
      int node1, int node2);
  SpecialCommand(int commandtype); 
  SpecialCommand();
  ~SpecialCommand();
  // CommandType: 0 no special command; 1 connect; 2 disconnect
  int CommandType;
  string Module1;
  string Module2;
  int Node1;
  int Node2;
}; // class SpecialCommand
class CommandPro  // abbr for Command protocol
{
 public:
  CommandPro();
  CommandPro(CommandPtr command);
  CommandPro(CommandPtr command, unsigned int timer);
  ~CommandPro();
  void SetTimer(unsigned int timer);
  void SetCondition(string condition);
  void SetDependency(string dependency);
  void SetSpecialCommand(SpecialCommand specialcommand);
  CommandPtr ActualCommandMessage;
  bool TimeBased;
  unsigned int TimeInterval;  // Millisecond
  bool ConditionCommand;
  string ConditionID;
  bool ConditionOnOtherCommand;
  string Dependency;
  bool SpecialCommandFlag;
  SpecialCommand Command;
}; // class CommandPro
class ModuleCommands
{
 public:
  ModuleCommands(SmoresModulePtr which_module);
  ~ModuleCommands();
  SmoresModulePtr WhichModule;
  // The vector of command arrays
  vector<CommandPro> CommandSquence;
  // The indicator of whether a command has been executing
  bool FinishedFlag;
  bool ReceivedFlag;
  bool ExecutionFlag;
}; // class ModuleCommands
}
#endif