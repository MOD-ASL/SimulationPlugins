#include "CommandManagement.hh"

namespace gazebo{
SpecialCommand::SpecialCommand(int commandtype, string modelname1, string modelname2,
    int node1, int node2)
{
  CommandType = commandtype;
  Module1 = modelname1;
  Module2 = modelname2;
  Node1 = node1;
  Node2 = node2;
} // SpecialCommand::SpecialCommand
SpecialCommand::SpecialCommand(int commandtype)
{
  CommandType = commandtype;
  Module1 = "";
  Module2 = "";
  Node1 = 0;
  Node2 = 0;
} // SpecialCommand::SpecialCommand
SpecialCommand::SpecialCommand()
{
  CommandType = 0;
  Module1 = "";
  Module2 = "";
  Node1 = 0;
  Node2 = 0;
} // SpecialCommand::SpecialCommand
SpecialCommand::~SpecialCommand(){}
CommandPro::CommandPro()
{
  TimeBased = false;
  TimeInterval = 0; // Not a time based command
  ConditionCommand = false;
  ConditionID = "";
  ConditionOnOtherCommand = false;
  Dependency = "";
  SpecialCommandFlag = false;
} // CommandPro::CommandPro
CommandPro::CommandPro(CommandPtr command)
{
  ActualCommandMessage = command;
  TimeBased = false;
  TimeInterval = 0; // Not a time based command
  ConditionCommand = false;
  ConditionID = "";
  ConditionOnOtherCommand = false;
  Dependency = "";
  SpecialCommandFlag = false;
} // CommandPro::CommandPro
CommandPro::CommandPro(CommandPtr command, unsigned int timer)
{
  ActualCommandMessage = command;
  TimeBased = true;
  TimeInterval = timer; // Not a time based command
  ConditionCommand = false;
  ConditionID = "";
  ConditionOnOtherCommand = false;
  Dependency = "";
  SpecialCommandFlag = false;
} // CommandPro::CommandPro
CommandPro::~CommandPro(){}
void CommandPro::SetTimer(unsigned int timer)
{
  TimeBased = true;
  TimeInterval = timer; 
} // CommandPro::SetTimer
void CommandPro::SetCondition(string condition)
{
  ConditionCommand = true;
  ConditionID = condition;
} // CommandPro::SetCondition
void CommandPro::SetDependency(string dependency)
{
  ConditionOnOtherCommand = true;
  Dependency = dependency;
} // CommandPro::SetDependency
void CommandPro::SetSpecialCommand(SpecialCommand specialcommand)
{
  SpecialCommandFlag = true;
  Command = specialcommand;
} // CommandPro::SetSpecialCommand
ModuleCommands::ModuleCommands(SmoresModulePtr which_module)
{
  this->WhichModule = which_module;
  FinishedFlag = false;
  ReceivedFlag = false;
  ExecutionFlag = false;
} // ModuleCommands::ModuleCommands
ModuleCommands::~ModuleCommands(){}
}