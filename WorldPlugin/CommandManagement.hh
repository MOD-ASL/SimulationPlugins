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
/// Smart pointer of CommandMessage object
typedef boost::shared_ptr<command_message::msgs::CommandMessage> CommandPtr;

namespace gazebo{
class SmoresModule;
/// Smart pointer of the SmoresModule object
typedef boost::shared_ptr<SmoresModule> SmoresModulePtr;
/// A data structure that stores special command message
//! Used by WorldPlugin, which basically specify connect or disconnect
class SpecialCommand
{
 public:
  /// Constructor
  /*!
    \param commandtype An integer which indicate what type of command this is
                       , see CommandType
    \param modelname1 Name of the first module
    \param modelname2 Name of the second module
    \param node1 An integer indicates the id of the node of the first module
    \param node2 An integer indicates the id of the node of the second module
  */
  SpecialCommand(int commandtype, string modelname1, string modelname2, 
      int node1, int node2);
  /// Constructor
  /*!
    \param commandtype An integer which indicate what type of command this is
                       , see CommandType
  */
  SpecialCommand(int commandtype); 
  /// Constructor
  SpecialCommand();
  /// Destructor
  ~SpecialCommand();
  /// CommandType: 0 no special command; 1 connect; 2 disconnect
  int CommandType;
  /// Name of the first module
  string Module1;
  /// Name of the second module
  string Module2;
  /// Node id of the first module
  int Node1;
  /// Node id of the second module
  int Node2;
}; // class SpecialCommand
/// A data structure that stores normal command message
class CommandPro  // abbr for Command protocol
{
 public:
  /// Constructor
  CommandPro();
  /// Constructor
  //! \param command A smart pointer of the CommandMessage object
  CommandPro(CommandPtr command);
  /// Constructor
  /*!
    \param command A smart pointer of the CommandMessage object
    \param timer Timer of the time-based command, unit: msec
  */
  CommandPro(CommandPtr command, unsigned int timer);
  /// Destructor
  ~CommandPro();
  /// Set a timer for the current command
  //! \param timer A timer, unit: msec
  void SetTimer(unsigned int timer);
  /// Set condition string of the current command
  //! \param condition Condition string
  void SetCondition(string condition);
  /// Set dependency string of the current command
  //! \param dependency Dependency string
  void SetDependency(string dependency);
  /// Make current command a special command
  //! \param specialcommand A specialcommand object
  void SetSpecialCommand(SpecialCommand specialcommand);
  /// A pointer of the CommandMessage object
  CommandPtr ActualCommandMessage;
  /// Flag indicates whether the current command is a time based command
  bool TimeBased;
  /// Timer of the current command, unit: msec
  unsigned int TimeInterval;  // Millisecond
  /// Flag indicates whether this command has a condition
  bool ConditionCommand;
  /// The condition string
  string ConditionID;
  /// Flag indicates whether this command has a dependency
  bool ConditionOnOtherCommand;
  /// The dependency string
  string Dependency;
  /// Flag indicates whether this command is a special command
  bool SpecialCommandFlag;
  /// Special command object
  SpecialCommand Command;
}; // class CommandPro
/// A class that used to manage the commands for each module
//! Used in the command execution
class ModuleCommands
{
 public:
  /// Constructor
  //! \param which_module Pointer of the parent SmoresModule object
  ModuleCommands(SmoresModulePtr which_module);
  /// Destructor
  ~ModuleCommands();
  /// Pointer of the parent SmoresModule object
  SmoresModulePtr WhichModule;
  /// The vector of command array
  vector<CommandPro> CommandSquence;
  /// The indicator of whether a command has been finished
  bool FinishedFlag;
  /// The indicator of whether a command has been received by model
  bool ReceivedFlag;
  /// The indicator of whether a command has been executing
  bool ExecutionFlag;
}; // class ModuleCommands
}
#endif