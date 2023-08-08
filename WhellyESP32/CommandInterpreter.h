#ifndef CommandInterpreter_h
#define CommandInterpreter_h

#include "Arduino.h"
#include "Wheelly.h"

/*
   Parses and execute for wheelly commands
*/
class CommandInterpreter {
  public:
    /*
       Creates the command interpreter
    */
    CommandInterpreter(Wheelly& wheelly) :
      _wheelly(wheelly)
    {}

    /*
      Execute a command.
      Returns true if command executed
      @param time the current time stamp
      @param command the command
    */
    const boolean execute(const unsigned long time, const char* command);

  private:
    Wheelly& _wheelly;

    const boolean handleScCommand(const char* cmd);
    const boolean handleMvCommand(const char* cmd);
    const boolean handleCcCommand(const char* cmd);
    const boolean handleCsCommand(const char* cmd);
    const boolean handleClrCommand(const char* cmd, const boolean left);
    const boolean validateIntArg(const int value, const int minValue, const int maxValue, const char* cmd, const int arg);
    const boolean parseCmdArgs(const char* command, const int from, const int argc, int *argv);
};
#endif
