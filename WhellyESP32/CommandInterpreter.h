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

    const boolean handleTcsCommand(const char* cmd, const boolean left);
    const boolean handleFeedbackCommand(const char* cmd, const boolean left);
    const boolean handleScCommand(const char* cmd);
    const boolean handleMvCommand(const char* cmd);
    const boolean handleCcCommand(const char* cmd);
    const boolean handleCsCommand(const char* cmd);
    const boolean handleCiCommand(const char* cmd);
    const boolean handleQcCommand(const char* cmd);
    const boolean validateIntArg(const int value, const int minValue, const int maxValue, const char* cmd, const int arg);
    const boolean validateLongArg(const long value, const long minValue, const long maxValue, const char* cmd, const int arg);
    const boolean parseCmdIntArgs(const char* command, const int from, const int argc, int *argv);
    const boolean parseCmdLongArgs(const char* command, const int from, const int argc, long *argv);
};
#endif
