#ifndef CommandInterpreter_h
#define CommandInterpreter_h

#include "Arduino.h"

#define MAX_COMMANDS 10
#define MAX_ARGUMENTS 10

/*
   Parses and execute for wheelly commands
*/
class CommandInterpreter {
  public:
    /*
       Creates the command interpreter
    */
    CommandInterpreter() {}

    /*
       Sets on reply call back
    */
    void onError(void(* callback)(void*, const char*), void* context = NULL) {
      _onError = callback;
      _context = context;
    }

    /**
      Adds command without arguments
      @param cmd the command
      @param context the context
      @param callback the command execution callback
    */
    void addCommand(const char *cmd,
                    void(*callback)(void *, const unsigned long, const char *),
                    void *context);

    /**
      Adds command with integer arguments
      @param cmd the command
      @param context the context
      @param callback the command execution callback
      @param argc the number of arguments
      @param varargs the pair of min max values
    */
    void addIntCommand(const char *cmd,
                       void(*callback)(void *, const unsigned long, const char *, const int*),
                       void *context,
                       const int argc, ...);

    /**
      Adds command with long arguments
      @param cmd the command
      @param callback the command execution callback
      @param argc the number of arguments
      @param varargs the pair of min max long values
      @param context the context
    */
    void addLongCommand(const char *cmd,
                        void(*callback)(void*, const unsigned long, const char *, const long*),
                        void* context,
                        const int argc, ...);

    /*
      Parse and execute a command.
      Returns true if command executed
      @param time the current time stamp
      @param command the command
    */
    const boolean execute(const unsigned long time, const char* command);

  private:
    int _voidCommandCount;
    int _intCommandCount;
    int _longCommandCount;
    void (*_onError)(void*, const char *);
    void *_context;
    
    struct {
      char command[8];
      void (*execute)(void*, const unsigned long, const char *);
      void *context;
    } _voidCommands[MAX_COMMANDS];
    
    struct {
      char command[8];
      void (*execute)(void*, const unsigned long, const char *, const int*);
      void *context;
      int argc;
      int min[MAX_ARGUMENTS];
      int max[MAX_ARGUMENTS];
    } _intCommands[MAX_COMMANDS];
    
    struct {
      char command[8];
      void (*execute)(void*, const unsigned long, const char *, const long*);
      void *context;
      int argc;
      long min[MAX_ARGUMENTS];
      long max[MAX_ARGUMENTS];
    } _longCommands[MAX_COMMANDS];

    const boolean validateIntArg(const int value, const int minValue, const int maxValue, const char* cmd, const int arg);
    const boolean validateLongArg(const long value, const long minValue, const long maxValue, const char* cmd, const int arg);
    const boolean parseCmdIntArgs(const char* command, const int from, const int argc, int *argv);
    const boolean parseCmdLongArgs(const char* command, const int from, const int argc, long *argv);
    void sendError(const char* msg) {
      if (_onError) {
        _onError(_context, msg);
      }
    }
};
#endif
