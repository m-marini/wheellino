/*
 * Copyright (c) 2023  Marco Marini, marco.marini@mmarini.org
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 *    END OF TERMS AND CONDITIONS
 *
 */

#include <stdarg.h>

#include "CommandInterpreter.h"
#include "MotionCtrl.h"

#define inrange(value, min, max) ((value) >= (min) && (value) <= (max))

//#define DEBUG
#include "debug.h"

/*
   Returns true if wrong number of arguments
*/
const boolean CommandInterpreter::parseCmdLongArgs(const char *command, const int from, const int argc, long *argv) {
  DEBUG_PRINT("// CommandInterpreter::parseCmdLongArgs[");
  DEBUG_PRINT(command);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();

  const char *s0 = command + from;
  const char *s1 = s0;
  char parm[100];
  for (int i = 0; i < argc - 1; i++) {
    s1 = strchr(s0, ' ');
    if (!s1 || s1 == s0) {
      // Missing argument
      char error[256];
      sprintf(error, "!! Found %d arguments, expected %d: %s",
              i + 1,
              argc,
              command);

      Serial.println(error);
      sendError(error);
      return false;
    }
    DEBUG_PRINT("// CommandInterpreter::parseCmdLongArgs");
    DEBUG_PRINT("s0=[");
    DEBUG_PRINT(s0);
    DEBUG_PRINT("], s1=[");
    DEBUG_PRINT(s1);
    DEBUG_PRINT("], s1-s0=");
    DEBUG_PRINT(s1 - s0);
    DEBUG_PRINTLN();

    strncpy(parm, s0, s1 - s0);
    parm[s1 - s0] = 0;
    DEBUG_PRINT("parm=[");
    DEBUG_PRINT(parm);
    DEBUG_PRINT("]");
    DEBUG_PRINTLN();

    *argv++ = atol(parm);
    s0 = s1 + 1;
  }
  DEBUG_PRINT("s0=[");
  DEBUG_PRINT(s0);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();
  *argv = atol(s0);
  return true;
}

const boolean CommandInterpreter::parseCmdIntArgs(const char *command, const int from, const int argc, int *argv) {
  DEBUG_PRINT("// CommandInterpreter::parseCmdIntArgs[");
  DEBUG_PRINT(command);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();

  const char *s0 = command + from;
  const char *s1 = s0;
  char parm[100];
  for (int i = 0; i < argc - 1; i++) {
    s1 = strchr(s0, ' ');
    if (!s1 || s1 == s0) {
      // Missing argument
      char error[256];
      sprintf(error, "!! Found %d arguments, expected %d: %s",
              i + 1,
              argc,
              command);

      Serial.println(error);
      sendError(error);
      return false;
    }
    DEBUG_PRINT("// CommandInterpreter::parseCmdIntArgs ");
    DEBUG_PRINT(" s0=[");
    DEBUG_PRINT(s0);
    DEBUG_PRINT("], s1=[");
    DEBUG_PRINT(s1);
    DEBUG_PRINT("], s1-s0=");
    DEBUG_PRINT(s1 - s0);
    DEBUG_PRINTLN();

    strncpy(parm, s0, s1 - s0);
    parm[s1 - s0] = 0;
    DEBUG_PRINT("parm=[");
    DEBUG_PRINT(parm);
    DEBUG_PRINT("]");
    DEBUG_PRINTLN();

    *argv++ = atoi(parm);
    s0 = s1 + 1;
  }
  DEBUG_PRINT("s0=[");
  DEBUG_PRINT(s0);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();
  *argv = atoi(s0);
  return true;
}

/*
   Returns true if value is invalid
*/
const boolean CommandInterpreter::validateIntArg(const int value, const int minValue, const int maxValue, const char *cmd, const int arg) {
  if (!inrange(value, minValue, maxValue)) {
    char error[256];
    sprintf(error, "!! Wrong arg[%d]: value %d must be between %d, %d range: %s",
            arg + 1,
            value,
            minValue,
            maxValue,
            cmd);
    Serial.println(error);
    sendError(error);
    return false;
  }
  return true;
}


/*
   Returns true if value is invalid
*/
const boolean CommandInterpreter::validateLongArg(const long value, const long minValue, const long maxValue, const char *cmd, const int arg) {
  if (!inrange(value, minValue, maxValue)) {
    char error[256];
    sprintf(error, "!! Wrong arg[%d]: value %ld must be between %ld, %ld range: %s",
            arg + 1,
            value,
            minValue,
            maxValue,
            cmd);
    Serial.println(error);
    sendError(error);
    return false;
  }
  return true;
}

/**
  Adds command without arguments
  @param cmd the command
  @param context the context
  @param callback the command execution callback
*/
void CommandInterpreter::addCommand(const char *cmd,
                                    void (*callback)(void *, const unsigned long, const char *),
                                    void *context) {
  if (_voidCommandCount >= MAX_COMMANDS) {
    sendError("!! CommandInterpreter::addCommand: Too void command");
    return;
  }
  strcpy(_voidCommands[_voidCommandCount].command, cmd);
  _voidCommands[_voidCommandCount].execute = callback;
  _voidCommands[_voidCommandCount].context = context;
  _voidCommandCount++;
}

/**
  Adds command without arguments
  @param cmd the command
  @param context the context
  @param callback the command execution callback
*/
void CommandInterpreter::addStrCommand(const char *cmd,
                                       void (*callback)(void *, const unsigned long, const char *),
                                       void *context) {
  if (_strCommandCount >= MAX_COMMANDS) {
    sendError("!! CommandInterpreter::addStrCommand: Too string command");
    return;
  }
  strcpy(_strCommands[_strCommandCount].command, cmd);
  strcat(_strCommands[_strCommandCount].command, " ");
  _strCommands[_strCommandCount].execute = callback;
  _strCommands[_strCommandCount].context = context;
  _strCommandCount++;
}

/**
  Adds command with integer arguments
  @param cmd the command
  @param context the context
  @param callback the command execution callback
  @param argc the number of arguments
  @param varargs the pair of min max values
*/
void CommandInterpreter::addIntCommand(const char *cmd,
                                       void (*callback)(void *, const unsigned long, const char *, const int *),
                                       void *context,
                                       const int argc, ...) {
  va_list ap;
  va_start(ap, argc);
  if (_intCommandCount >= MAX_COMMANDS) {
    sendError("!! CommandInterpreter::addIntCommand: Too commands");
    return;
  }
  if (argc > MAX_ARGUMENTS) {
    sendError("!! CommandInterpreter::addIntCommand: Too arguments");
    return;
  }
  strcpy(_intCommands[_intCommandCount].command, cmd);
  strcat(_intCommands[_intCommandCount].command, " ");

  _intCommands[_intCommandCount].execute = callback;
  _intCommands[_intCommandCount].context = context;
  _intCommands[_intCommandCount].argc = argc;
  for (int i = 0; i < argc; i++) {
    _intCommands[_intCommandCount].min[i] = va_arg(ap, int);
    _intCommands[_intCommandCount].max[i] = va_arg(ap, int);
  }
  va_end(ap);
  _intCommandCount++;
}


/**
  Adds command with long arguments
  @param cmd the command
  @param context the context
  @param callback the command execution callback
  @param argc the number of arguments
  @param varargs the pair of min max long values
*/
void CommandInterpreter::addLongCommand(const char *cmd,
                                        void (*callback)(void *, const unsigned long, const char *, const long *),
                                        void *context,
                                        const int argc, ...) {
  va_list ap;
  va_start(ap, argc);
  if (_longCommandCount >= MAX_COMMANDS) {
    sendError("!! CommandInterpreter::addLongCommand: Too commands");
    return;
  }
  if (argc > MAX_ARGUMENTS) {
    sendError("!! CommandInterpreter::addLongCommand: Too arguments");
    return;
  }
  strcpy(_longCommands[_longCommandCount].command, cmd);
  strcat(_longCommands[_longCommandCount].command, " ");

  _longCommands[_longCommandCount].execute = callback;
  _longCommands[_longCommandCount].context = context;
  _longCommands[_longCommandCount].argc = argc;
  for (int i = 0; i < argc; i++) {
    _longCommands[_longCommandCount].min[i] = va_arg(ap, long);
    _longCommands[_longCommandCount].max[i] = va_arg(ap, long);
  }
  va_end(ap);
  _longCommandCount++;
}

/*
  Executes a command.
  Returns true if command executed
  @param time the current time stamp
  @param command the command
*/
const boolean CommandInterpreter::execute(const unsigned long t0, const char *command) {
  char cmd[256];
  // Left trim
  while (isspace(*command)) {
    command++;
  }
  strcpy(cmd, command);

  // Right trim
  for (int i = strlen(cmd) - 1; i >= 0 && isspace(cmd[i]); i--) {
    cmd[i] = 0;
  }

  DEBUG_PRINT("// CommandInterpreter::execute: ");
  DEBUG_PRINTLN(cmd);

  /* Ignore comments, errors, empty line */
  if (strncmp(cmd, "//", 2) == 0
      || strncmp(cmd, "!!", 2) == 0
      || cmd[0] == 0) {
    return true;
  }

  /* Looks for void command */
  int cmdIdx = -1;
  int cmdType = 0;
  for (int i = 0; i < _voidCommandCount; i++) {
    if (strcmp(cmd, _voidCommands[i].command) == 0) {
      cmdIdx = i;
      break;
    }
  }


  /* Looks for int command */
  if (cmdIdx < 0) {
    for (int i = 0; i < _intCommandCount; i++) {
      if (strncmp(cmd, _intCommands[i].command, strlen(_intCommands[i].command)) == 0) {
        cmdIdx = i;
        cmdType = 1;
        break;
      }
    }
  }

  /* Looks for long command */
  if (cmdIdx < 0) {
    for (int i = 0; i < _longCommandCount; i++) {
      if (strncmp(cmd, _longCommands[i].command, strlen(_longCommands[i].command)) == 0) {
        cmdIdx = i;
        cmdType = 2;
        break;
      }
    }
  }

  /* Looks for string command */
  if (cmdIdx < 0) {
    for (int i = 0; i < _strCommandCount; i++) {
      if (strncmp(cmd, _strCommands[i].command, strlen(_strCommands[i].command)) == 0) {
        cmdIdx = i;
        cmdType = 3;
        break;
      }
    }
  }

  /* Checks for command not found */
  if (cmdIdx < 0) {
    char error[256];
    strcpy(error, "!! Wrong command: ");
    strcat(error, cmd);
    Serial.println(error);
    sendError(error);
    return false;
  }

  DEBUG_PRINT("// CommandInterpreter::execute cmdIdx: ");
  DEBUG_PRINT(cmdIdx);
  DEBUG_PRINT(", cmdType");
  DEBUG_PRINT(cmdType);
  DEBUG_PRINTLN();

  if (cmdType == 0) {
    _voidCommands[cmdIdx].execute(_voidCommands[cmdIdx].context, t0, cmd);
  } else if (cmdType == 1) {
    /* Parse for integer arguments */
    const int argc = _intCommands[cmdIdx].argc;
    DEBUG_PRINT("// CommandInterpreter::execute argc =");
    DEBUG_PRINT(argc);
    DEBUG_PRINTLN();

    int args[MAX_ARGUMENTS];
    if (!parseCmdIntArgs(cmd, strlen(_intCommands[cmdIdx].command), argc, args)) {
      return false;
    }
    DEBUG_PRINT("// CommandInterpreter::execute validate");
    DEBUG_PRINTLN();
    // Validate arguments
    for (int i = 0; i < argc; i++) {
      if (!validateIntArg(args[i],
                          _intCommands[cmdIdx].min[i],
                          _intCommands[cmdIdx].max[i],
                          cmd, i)) {
        return false;
      }
    }
    _intCommands[cmdIdx].execute(_intCommands[cmdIdx].context, t0, cmd, args);
  } else if (cmdType == 2) {
    /* Parse for long arguments */
    const int argc = _longCommands[cmdIdx].argc;
    DEBUG_PRINT("// CommandInterpreter::execute argc =");
    DEBUG_PRINT(argc);
    DEBUG_PRINTLN();

    long args[MAX_ARGUMENTS];
    if (!parseCmdLongArgs(cmd, strlen(_longCommands[cmdIdx].command), argc, args)) {
      return false;
    }
    DEBUG_PRINT("// CommandInterpreter::execute validate");
    DEBUG_PRINTLN();
    // Validate arguments
    for (int i = 0; i < argc; i++) {
      if (!validateLongArg(args[i],
                           _longCommands[cmdIdx].min[i],
                           _longCommands[cmdIdx].max[i],
                           cmd, i)) {
        return false;
      }
    }
    _longCommands[cmdIdx].execute(_longCommands[cmdIdx].context, t0, cmd, args);
  } else if (cmdType == 3) {
    _strCommands[cmdIdx].execute(_strCommands[cmdIdx].context, t0, cmd);
  } else {
    return false;
  }
  return true;
}
