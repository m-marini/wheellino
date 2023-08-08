#include "CommandInterpreter.h"

#define inrange(value, min, max) ((value) >= (min) && (value) <= (max))

//#define DEBUG
#include "debug.h"

static const int MAX_SPEED = 40;

/*
   Returns true if wrong number of arguments
*/
const boolean CommandInterpreter::parseCmdArgs(const char* command, const int from, const int argc, int *argv) {
  DEBUG_PRINT("parseCmdArgs[");
  DEBUG_PRINT(command);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();

  const char* s0 = command + from;
  const char* s1 = s0;
  char parm[100];
  for (int i = 0; i < argc - 1; i++) {
    s1 = strchr(s0, ' ');
    DEBUG_PRINT("parseCmdArgs");
    if (!s1 || s1 == s0) {
      // Missing argument
      char error[256];
      sprintf(error, "!! Found %d arguments, expected %d: %s",
              i + 1,
              argc,
              command);

      Serial.println(error);
      _wheelly.sendReply(error);
      return false;
    }
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

    *argv++ = atoi(parm);
    s0 = s1 + 1;
  }
  DEBUG_PRINT("s0=[");
  DEBUG_PRINT(s0);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();
  *argv = atoi(s0);
  /*
    const String cmd = command;
    const String args = cmd.substring(from);
    int s0 = 0;
    int s1 = 0;
    for (int i = 0; i < argc - 1; i++) {
    s1 = args.indexOf(' ', s0);
    if (s1 <= 0) {
      char error[256];
      sprintf(error, "!! Found %d arguments, expected %d: %s",
              i + 1,
              argc,
              cmd.c_str());
      Serial.println(error);
      _wheelly.sendReply(error);
      return false;
    }
     argv++ = args.substring(s0 , s1).toInt();
    s0 = s1 + 1;
    }
    argv = args.substring(s1).toInt();
  */
  return true;
}

/*
   Returns true if value is invalid
*/
const boolean CommandInterpreter::validateIntArg(const int value, const int minValue, const int maxValue, const char* cmd, const int arg) {
  if (!inrange(value, minValue, maxValue)) {
    char error[256];
    sprintf(error, "!! Wrong arg[%d]: value %d must be between %d, %d range: %s",
            arg + 1,
            value,
            minValue,
            maxValue,
            cmd);
    Serial.println(error);
    _wheelly.sendReply(error);
    return false;
  }
  return true;
}


/*
  Executes a command.
  Returns true if command executed
  @param time the current time stamp
  @param command the command
*/
const boolean CommandInterpreter::execute(const unsigned long time, const char* command) {
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

  DEBUG_PRINT("// processCommand: ");
  DEBUG_PRINTLN(cmd);
  if (strcmp(cmd, "ha") == 0) {
    // ha command
    _wheelly.halt();
    return true;
  } else if (strncmp(cmd, "sc ", 3) == 0) {
    // sc command
    return handleScCommand(cmd);
  } else if (strncmp(cmd, "mv ", 3) == 0) {
    // mv command
    return handleMvCommand(cmd);
  } else if (strcmp(cmd, "rs") == 0) {
    // rs command
    _wheelly.reset();
    return true;
  } else if (strncmp(cmd, "ck ", 3) == 0) {
    // ck command
    char bfr[256];
    sprintf(bfr, "%s %ld %ld", cmd, time, millis());
    _wheelly.sendReply(bfr);
    return true;
  } else if (strncmp(cmd, "cc ", 3) == 0) {
    // cc command
    return handleCcCommand(cmd);
  } else if (strncmp(cmd, "cs ", 3) == 0) {
    // cs command
    return handleCsCommand(cmd);
  } else if (strncmp(cmd, "ct ", 3) == 0) {
    // For compatibility ignore ct command and just reply
    char bfr[256];
    strcpy(bfr, "// ");
    strcat(bfr, cmd);
    _wheelly.sendReply(bfr);
    return true;
  } else if (strncmp(cmd, "cl ", 3) == 0) {
    // cl command
    return handleClrCommand(cmd, true);
  } else if (strncmp(cmd, "cr ", 3) == 0) {
    // cr command
    return handleClrCommand(cmd, false);
  } else if (strncmp(cmd, "//", 2) == 0
             || strncmp(cmd, "!!", 2) == 0
             || cmd[0] == 0) {
    // Ignore comments, errors, empty line
    return true;
  } else {
    char msg[256];
    strcpy(msg, "!! Wrong command: ");
    strcat(msg, cmd);
    Serial.println(msg);
    _wheelly.sendReply(msg);
    return false;
  }
}

/*
  Handles sc command
*/
const boolean CommandInterpreter::handleScCommand(const char* cmd) {
  int angle;
  if (!parseCmdArgs(cmd, 3, 1, &angle)) {
    return false;
  }
  if (!validateIntArg(angle, -90, 90, cmd, 0)) {
    return false;
  }
  _wheelly.scan(angle);
  DEBUG_PRINT("// handleScCommand: next scan=");
  DEBUG_PRINT(angle);
  return true;
}

/*
  Handles mv command
*/
const boolean CommandInterpreter::handleMvCommand(const char* cmd) {
  int params[2];
  if (!parseCmdArgs(cmd, 3, 2, params)) {
    return false;
  }
  // Validate direction
  if (!validateIntArg(params[0], -180, 179, cmd, 0)) {
    return false;
  }
  // Validate speed
  if (!validateIntArg(params[1], -MAX_SPEED, MAX_SPEED, cmd, 1)) {
    return false;
  }

  DEBUG_PRINT(F("     params="));
  DEBUG_PRINT(params[0]);
  DEBUG_PRINT(F(", "));
  DEBUG_PRINT(params[1]);
  DEBUG_PRINTLN();
  _wheelly.move(params[0], params[1]);
  return true;
}

/*
   Handles cc command (configure controller)
   [ moveRotThreshold, minRotRange, maxRotRange, maxRotPps ]
*/
const boolean CommandInterpreter::handleCcCommand(const char* cmd) {
  int params[4];
  if (!parseCmdArgs(cmd, 3, 4, params)) {
    return false;
  }
  // Validates
  for (int i = 0; i < 3; i++) {
    if (!validateIntArg(params[i], 0, 180, cmd, i)) {
      return false;
    }
  }
  if (!validateIntArg(params[3], 0, 20, cmd, 3)) {
    return false;
  }

  _wheelly.configMotionController(params);
  char bfr[256];
  strcpy(bfr, "// ");
  strcat(bfr, cmd);
  _wheelly.sendReply(bfr);
  return true;
}

/*
    Handles cs command (configure decay time of motion controller)
*/
const boolean CommandInterpreter::handleCsCommand(const char* cmd) {
  int tau;
  if (!parseCmdArgs(cmd, 3, 1, &tau)) {
    return false;
  }
  if (!validateIntArg(tau, 1, 10000, cmd, 0)) {
    return false;
  }
  _wheelly.configMotorSensors(tau);
  char bfr[256];
  strcpy(bfr, "// ");
  strcat(bfr, cmd);
  _wheelly.sendReply(bfr);
  return true;
}

/*
   Handles clr command (configure motor controllers)
       [
         muForw, muBack, ax
         p0Forw, p1Forw,
         p0Back, p1Back,
       ]
*/
const boolean CommandInterpreter::handleClrCommand(const char* cmd, const boolean left) {
  int params[7];
  if (!parseCmdArgs(cmd, 3, 7, params)) {
    return false;
  }
  // Validates
  for (int i = 0; i < 3; i++) {
    if (!validateIntArg(params[i], 0, 16383, cmd, i)) {
      return false;
    }
  }
  for (int i = 3; i < 5; i++) {
    if (!validateIntArg(params[i], 0, 255, cmd, i)) {
      return false;
    }
  }
  for (int i = 5; i < 7; i++) {
    if (!validateIntArg(params[i], -255, 0, cmd, i)) {
      return false;
    }
  }
  if (left) {
    _wheelly.configLeftMotorController(params);
  } else {
    _wheelly.configRightMotorController(params);
  }
  char bfr[256];
  strcpy(bfr, "// ");
  strcat(bfr, cmd);
  _wheelly.sendReply(bfr);
  return true;
}
