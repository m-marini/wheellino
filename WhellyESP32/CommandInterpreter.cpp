#include "CommandInterpreter.h"
#include "MotionCtrl.h"

#define inrange(value, min, max) ((value) >= (min) && (value) <= (max))

//#define DEBUG
#include "debug.h"

static const int MAX_SPEED = 40;
static const char version[] = "0.7.0";

/*
   Returns true if wrong number of arguments
*/
const boolean CommandInterpreter::parseCmdLongArgs(const char* command, const int from, const int argc, long *argv) {
  DEBUG_PRINT("parseCmdArgs[");
  DEBUG_PRINT(command);
  DEBUG_PRINT("]");
  DEBUG_PRINTLN();

  const char* s0 = command + from;
  const char* s1 = s0;
  char parm[100];
  for (int i = 0; i < argc - 1; i++) {
    s1 = strchr(s0, ' ');
    DEBUG_PRINT("parseCmdLongArgs");
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

const boolean CommandInterpreter::parseCmdIntArgs(const char* command, const int from, const int argc, int *argv) {
  DEBUG_PRINT("parseCmdIntArgs[");
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
   Returns true if value is invalid
*/
const boolean CommandInterpreter::validateLongArg(const long value, const long minValue, const long maxValue, const char* cmd, const int arg) {
  if (!inrange(value, minValue, maxValue)) {
    char error[256];
    sprintf(error, "!! Wrong arg[%d]: value %ld must be between %ld, %ld range: %s",
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
  } else if (strcmp(cmd, "qc") == 0) {
    // qc command
    return handleQcCommand(cmd);
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
  } else if (strncmp(cmd, "fl ", 3) == 0) {
    // fl command
    return handleFeedbackCommand(cmd, 1);
  } else if (strncmp(cmd, "fr ", 3) == 0) {
    // fr command
    return handleFeedbackCommand(cmd, 0);
  } else if (strncmp(cmd, "tcsr ", 5) == 0) {
    // tcsr command
    return handleTcsCommand(cmd, 0);
  } else if (strncmp(cmd, "tcsl ", 5) == 0) {
    // tcsl command
    return handleTcsCommand(cmd, 1);
  } else if (strncmp(cmd, "cc ", 3) == 0) {
    // cc command
    return handleCcCommand(cmd);
  } else if (strncmp(cmd, "cs ", 3) == 0) {
    // cs command
    return handleCsCommand(cmd);
  } else if (strncmp(cmd, "ci ", 3) == 0) {
    // cr command
    return handleCiCommand(cmd);
  } else if (strcmp(cmd, "vr") == 0) {
    // vr command
    char bfr[256];
    strcpy(bfr, "// vr ");
    strcat(bfr, version);
    _wheelly.sendReply(bfr);
    return true;
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
  if (!parseCmdIntArgs(cmd, 3, 1, &angle)) {
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
  if (!parseCmdIntArgs(cmd, 3, 2, params)) {
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
   [ minRotRange, maxRotRange, maxRotPps ]
*/
const boolean CommandInterpreter::handleCcCommand(const char* cmd) {
  int params[3];
  if (!parseCmdIntArgs(cmd, 3, 3, params)) {
    return false;
  }
  // Validates
  for (int i = 0; i < 2; i++) {
    if (!validateIntArg(params[i], 0, 180, cmd, i)) {
      return false;
    }
  }
  if (!validateIntArg(params[2], 0, 20, cmd, 2)) {
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
  if (!parseCmdIntArgs(cmd, 3, 1, &tau)) {
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
    Handles ci command (configure intervals)
*/
const boolean CommandInterpreter::handleCiCommand(const char* cmd) {
  int intervals[2];
  if (!parseCmdIntArgs(cmd, 3, 2, intervals)) {
    return false;
  }
  for (int i = 0; i < 2; i++) {
    if (!validateIntArg(intervals[i], 1, 60000, cmd, i)) {
      return false;
    }
  }
  _wheelly.configIntervals(intervals);
  char bfr[256];
  strcpy(bfr, "// ");
  strcat(bfr, cmd);
  _wheelly.sendReply(bfr);
  return true;
}

/*
   Handles tcs commands (configure motor controllers)
       [
         p0Forw, p1Forw, pxForw
         p0Back, p1Back, pxBack
         ax, alpha
       ]
*/
const boolean CommandInterpreter::handleTcsCommand(const char* cmd, const boolean left) {
  int params[8];
  if (!parseCmdIntArgs(cmd, 5, 8, params)) {
    return false;
  }
  // Validates 3 friction forward parms
  for (int i = 0; i < 3; i++) {
    if (!validateIntArg(params[i], 0, 1000, cmd, i)) {
      return false;
    }
  }
  // Validates 3 friction backward parms
  for (int i = 3; i < 6; i++) {
    if (!validateIntArg(params[i], -1000, 0, cmd, i)) {
      return false;
    }
  }
  // ax
  if (!validateIntArg(params[6], 0, 16383, cmd, 6)) {
    return false;
  }
  // Alpha
  if (!validateIntArg(params[7], 0, 100, cmd, 7)) {
    return false;
  }
  _wheelly.configTcsMotorController(params, left);
  char bfr[256];
  strcpy(bfr, "// ");
  strcat(bfr, cmd);
  _wheelly.sendReply(bfr);
  return true;
}

/*
   Handles feedback commands (configure motor controllers)
       [
         muForw, muBack
       ]
*/
const boolean CommandInterpreter::handleFeedbackCommand(const char* cmd, const boolean left) {
  long params[2];
  if (!parseCmdLongArgs(cmd, 3, 2, params)) {
    return false;
  }
  // Validates mu parms
  for (int i = 0; i < 2; i++) {
    if (!validateLongArg(params[i], 0, 2000000, cmd, i)) {
      return false;
    }
  }
  _wheelly.configFeedbackMotorController(params, left);
  char bfr[256];
  strcpy(bfr, "// ");
  strcat(bfr, cmd);
  _wheelly.sendReply(bfr);
  return true;
}

/**
   Handles qc command
*/
const boolean CommandInterpreter::handleQcCommand(const char* cmd) {
  char bfr[256];
  MotionCtrlClass& motionCtrl = _wheelly.motionCtrl();

  MotorCtrl& leftMotor = motionCtrl.leftMotor();
  sprintf(bfr, "// tcsl %d %d %d %d %d %d %d %d",
          leftMotor.p0Forw(),
          leftMotor.p1Forw(),
          leftMotor.pxForw(),
          leftMotor.p0Back(),
          leftMotor.p1Back(),
          leftMotor.pxBack(),
          leftMotor.ax(),
          leftMotor.alpha()
         );
  _wheelly.sendReply(bfr);
  sprintf(bfr, "// fl %ld %ld",
          leftMotor.muForw(),
          leftMotor.muBack()
         );
  _wheelly.sendReply(bfr);

  MotorCtrl& rightMotor = motionCtrl.rightMotor();
  sprintf(bfr, "// tcsr %d %d %d %d %d %d %d %d",
          rightMotor.p0Forw(),
          rightMotor.p1Forw(),
          rightMotor.pxForw(),
          rightMotor.p0Back(),
          rightMotor.p1Back(),
          rightMotor.pxBack(),
          rightMotor.ax(),
          rightMotor.alpha()
         );
  _wheelly.sendReply(bfr);
  sprintf(bfr, "// fr %ld %ld",
          rightMotor.muForw(),
          rightMotor.muBack()
         );
  _wheelly.sendReply(bfr);

  sprintf(bfr, "// cc %d %d %d",
          motionCtrl.minRotRange(),
          motionCtrl.maxRotRange(),
          motionCtrl.maxRotPps()
         );
  _wheelly.sendReply(bfr);

  sprintf(bfr, "// cs %lu",
          motionCtrl.sensors().tau()
         );
  _wheelly.sendReply(bfr);
  // ci not implemented
  return true;
}
