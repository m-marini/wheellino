
#include "Display.h"

//#define DEBUG
#include "debug.h"

static const unsigned long TIMER_INTERVAL = 400ul;
static const unsigned long BLINK_ON = 0;
static const unsigned long BLINK_COUNT = 2;
static const unsigned long ACTIVITY_INTERVAL = 2000ul;

static const char* ERROR_STRING = "\x01 ";
static const char* MOVING_STRING = "\x02 ";
static const char* BLOCK_STRING = "\x03 ";
static const char* CONNECTED_STRING = "\x04\x05 ";
static const char* ACTIVITY_STRING = "\x06";
static const char* SUPPLY_STRING = "\x07";
static const char* BLANK1_STRING = " ";
static const char* BLANK2_STRING = "  ";
static const char* BLANK3_STRING = "   ";
static const char* BLANK4_STRING = "    ";

static const int STOP_DISTANCE = 20;

static uint8_t errorChar[] = {
  0b01110,
  0b10001,
  0b10101,
  0b10101,
  0b10001,
  0b10101,
  0b10001,
  0b01110,
};

static uint8_t fullBlockChar[] = { 0b01110,
                                   0b10001,
                                   0b10001,
                                   0b11011,
                                   0b10101,
                                   0b11011,
                                   0b10001,
                                   0b11111 };

static uint8_t forwardBlockChar[] = {
  0b01010,
  0b00100,
  0b01010,
  0b00000,
  0b01110,
  0b10001,
  0b10001,
  0b11111,
};

static uint8_t backwardBlockChar[] = {
  0b01110,
  0b10001,
  0b10001,
  0b11111,
  0b00000,
  0b11011,
  0b00100,
  0b11011,
};

static uint8_t connectedChar[] = { 0b00000,
                                   0b00000,
                                   0b00000,
                                   0b11100,
                                   0b00010,
                                   0b11001,
                                   0b00101,
                                   0b10101 };

static uint8_t uparrowChar[] = {
  0b00100,
  0b01010,
  0b11011,
  0b01010,
  0b01010,
  0b01010,
  0b01010,
  0b01110,
};

static uint8_t terminalChar[] = { 0b01110,
                                  0b10001,
                                  0b10001,
                                  0b10001,
                                  0b10001,
                                  0b10101,
                                  0b10001,
                                  0b01110 };

static uint8_t activityChar[] = { 0b01110,
                                  0b10001,
                                  0b10101,
                                  0b00100,
                                  0b00100,
                                  0b00100,
                                  0b01010,
                                  0b10001 };

static uint8_t supplyCharSet[][8] = {
  { // Supply chars
    0b01110,
    0b11111,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b11111 },
  { 0b01110,
    0b11111,
    0b10001,
    0b10001,
    0b10001,
    0b10001,
    0b11111,
    0b11111 },
  { 0b01110,
    0b11111,
    0b10001,
    0b10001,
    0b10001,
    0b11111,
    0b11111,
    0b11111 },
  { 0b01110,
    0b11111,
    0b10001,
    0b10001,
    0b11111,
    0b11111,
    0b11111,
    0b11111 },
  { 0b01110,
    0b11111,
    0b10001,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111 },
  { 0b01110,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111,
    0b11111 }
};

/*
   Handles scrolling timer
*/
void DisplayClass::handleTimer(void* context, const unsigned long n) {
  ((DisplayClass*)context)->scroll(n);
  ((DisplayClass*)context)->blink(n);
}

/*
   Creates the DisplayClass
*/
DisplayClass::DisplayClass(const uint8_t addr)
  : _lcd(addr),
    _distance(-1) {
  _timer.onNext(handleTimer, this);
  _timer.interval(TIMER_INTERVAL);
  _timer.continuous(true);
}

/*
   Initializes DisplayClass
*/
void DisplayClass::begin() {
  _lcd.begin(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  _lcd.createChar(ERROR_STRING[0], errorChar);
  _lcd.createChar(MOVING_STRING[0], uparrowChar);
  _lcd.createChar(CONNECTED_STRING[0], connectedChar);
  _lcd.createChar(CONNECTED_STRING[1], terminalChar);
  _lcd.createChar(ACTIVITY_STRING[0], activityChar);
  _lcd.createChar(SUPPLY_STRING[0], supplyCharSet[0]);
  _lcd.setBacklight(255);
  _timer.start();
}

/*
   Polls
*/
void DisplayClass::polling(const unsigned long clockTime) {
  _timer.polling(clockTime);
}

/*
   Clears the scrolling buffer
*/
void DisplayClass::clear() {
  _buffer[0] = 0;
  _scrollOffset = 0;
  showScroll();
}

/*
   Sets the error status
   @parm error the error code
*/
void DisplayClass::error(const uint8_t error) {
  uint8_t old = _error;
  _error = error;
  if (old != error) {
    showInfo();
    refreshInfo();
  }
}

/*
   Sets the connected status
   @param connected true if connected
*/
void DisplayClass::connected(const boolean connected) {
  boolean old = _connected;
  _connected = connected;
  if (old != connected) {
    showInfo();
  }
}

/*
   Sets the moving status
   @param connected true if connected
*/
void DisplayClass::move(const boolean move) {
  boolean old = _move;
  _move = move;
  if (old != move) {
    showInfo();
  }
}

/*
   Sets the connected status
   @param distance the distance flag
*/
void DisplayClass::distance(const int distance) {
  int old = _distance;
  _distance = distance;
  if (old != distance) {
    showInfo();
  }
}

/*
   Sets the block status
   @param block the block
*/
void DisplayClass::block(const block_t block) {
  const block_t old = _block;
  _block = block;
  if (old != _block) {
    uint8_t* customChar = fullBlockChar;
    switch (_block) {
      case FORWARD_BLOCK:
        customChar = forwardBlockChar;
        break;
      case BACKWARD_BLOCK:
        customChar = backwardBlockChar;
        break;
      default:
        customChar = fullBlockChar;
        break;
    }
    _lcd.createChar(BLOCK_STRING[0], customChar);
    showInfo();
  }
}

/*
   Sets the activity status
   @param time the activity time
*/
void DisplayClass::activity(const unsigned long time) {
  unsigned long old = _activityTime;
  _activityTime = time + ACTIVITY_INTERVAL;
  if (time >= old) {
    showInfo();
  }
}

/*
   Sets the supply status
   @param levelthe supply level
*/
void DisplayClass::supply(const int level) {
  const int old = _supplyLevel;
  _supplyLevel = min(max(level, 0), (int)(sizeof(supplyCharSet) / sizeof(supplyCharSet[0])));
  if (old != _supplyLevel) {
    DEBUG_PRINT("DisplayClass::supply(");
    DEBUG_PRINT(_supplyLevel);
    DEBUG_PRINT(")");
    DEBUG_PRINTLN();
    _lcd.createChar(SUPPLY_STRING[0], supplyCharSet[_supplyLevel]);
    showInfo();
  }
}

/*
   Write a char in the scrolling buffer
*/
size_t DisplayClass::write(uint8_t data) {
  size_t len = strlen(_buffer);
  if (len < SCROLL_BUFFER_SIZE) {
    _buffer[len++] = data;
    _buffer[len] = 0;
  }
  showScroll();
  return 1;
}

/**
   Shows the scroll line
*/
void DisplayClass::showInfo() {
  _lcd.setCursor(0, 1);

  // ER
  _lcd.print(_error && !_blink ? ERROR_STRING : BLANK2_STRING);

  // HA
  _lcd.setCursor(2, 1);
  _lcd.print(_move ? MOVING_STRING : BLANK2_STRING);

  // BL
  _lcd.setCursor(4, 1);
  _lcd.print(_blink || _block == NO_BLOCK ? BLANK2_STRING : BLOCK_STRING);

  // Distance
  char distance[10];
  if (_distance < 0 || _distance <= STOP_DISTANCE && _blink) {
    strcpy(distance, BLANK4_STRING);
  } else {
    sprintf(distance, "%3d ", _distance);
  }
  _lcd.setCursor(6, 1);
  _lcd.print(distance);

  // CN
  _lcd.setCursor(10, 1);
  _lcd.print(_connected ? CONNECTED_STRING : BLANK3_STRING);

  // Act
  _lcd.setCursor(13, 1);
  _lcd.print(millis() <= _activityTime && !_blink ? ACTIVITY_STRING : BLANK1_STRING);

  // Supply
  _lcd.setCursor(15, 1);
  _lcd.print(SUPPLY_STRING);
}

/**
   Shows the scroll line
*/
void DisplayClass::showScroll() {

  _lcd.setCursor(0, 0);
  size_t len = strlen(_buffer);
  if (len <= DISPLAY_WIDTH) {
    _lcd.print(_buffer);
    for (int i = DISPLAY_WIDTH - len; i--;) {
      _lcd.print(' ');
    }
  } else {
    size_t tailLen = len - _scrollOffset;
    char bfr[DISPLAY_WIDTH + 1];
    if (tailLen >= DISPLAY_WIDTH) {
      strncpy(bfr, _buffer + _scrollOffset, DISPLAY_WIDTH);
    } else {
      _lcd.print(_buffer + _scrollOffset);
      strncpy(bfr, _buffer, DISPLAY_WIDTH - tailLen);
    }
    _lcd.print(bfr);
  }
}

/*
   Shows wifi info
*/
void DisplayClass::showWiFiInfo(const char* info) {
  strncpy(_wiFiInfo, info, WIFI_BUFFER_SIZE - 1);
  refreshInfo();
}

/*
   Refresh the info
*/
void DisplayClass::refreshInfo() {
  char bfr[SCROLL_BUFFER_SIZE];
  strcpy(bfr, _wiFiInfo);
  if (_error != 0) {
    strcat(bfr, " - RC: ");
    itoa(_error, bfr + strlen(bfr), 10);
    // add _error
  }
  if (strlen(bfr) > 0) {
    strcat(bfr, " - ");
  }
  refreshScroll(bfr);
}

/*
   Refreshed the scroll
*/
void DisplayClass::refreshScroll(const char* line) {
  if (strcmp(_buffer, line) != 0) {
    clear();
    print(line);
  }
}


/*
   Scrolls line
*/
void DisplayClass::scroll(const unsigned long n) {
  _scrollOffset++;
  if (_scrollOffset >= strlen(_buffer)) {
    _scrollOffset = 0;
  }
  showScroll();
}

/*
   Blink data
*/
void DisplayClass::blink(const unsigned long n) {
  _blink = (n % BLINK_COUNT) > BLINK_ON;
  showInfo();
}
