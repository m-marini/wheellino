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

#ifndef Display_h
#define Display_h

#include <Arduino.h>
#include <LiquidCrystal_PCF8574.h>

#include "Timer.h"

#define DISPLAY_WIDTH 16
#define DISPLAY_HEIGHT 2
#define SCROLL_BUFFER_SIZE 128
#define WIFI_BUFFER_SIZE 64

/*
   The type of block
*/
typedef enum {
  NO_BLOCK,
  FORWARD_BLOCK,
  BACKWARD_BLOCK,
  FULL_BLOCK,
} block_t;

/*
   Displays status of Wheelly on LCD
*/
class DisplayClass : public Print {
private:
  LiquidCrystal_PCF8574 _lcd;
  Timer _timer;
  size_t _scrollOffset;
  boolean _blink;
  uint8_t _error;
  boolean _move;
  block_t _block;
  uint8_t _connected;
  unsigned long _activityTime;
  int _distance;
  int _supplyLevel;
  char _buffer[SCROLL_BUFFER_SIZE];
  char _wiFiInfo[WIFI_BUFFER_SIZE];

  static void handleTimer(void*, const unsigned long);

  void showScroll(void);
  void showInfo(void);
  void refreshInfo(void);
  void refreshScroll(const char* line);
  void scroll(const unsigned long);
  void blink(const unsigned long);

public:
  // Creates the display
  DisplayClass(const uint8_t addr = 0x27);

  // Initializes display
  void begin(void);

  // Polls
  void polling(const unsigned long time = millis());

  // Clears scroll line
  void clear(void);

  // Writes a byte
  virtual size_t write(uint8_t data);

  // Sets the error status
  void error(const uint8_t error);

  // Sets the halt status
  void connected(const boolean connected);

  // Sets the moving status
  void move(const boolean move);

  // Sets the block status
  void block(const block_t block);

  // Sets the activity instant
  void activity(const unsigned long time = millis());

  // Sets the distance
  void distance(const int distance);

  // Sets the supply level
  void supply(const int level);

  // Shows the wifi info
  void showWiFiInfo(const char* info);

  using Print::write;
};

#endif
