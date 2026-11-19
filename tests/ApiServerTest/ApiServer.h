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

#ifndef ApiServer_h
#define ApiServer_h

#include <WebServer.h>

/*
   Handles the api server used to manage wifi module configuration
*/
class ApiServerClass;

typedef void (*onActivity_cb_t)(void* context, ApiServerClass& server);

class ApiServerClass {
public:
  /*
       Creates api server
    */
  ApiServerClass() {}

  /*
       Initializes api server
    */
  void begin(const char* wheellyId);

  /**
  * Starts the api server (must be call after the wifi connection)
  */
  void start(void);

  /*
       Sets callback on activity
    */
  void onActivity(onActivity_cb_t callback, void* context = NULL);
  /*
   Reloads the network list if required
  */
  void polling(const unsigned long t0 = millis());
};

extern ApiServerClass ApiServer;

#endif
