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

#include <esp_log.h>
static const char *TAG = "WebServer";

#include <Arduino.h>
#include <esp_http_server.h>
#include <WiFi.h>

#include "WebServer.h"
#include "Camera.h"

static char baseUrl[64];

/**
 * @struct send_frame_context_t
 * @brief Context structure for streaming JPEG data chunks over HTTP.
 * 
 * This structure encapsulates the active ESP-IDF HTTP request framework handle 
 * along with a running total of the processed payload length. It is passed 
 * as an opaque state argument (`void *arg`) into the `frame2jpg_cb` encoder 
 * pipeline to maintain state throughout the image compression stream.
 */
typedef struct {
  httpd_req_t *req;
  size_t len;
  unsigned long start;
} send_frame_context_t;

static httpd_handle_t camera_httpd = NULL;

const char *webUrl(void) {
  return baseUrl;
}

static size_t sendFrame(void *context, capture_data_t *data) {
  send_frame_context_t *j = (send_frame_context_t *)context;
  size_t len = 0;
  esp_err_t res;
  unsigned long captureTime = millis();
  switch (data->type) {
    case SINGLE_FRAME:
      res = httpd_resp_send(j->req, (const char *)data->data, data->len);
      ESP_LOGI(TAG, "Captured in %ul ms, sent in %ul ms", millis() - captureTime, captureTime - j->start);
      if (res == ESP_OK) {
        len = data->len;
      } else {
        ESP_LOGE(TAG, "Error sending response %xd", res);
      }
      break;
    default:
      ESP_LOGE(TAG, "Chunk not supported");
      break;
  }
  return j->len = len;
}

static esp_err_t parseIntQueryString(const char *queryString, const char *key, int &value, int min, int max) {
  char valueStr[16] = { 0 };
  esp_err_t res = httpd_query_key_value(queryString, key, valueStr, sizeof(valueStr));
  if (res == ESP_ERR_NOT_FOUND) {
    return ESP_OK;
  }
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Unable to query key value [%s]", key);
    return res;
  }
  sscanf(valueStr, "%d", &value);
  value = constrain(value, min, max);
  return ESP_OK;
}

static esp_err_t retrieveQueryString(httpd_req_t *req, char **obuf) {
  char *buf = NULL;
  size_t buf_len = 0;

  *obuf = NULL;
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);
    if (!buf) {
      return ESP_FAIL;
    }
    esp_err_t res = httpd_req_get_url_query_str(req, buf, buf_len);
    if (res != ESP_OK) {
      ESP_LOGE(TAG, "Unable to get query string %d", res);
      free(buf);
      return res;
    }
    *obuf = buf;
    return ESP_OK;
  }
  ESP_LOGI(TAG, "empy query");
  return ESP_OK;
}

static esp_err_t parseForProperties(httpd_req_t *req, image_properties_t &props) {
  // Get query string
  char *queryString = NULL;
  esp_err_t res = retrieveQueryString(req, &queryString);
  if (res != ESP_OK) {
    return res;
  }
  if (queryString == NULL) {
    // query string empty
    return ESP_OK;
  }

  // Parse properties
  res = parseIntQueryString(queryString, "l", props.flashIntensity, 0, CONFIG_LED_MAX_INTENSITY);
  if (res != ESP_OK) {
    free(queryString);
    return res;
  }
  res = parseIntQueryString(queryString, "f", props.frameSize, FRAMESIZE_QVGA, FRAMESIZE_UXGA);
  if (res != ESP_OK) {
    free(queryString);
    return res;
  }
  res = parseIntQueryString(queryString, "q", props.jpgQuality, 0, 100);
  if (res != ESP_OK) {
    free(queryString);
    return res;
  }
  res = parseIntQueryString(queryString, "v", props.vflip, 0, 1);
  if (res != ESP_OK) {
    free(queryString);
    return res;
  }
  res = parseIntQueryString(queryString, "h", props.hmirror, 0, 1);
  if (res != ESP_OK) {
    free(queryString);
    return res;
  }
  res = parseIntQueryString(queryString, "b", props.brightness, -2, 2);
  if (res != ESP_OK) {
    free(queryString);
    return res;
  }
  res = parseIntQueryString(queryString, "c", props.contrast, -2, 2);
  if (res != ESP_OK) {
    free(queryString);
    return res;
  }
  res = parseIntQueryString(queryString, "s", props.saturation, -2, 2);
  if (res != ESP_OK) {
    free(queryString);
    return res;
  }
  free(queryString);
  return ESP_OK;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  ESP_LOGI(TAG, "Capturing image");

  // Sets default values
  image_properties_t props = {
    .frameSize = FRAMESIZE_SVGA,
    .vflip = 0,
    .hmirror = 0,
    .brightness = 0,
    .contrast = 0,
    .saturation = 0,
    .flashIntensity = 0,
    .jpgQuality = 80
  };

  // Parses query string
  esp_err_t res = parseForProperties(req, props);
  if (res != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unable to parse query string");
    return res;
  }
  // sets the headers
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  struct timeval tv;
  gettimeofday(&tv, NULL);
  char ts[32];
  snprintf(ts, sizeof(ts), "%lld.%06ld", (long long)tv.tv_sec, (long)tv.tv_usec);
  httpd_resp_set_hdr(req, "X-Timestamp", ts);

  // Capture image
  send_frame_context_t sendFrameCtx = { req, 0, millis() };
  res = cameraCapture(sendFrame, &sendFrameCtx, props);
  if (res != ESP_OK) {
    ESP_LOGE(TAG, "Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Sent %ld B", sendFrameCtx.len);
  return res;
}

const esp_err_t startWebServer(void) {
  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;
  esp_err_t res = httpd_start(&camera_httpd, &config);
  ESP_LOGI(TAG, "Starting web server");
  if (res == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    snprintf(baseUrl, sizeof(baseUrl), "http://%s:%d", WiFi.localIP().toString().c_str(), config.server_port);
    ESP_LOGI(TAG, "Web server started");
    ESP_LOGI(TAG, "Use '%s' to connect", webUrl());
  } else {
    ESP_LOGE(TAG, "Error on httpd start %xd", res);
  }
  return res;
}
