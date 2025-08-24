#ifndef _Camera_h_
#define _Camera_h_

/**
 * Send and receive text messages from a broker
 */
class Camera {
private:
  int _ledIntensity;
public:
  const esp_err_t begin(void);
  const esp_err_t capture(const size_t (*callback)(void *ctx, const size_t index, const byte *data, const size_t len), void *context);
  const esp_err_t frameSize(const framesize_t value);
  void ledIntensity(const int value) {
    _ledIntensity = value;
  }
  const int ledIntensity(void) const {
    return _ledIntensity;
  }

  void lightLed(const int intensity);

  void flashing(const int n, const int intensity = 255, const int on = 20, const int off = 80);
};

extern Camera camera;

#endif