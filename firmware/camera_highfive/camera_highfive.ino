// High-Five Robot camera: Espressif's CameraWebServer example, set up for the XIAO ESP32S3 Sense with the wide OV2640.
// Boots straight into the picture tools/highfive.py expects (VGA, vflip on, no mirror; see tools/cam_snap.py) and
// announces itself as highfive-cam.local, so the Mac finds it whatever address the router hands out.
#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
// Wi-Fi details come from wifi_secrets.h, generated from the project .env by make_secrets.py (never committed)
#include "wifi_secrets.h"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

void startCameraServer();
void setupLedFlash();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // the high-five picture: VGA (4:3, the full view), then the flips (a framesize change can reset them)
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_vflip(s, 1);                  // CAM_VFLIP in tools/cam_snap.py (OV2640 in the v5 mount)
  s->set_hmirror(s, 0);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  WiFi.setHostname("highfive-cam");
  WiFi.setAutoReconnect(true);
  delay(1500 + esp_random() % 3000);         // the arm powers up at the same moment: don't ask the router together
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  uint32_t tries = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - tries > 20000) {        // the router refused us, say: start the join over instead of waiting forever
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      tries = millis();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();
  if (MDNS.begin("highfive-cam")) MDNS.addService("http", "tcp", 80);

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // Everything else is done in another task by the web server. Here: if Wi-Fi has been gone for a minute and
  // auto-reconnect hasn't brought it back, restart the board (a clean start always rejoins)
  static uint32_t lost = 0;
  if (WiFi.status() == WL_CONNECTED) lost = 0;
  else if (!lost) lost = millis();
  else if (millis() - lost > 60000) ESP.restart();
  delay(1000);
}
