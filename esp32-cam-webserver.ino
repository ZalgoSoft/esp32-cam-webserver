#include <esp_camera.h>
#include <esp_int_wdt.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <esp_log.h>
#include <freertos/FreeRTOSConfig.h>
#include "src/parsebytes.h"
#include "time.h"
#include <ESPmDNS.h>

#include <ESP32Servo.h>


/* This sketch is a extension/expansion/reork of the 'official' ESP32 Camera example
    sketch from Expressif:
    https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer

    It is modified to allow control of Illumination LED Lamps's (present on some modules),
    greater feedback via a status LED, and the HTML contents are present in plain text
    for easy modification.

    A camera name can now be configured, and wifi details can be stored in an optional
    header file to allow easier updated of the repo.

    The web UI has had changes to add the lamp control, rotation, a standalone viewer,
    more feeedback, new controls and other tweaks and changes,
   note: Make sure that you have either selected ESP32 AI Thinker,
         or another board which has PSRAM enabled to use high resolution camera modes
*/


/*
    FOR NETWORK AND HARDWARE SETTINGS COPY OR RENAME 'myconfig.sample.h' TO 'myconfig.h' AND EDIT THAT.

   By default this sketch will assume an AI-THINKER ESP-CAM and create
   an accesspoint called "ESP32-CAM-CONNECT" (password: "InsecurePassword")

*/
//#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
//esp_log_level_set("*", ESP_LOG_ERROR);
static const char* TAG = "main";
// Primary config, or defaults.
#if __has_include("myconfig.h")
struct station {
  const char ssid[65];
  const char password[65];
  const bool dhcp;
};  // do no edit
#include "myconfig.h"
#else
#warning "Using Defaults: Copy myconfig.sample.h to myconfig.h and edit that to use your own settings"
#define WIFI_AP_ENABLE
#define CAMERA_MODEL_AI_THINKER
struct station {
  const char ssid[65];
  const char password[65];
  const bool dhcp;
}
stationList[] = {{"ESP32-CAM-CONNECT", "InsecurePassword", true}};
#endif

//#include <TLog.h>      // The T-Logging library.
//#include <WebSerialStream.h>
//WebSerialStream webSerialStream = WebSerialStream(8514);
TaskHandle_t TaskCore0Handle;
TaskHandle_t asyncPos1Handle;
TaskHandle_t xHandle;
const TickType_t xDelay = 10;
// Upstream version string
#include "src/version.h"

// Pin Mappings
#include "camera_pins.h"

// Camera config structure
camera_config_t config;

// Internal filesystem (SPIFFS)
// used for non-volatile camera settings
#include "storage.h"

Servo servo1;
Servo servo2;
int Servo1Pin = 14;
int Servo2Pin  = 15;
int Servo_Step = 5;
int ptz_y = 120;
int ptz_x = 64;
int ptz_y_now = 120;
int ptz_x_now = 64;

// Sketch Info
int sketchSize;
int sketchSpace;
String sketchMD5;

// Start with accesspoint mode disabled, wifi setup will activate it if
// no known networks are found, and WIFI_AP_ENABLE has been defined
bool accesspoint = false;

// IP address, Netmask and Gateway, populated when connected
IPAddress ip;
IPAddress net;
IPAddress gw;

// Declare external function from app_httpd.cpp
extern void startCameraServer(int hPort, int sPort);
extern void serialDump();

// Names for the Camera. (set these in myconfig.h)
#if defined(CAM_NAME)
char myName[] = CAM_NAME;
#else
char myName[] = "ESP32 camera server";
#endif

#if defined(MDNS_NAME)
char mdnsName[] = MDNS_NAME;
#else
char mdnsName[] = "esp32-cam";
#endif

// Ports for http and stream (override in myconfig.h)
#if defined(HTTP_PORT)
int httpPort = HTTP_PORT;
#else
int httpPort = 80;
#endif

#if defined(STREAM_PORT)
int streamPort = STREAM_PORT;
#else
int streamPort = 81;
#endif

#if !defined(WIFI_WATCHDOG)
#define WIFI_WATCHDOG 15000
#endif

// Number of known networks in stationList[]
int stationCount = sizeof(stationList) / sizeof(stationList[0]);

// If we have AP mode enabled, ignore first entry in the stationList[]
#if defined(WIFI_AP_ENABLE)
int firstStation = 1;
#else
int firstStation = 0;
#endif

// Select between full and simple index as the default.
#if defined(DEFAULT_INDEX_FULL)
char default_index[] = "full";
#else
char default_index[] = "simple";
#endif

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;
bool captivePortal = false;
char apName[64] = "Undefined";

// The app and stream URLs
char httpURL[64] = {"Undefined"};
char streamURL[64] = {"Undefined"};

// Counters for info screens and debug
int8_t streamCount = 0;          // Number of currently active streams
unsigned long streamsServed = 0; // Total completed streams
unsigned long imagesServed = 0;  // Total image requests

// This will be displayed to identify the firmware
char myVer[] PROGMEM = __DATE__ " @ " __TIME__;

// This will be set to the sensors PID (identifier) during initialisation
//camera_pid_t sensorPID;
int sensorPID;

// Camera module bus communications frequency.
// Originally: config.xclk_freq_mhz = 20000000, but this lead to visual artifacts on many modules.
// See https://github.com/espressif/esp32-camera/issues/150#issuecomment-726473652 et al.
#if !defined (XCLK_FREQ_MHZ)
unsigned long xclk = 8;
#else
unsigned long xclk = XCLK_FREQ_MHZ;
#endif

// initial rotation
// can be set in myconfig.h
#if !defined(CAM_ROTATION)
#define CAM_ROTATION 0
#endif
int myRotation = CAM_ROTATION;

// minimal frame duration in ms, effectively 1/maxFPS
#if !defined(MIN_FRAME_TIME)
#define MIN_FRAME_TIME 0
#endif
int minFrameTime = MIN_FRAME_TIME;

// Illumination LAMP and status LED
#if defined(LAMP_DISABLE)
int lampVal = -1; // lamp is disabled in config
#elif defined(LAMP_PIN)
#if defined(LAMP_DEFAULT)
int lampVal = constrain(LAMP_DEFAULT, 0, 100); // initial lamp value, range 0-100
#else
int lampVal = 0; //default to off
#endif
#else
int lampVal = -1; // no lamp pin assigned
#endif

#if defined(LED_DISABLE)
#undef LED_PIN    // undefining this disables the notification LED
#endif

bool autoLamp = false;         // Automatic lamp (auto on while camera running)

int lampChannel = 7;           // a free PWM channel (some channels used by camera)
const int pwmfreq = 50000;     // 50K pwm frequency
const int pwmresolution = 9;   // duty cycle bit range
const int pwmMax = pow(2, pwmresolution) - 1;

#if defined(NO_FS)
bool filesystem = false;
#else
bool filesystem = true;
#endif

#if defined(NO_OTA)
bool otaEnabled = false;
#else
bool otaEnabled = true;
#endif

#if defined(OTA_PASSWORD)
char otaPassword[] = OTA_PASSWORD;
#else
char otaPassword[] = "";
#endif

#if defined(NTPSERVER)
bool haveTime = true;
const char* ntpServer = NTPSERVER;
const long  gmtOffset_sec = NTP_GMT_OFFSET;
const int   daylightOffset_sec = NTP_DST_OFFSET;
#else
bool haveTime = false;
const char* ntpServer = "";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0;
#endif

// Critical error string; if set during init (camera hardware failure) it
// will be returned for all http requests
String critERR = "";

// Debug flag for stream and capture data
bool debugData;

void debugOn() {
  debugData = true;
  ESP_LOGI(TAGLOG, "Camera debug data is enabled (send 'd' for status dump, or any other char to disable debug)");
}

void debugOff() {
  debugData = false;
  ESP_LOGI(TAGLOG, "Camera debug data is disabled (send 'd' for status dump, or any other char to enable debug)");
}

// Serial input (debugging controls)
void handleSerial() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'd' ) {
      serialDump();
    } else if (cmd == 'i' ) {
      ESP_LOGI(TAGLOG, "TaskCore0 start CPU ");
      ESP_LOGI(TAGLOG, xPortGetCoreID());
    }
    /*      else if (cmd == 'a' ) {
            ESP_LOGI(TAGLOG, "incPrio()");
            xHandle = xTaskGetCurrentTaskHandle();
            ESP_LOGI(TAGLOG, uxTaskPriorityGet( xHandle ));
            vTaskPrioritySet( xHandle  , uxTaskPriorityGet( xHandle) +1);
            ESP_LOGI(TAGLOG, uxTaskPriorityGet( xHandle ));
          }
          else if (cmd == 'z' ) {
            ESP_LOGI(TAGLOG, "decPrio()");
            xHandle = xTaskGetCurrentTaskHandle();
            ESP_LOGI(TAGLOG, uxTaskPriorityGet( xHandle ));
            vTaskPrioritySet( xHandle  , uxTaskPriorityGet( xHandle) -1);
            ESP_LOGI(TAGLOG, uxTaskPriorityGet( xHandle ));
          }
          else if (cmd == 'l' ) {
            ESP_LOGI(TAGLOG, "listtasks()");
            ESP_LOGI(TAGLOG,  "Task Name\tStatus\tPrio\tHWM\tTask\tAffinity\n");
            vTaskList(&pcwriteBuffer);
            ESP_LOGI(TAGLOG,  pcwriteBuffer);
          }*/
    else {
      if (debugData) debugOff();
      else debugOn();
    }
  }
  while (Serial.available()) Serial.read();  // chomp the buffer
}

// Notification LED
void flashLED(int flashtime) {
#if defined(LED_PIN)                // If we have it; flash it.
  digitalWrite(LED_PIN, LED_ON);  // On at full power.
  delay(flashtime);               // delay
  digitalWrite(LED_PIN, LED_OFF); // turn Off
#else
  return;                         // No notifcation LED, do nothing, no delay
#endif
}

// Lamp Control
void setLamp(int newVal) {
#if defined(LAMP_PIN)
  if (newVal != -1) {
    // Apply a logarithmic function to the scale.
    int brightness = round((pow(2, (1 + (newVal * 0.02))) - 2) / 6 * pwmMax);
    ledcWrite(lampChannel, brightness);
    ESP_LOGI(TAGLOG, "Lamp: ");
    ESP_LOGI(TAGLOG, newVal);
    ESP_LOGI(TAGLOG, "%, pwm = ");
    ESP_LOGI(TAGLOG, brightness);
  }
#endif
}

void printLocalTime(bool extraData = false) {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    ESP_LOGI(TAGLOG, "Failed to obtain time");
  } else {
    ESP_LOGI(TAGLOG, &timeinfo, "%H:%M:%S, %A, %B %d %Y");
  }
  if (extraData) {
    ESP_LOGI(TAGLOG, "NTP Server: %s, GMT Offset: %li(s), DST Offset: %i(s)\r\n", ntpServer, gmtOffset_sec, daylightOffset_sec);
  }
}

void TaskCore0 (void * pvParameters ) {
  ESP_LOGI(TAGLOG, "TaskCore0 start CPU ");
  ESP_LOGI(TAGLOG, xPortGetCoreID());
  // Start the camera server
  startCameraServer(httpPort, streamPort);
  if (critERR.length() == 0) {
    ESP_LOGI(TAGLOG, "\r\nCamera Ready!\r\nUse '%s' to connect\r\n", httpURL);
    ESP_LOGI(TAGLOG, "Stream viewer available at '%sview'\r\n", streamURL);
    ESP_LOGI(TAGLOG, "Raw stream URL is '%s'\r\n", streamURL);
#if defined(DEBUG_DEFAULT_ON)
    debugOn();
#else
    debugOff();
#endif
  } else {
    ESP_LOGI(TAGLOG, "\r\nCamera unavailable due to initialisation errors.\r\n\r\n");
  }
  // Info line; use for Info messages; eg 'This is a Beta!' warnings, etc. as necesscary
  ESP_LOGI(TAGLOG, "\r\nThis is the 4.1 beta\r\n");
  // do something every 5 seconds.
  static unsigned  long last_report = millis();
  for (;;) {
    //Log.loop();
    if (millis() - last_report < 7 * 1000)
    {
      taskYIELD( );
    }
    else {
      //Log.print (millis());
      //ESP_LOGI(TAGLOG, " CPU "); ESP_LOGI(TAGLOG, xPortGetCoreID());
      last_report = millis();
      //vTaskGetRunTimeStats( char *pcWriteBuffer );
      taskYIELD( );
    }
  }
}
void asyncPos (void * pvParameters ) {
  ESP_LOGI(TAGLOG, "asyncPos start CPU ");
  ESP_LOGI(TAGLOG, xPortGetCoreID());
  // Start the camera server
  // Info line; use for Info messages; eg 'This is a Beta!' warnings, etc. as necesscary
  ESP_LOGI(TAGLOG, "\r\nasyncPos thread.\r\n");
  // do something every 5 seconds.
  bool isYreached = false;
  bool isXreached = false;
  bool isPrefPosChanged = true;
  int periodtimer = 10;
  if ((int)pvParameters != 0)// || (int)pvParameters != NULL)
    periodtimer = (int)&pvParameters;
  static unsigned  long lastTime = millis();
  for (;;) {
    //Log.loop();
    if (millis() - lastTime < periodtimer)
    {
      taskYIELD( );
    }
    else {
      if (ptz_y_now != ptz_y) {
        isYreached = false;
        isPrefPosChanged = true;
        if (ptz_y < ptz_y_now) ptz_y_now--;
        else ptz_y_now ++;
        servo1.write(ptz_y_now);
      }
      else
        isYreached = true;
      if (ptz_x_now != ptz_x) {
        isXreached = false;
        isPrefPosChanged = true;
        if (ptz_x < ptz_x_now) ptz_x_now--;
        else ptz_x_now++;
        servo2.write(ptz_x_now);
      }
      else
        isXreached = true;
      if ( isYreached && isXreached && isPrefPosChanged )
      {
        saveposPrefs(SPIFFS);
        isPrefPosChanged = false;
      }
      lastTime = millis();
      taskYIELD( );
    }
  }
}

void calcURLs() {
  // Set the URL's
#if defined(URL_HOSTNAME)
  if (httpPort != 80) {
    sprintf(httpURL, "http://%s:%d/", URL_HOSTNAME, httpPort);
  } else {
    sprintf(httpURL, "http://%s/", URL_HOSTNAME);
  }
  sprintf(streamURL, "http://%s:%d/", URL_HOSTNAME, streamPort);
#else
  ESP_LOGI(TAGLOG, "Setting httpURL");
  if (httpPort != 80) {
    sprintf(httpURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], httpPort);
  } else {
    sprintf(httpURL, "http://%d.%d.%d.%d/", ip[0], ip[1], ip[2], ip[3]);
  }
  sprintf(streamURL, "http://%d.%d.%d.%d:%d/", ip[0], ip[1], ip[2], ip[3], streamPort);
#endif
}

void StartCamera() {
  ESP_LOGI(TAGLOG, "StartCamera CPU ");
  ESP_LOGI(TAGLOG, xPortGetCoreID());
  // Populate camera config structure with hardware and other defaults
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = xclk * 1000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  // Pre-allocate large buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    delay(100);  // need a delay here or the next serial o/p gets missed
    ESP_LOGI(TAGLOG, "\r\n\r\nCRITICAL FAILURE: Camera sensor failed to initialise.\r\n\r\n");
    ESP_LOGI(TAGLOG, "A full (hard, power off/on) reboot will probably be needed to recover from this.\r\n");
    ESP_LOGI(TAGLOG, "Meanwhile; this unit will reboot in 1 minute since these errors sometime clear automatically\r\n");
    // Reset the I2C bus.. may help when rebooting.
    periph_module_disable(PERIPH_I2C0_MODULE); // try to shut I2C down properly in case that is the problem
    periph_module_disable(PERIPH_I2C1_MODULE);
    periph_module_reset(PERIPH_I2C0_MODULE);
    periph_module_reset(PERIPH_I2C1_MODULE);
    // And set the error text for the UI
    critERR = "<h1>Error!</h1><hr><p>Camera module failed to initialise!</p><p>Please reset (power off/on) the camera.</p>";
    critERR += "<p>We will continue to reboot once per minute since this error sometimes clears automatically.</p>";
    // Start a 60 second watchdog timer
    esp_task_wdt_init(60, true);
    esp_task_wdt_add(NULL);
  } else {
    ESP_LOGI(TAGLOG, "Camera init succeeded");

    // Get a reference to the sensor
    sensor_t * s = esp_camera_sensor_get();

    // Dump camera module, warn for unsupported modules.
    sensorPID = s->id.PID;
    switch (sensorPID) {
      case OV9650_PID: ESP_LOGI(TAGLOG, "WARNING: OV9650 camera module is not properly supported, will fallback to OV2640 operation"); break;
      case OV7725_PID: ESP_LOGI(TAGLOG, "WARNING: OV7725 camera module is not properly supported, will fallback to OV2640 operation"); break;
      case OV2640_PID: ESP_LOGI(TAGLOG, "OV2640 camera module detected"); break;
      case OV3660_PID: ESP_LOGI(TAGLOG, "OV3660 camera module detected"); break;
      default: ESP_LOGI(TAGLOG, "WARNING: Camera module is unknown and not properly supported, will fallback to OV2640 operation");
    }

    // OV3660 initial sensors are flipped vertically and colors are a bit saturated
    if (sensorPID == OV3660_PID) {
      s->set_vflip(s, 1);  //flip it back
      s->set_brightness(s, 1);  //up the blightness just a bit
      s->set_saturation(s, -2);  //lower the saturation
    }

    // M5 Stack Wide has special needs
#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#endif

    // Config can override mirror and flip
#if defined(H_MIRROR)
    s->set_hmirror(s, H_MIRROR);
#endif
#if defined(V_FLIP)
    s->set_vflip(s, V_FLIP);
#endif

    // set initial frame rate
#if defined(DEFAULT_RESOLUTION)
    s->set_framesize(s, DEFAULT_RESOLUTION);
#else
    s->set_framesize(s, FRAMESIZE_SVGA);
#endif

    /*
      Add any other defaults you want to apply at startup here:
      uncomment the line and set the value as desired (see the comments)

      these are defined in the esp headers here:
      https://github.com/espressif/esp32-camera/blob/master/driver/include/sensor.h#L149
    */

    //s->set_framesize(s, FRAMESIZE_SVGA); // FRAMESIZE_[QQVGA|HQVGA|QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA|QXGA(ov3660)]);
    //s->set_quality(s, val);       // 10 to 63
    //s->set_brightness(s, 0);      // -2 to 2
    //s->set_contrast(s, 0);        // -2 to 2
    //s->set_saturation(s, 0);      // -2 to 2
    //s->set_special_effect(s, 0);  // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    //s->set_whitebal(s, 1);        // aka 'awb' in the UI; 0 = disable , 1 = enable
    //s->set_awb_gain(s, 1);        // 0 = disable , 1 = enable
    //s->set_wb_mode(s, 0);         // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    //s->set_exposure_ctrl(s, 1);   // 0 = disable , 1 = enable
    //s->set_aec2(s, 0);            // 0 = disable , 1 = enable
    //s->set_ae_level(s, 0);        // -2 to 2
    //s->set_aec_value(s, 300);     // 0 to 1200
    //s->set_gain_ctrl(s, 1);       // 0 = disable , 1 = enable
    //s->set_agc_gain(s, 0);        // 0 to 30
    //s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
    //s->set_bpc(s, 0);             // 0 = disable , 1 = enable
    //s->set_wpc(s, 1);             // 0 = disable , 1 = enable
    //s->set_raw_gma(s, 1);         // 0 = disable , 1 = enable
    //s->set_lenc(s, 1);            // 0 = disable , 1 = enable
    //s->set_hmirror(s, 0);         // 0 = disable , 1 = enable
    //s->set_vflip(s, 0);           // 0 = disable , 1 = enable
    //s->set_dcw(s, 1);             // 0 = disable , 1 = enable
    //s->set_colorbar(s, 0);        // 0 = disable , 1 = enable
  }
  // We now have camera with default init
}

void WifiSetup() {
  // Feedback that we are now attempting to connect
  flashLED(300);
  delay(100);
  flashLED(300);
  ESP_LOGI(TAGLOG, "Starting WiFi");

  // Disable power saving on WiFi to improve responsiveness
  // (https://github.com/espressif/arduino-esp32/issues/1484)
  WiFi.setSleep(false);

  ESP_LOGI(TAGLOG, "Known external SSIDs: ");
  if (stationCount > firstStation) {
    for (int i = firstStation; i < stationCount; i++) ESP_LOGI(TAGLOG, " '%s'", stationList[i].ssid);
  } else {
    ESP_LOGI(TAGLOG, "None");
  }
  //ESP_LOGI(TAGLOG, );
  byte mac[6] = {0, 0, 0, 0, 0, 0};
  WiFi.macAddress(mac);
  ESP_LOGI(TAGLOG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  int bestStation = -1;
  long bestRSSI = -1024;
  char bestSSID[65] = "";
  uint8_t bestBSSID[6];
  if (stationCount > firstStation) {
    // We have a list to scan
    ESP_LOGI(TAGLOG, "Scanning local Wifi Networks\r\n");
    int stationsFound = WiFi.scanNetworks();
    ESP_LOGI(TAGLOG, "%i networks found\r\n", stationsFound);
    if (stationsFound > 0) {
      for (int i = 0; i < stationsFound; ++i) {
        // Print SSID and RSSI for each network found
        String thisSSID = WiFi.SSID(i);
        int thisRSSI = WiFi.RSSI(i);
        String thisBSSID = WiFi.BSSIDstr(i);
        ESP_LOGI(TAGLOG, "%3i : [%s] %s (%i)", i + 1, thisBSSID.c_str(), thisSSID.c_str(), thisRSSI);
        // Scan our list of known external stations
        for (int sta = firstStation; sta < stationCount; sta++) {
          if ((strcmp(stationList[sta].ssid, thisSSID.c_str()) == 0) ||
              (strcmp(stationList[sta].ssid, thisBSSID.c_str()) == 0)) {
            ESP_LOGI(TAGLOG, "  -  Known!");
            // Chose the strongest RSSI seen
            if (thisRSSI > bestRSSI) {
              bestStation = sta;
              strncpy(bestSSID, thisSSID.c_str(), 64);
              // Convert char bssid[] to a byte array
              parseBytes(thisBSSID.c_str(), ':', bestBSSID, 6, 16);
              bestRSSI = thisRSSI;
            }
          }
        }
        //ESP_LOGI(TAGLOG, );
      }
    }
  } else {
    // No list to scan, therefore we are an accesspoint
    accesspoint = true;
  }

  if (bestStation == -1) {
    if (!accesspoint) {
#if defined(WIFI_AP_ENABLE)
      ESP_LOGI(TAGLOG, "No known networks found, entering AccessPoint fallback mode");
      accesspoint = true;
#else
      ESP_LOGI(TAGLOG, "No known networks found");
#endif
    } else {
      ESP_LOGI(TAGLOG, "AccessPoint mode selected in config");
    }
  } else {
    ESP_LOGI(TAGLOG, "Connecting to Wifi Network %d: [%02X:%02X:%02X:%02X:%02X:%02X] %s \r\n",
             bestStation, bestBSSID[0], bestBSSID[1], bestBSSID[2], bestBSSID[3],
             bestBSSID[4], bestBSSID[5], bestSSID);
    // Apply static settings if necesscary
    if (stationList[bestStation].dhcp == false) {
#if defined(ST_IP)
      ESP_LOGI(TAGLOG, "Applying static IP settings");
#if !defined (ST_GATEWAY)  || !defined (ST_NETMASK)
#error "You must supply both Gateway and NetMask when specifying a static IP address"
#endif
      IPAddress staticIP(ST_IP);
      IPAddress gateway(ST_GATEWAY);
      IPAddress subnet(ST_NETMASK);
#if !defined(ST_DNS1)
      WiFi.config(staticIP, gateway, subnet);
#else
      IPAddress dns1(ST_DNS1);
#if !defined(ST_DNS2)
      WiFi.config(staticIP, gateway, subnet, dns1);
#else
      IPAddress dns2(ST_DNS2);
      WiFi.config(staticIP, gateway, subnet, dns1, dns2);
#endif
#endif
#else
      ESP_LOGI(TAGLOG, "Static IP settings requested but not defined in config, falling back to dhcp");
#endif
    }

    WiFi.setHostname(mdnsName);

    // Initiate network connection request (3rd argument, channel = 0 is 'auto')
    WiFi.begin(bestSSID, stationList[bestStation].password, 0, bestBSSID);

    // Wait to connect, or timeout
    unsigned long start = millis();
    while ((millis() - start <= WIFI_WATCHDOG) && (WiFi.status() != WL_CONNECTED)) {
      delay(500);
      ESP_LOGI(TAGLOG, '.');
    }
    //ESP_LOGI(TAGLOG, );
    //Log.begin();
    // If we have connected, inform user
    if (WiFi.status() == WL_CONNECTED) {
      ESP_LOGI(TAGLOG, "Client connection succeeded");
      accesspoint = false;
      // Note IP details
      ip = WiFi.localIP();
      net = WiFi.subnetMask();
      gw = WiFi.gatewayIP();
      ESP_LOGI(TAGLOG, "IP address: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
      ESP_LOGI(TAGLOG, "Netmask   : %d.%d.%d.%d\r\n", net[0], net[1], net[2], net[3]);
      ESP_LOGI(TAGLOG, "Gateway   : %d.%d.%d.%d\r\n", gw[0], gw[1], gw[2], gw[3]);
      calcURLs();
      // Flash the LED to show we are connected
      for (int i = 0; i < 5; i++) {
        flashLED(50);
        delay(150);
      }
    } else {
      ESP_LOGI(TAGLOG, "Client connection Failed");
      WiFi.disconnect();   // (resets the WiFi scan)
    }
  }

  if (accesspoint && (WiFi.status() != WL_CONNECTED)) {
    // The accesspoint has been enabled, and we have not connected to any existing networks
#if defined(AP_CHAN)
    ESP_LOGI(TAGLOG, "Setting up Fixed Channel AccessPoint");
    ESP_LOGI(TAGLOG, "  SSID     : ");
    ESP_LOGI(TAGLOG, stationList[0].ssid);
    ESP_LOGI(TAGLOG, "  Password : ");
    ESP_LOGI(TAGLOG, stationList[0].password);
    ESP_LOGI(TAGLOG, "  Channel  : ");
    ESP_LOGI(TAGLOG, AP_CHAN);
    WiFi.softAP(stationList[0].ssid, stationList[0].password, AP_CHAN);
# else
    ESP_LOGI(TAGLOG, "Setting up AccessPoint");
    ESP_LOGI(TAGLOG, "  SSID     : ");
    ESP_LOGI(TAGLOG, stationList[0].ssid);
    ESP_LOGI(TAGLOG, "  Password : ");
    ESP_LOGI(TAGLOG, stationList[0].password);
    WiFi.softAP(stationList[0].ssid, stationList[0].password);
#endif
#if defined(AP_ADDRESS)
    // User has specified the AP details; apply them after a short delay
    // (https://github.com/espressif/arduino-esp32/issues/985#issuecomment-359157428)
    delay(100);
    IPAddress local_IP(AP_ADDRESS);
    IPAddress gateway(AP_ADDRESS);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);
#endif
    // Note AP details
    ip = WiFi.softAPIP();
    net = WiFi.subnetMask();
    gw = WiFi.gatewayIP();
    strcpy(apName, stationList[0].ssid);
    ESP_LOGI(TAGLOG, "IP address: %d.%d.%d.%d\r\n", ip[0], ip[1], ip[2], ip[3]);
    calcURLs();
    // Flash the LED to show we are connected
    for (int i = 0; i < 5; i++) {
      flashLED(150);
      delay(50);
    }
    // Start the DNS captive portal if requested
    if (stationList[0].dhcp == true) {
      ESP_LOGI(TAGLOG, "Starting Captive Portal");
      dnsServer.start(DNS_PORT, "*", ip);
      captivePortal = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  //Log.addPrintStream(std::make_shared<WebSerialStream>(webSerialStream));
  if (stationCount == 0) {
    ESP_LOGI(TAGLOG, "\r\nFatal Error; Halting");
    while (true) {
      ESP_LOGI(TAGLOG, "No wifi details have been configured; we cannot connect to existing WiFi or start our own AccessPoint, there is no point in proceeding.");
      delay(5000);
    }
  }
  // Start Wifi and loop until we are connected or have started an AccessPoint
  while ((WiFi.status() != WL_CONNECTED) && !accesspoint)  {
    WifiSetup();
  }
  //Log.begin();
  ESP_LOGI(TAGLOG, "setup: Log.begin(), CPU ");
  ESP_LOGI(TAGLOG, xPortGetCoreID());
  //ESP_LOGI(TAGLOG, );
  ESP_LOGI(TAGLOG, "====");
  ESP_LOGI(TAGLOG, "esp32-cam-webserver: ");
  ESP_LOGI(TAGLOG, myName);
  ESP_LOGI(TAGLOG, "Code Built: ");
  ESP_LOGI(TAGLOG, myVer);
  ESP_LOGI(TAGLOG, "Base Release: ");
  ESP_LOGI(TAGLOG, baseVersion);
  //ESP_LOGI(TAGLOG, );
  // Warn if no PSRAM is detected (typically user error with board selection in the IDE)
  if (!psramFound()) {
    ESP_LOGI(TAGLOG, "\r\nFatal Error; Halting");
    while (true) {
      ESP_LOGI(TAGLOG, "No PSRAM found; camera cannot be initialised: Please check the board config for your module.");
      delay(5000);
    }
  }

#if defined(LED_PIN)  // If we have a notification LED, set it to output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_ON);
#endif

  // Start the SPIFFS filesystem before we initialise the camera
  if (filesystem) {
    filesystemStart();
    delay(200); // a short delay to let spi bus settle after SPIFFS init
  }

  // Start (init) the camera
  StartCamera();

  // Now load and apply any saved preferences
  if (filesystem) {
    delay(200); // a short delay to let spi bus settle after camera init
    loadPrefs(SPIFFS);
    loadposPrefs(SPIFFS);
  } else {
    ESP_LOGI(TAGLOG, "No Internal Filesystem, cannot load or save preferences");
  }

  /*
    Camera setup complete; initialise the rest of the hardware.
  */

  servo1.attach(Servo1Pin);
  servo2.attach(Servo2Pin);

  // Set up OTA
  if (otaEnabled) {
    // Start OTA once connected
    ESP_LOGI(TAGLOG, "Setting up OTA");
    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);
    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname(mdnsName);
    // No authentication by default
    if (strlen(otaPassword) != 0) {
      ArduinoOTA.setPassword(otaPassword);
      ESP_LOGI(TAGLOG, "OTA Password: %s\n\r", otaPassword);
    } else {
      ESP_LOGI(TAGLOG, "\r\nNo OTA password has been set! (insecure)\r\n\r\n");
    }
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        type = "filesystem";
      ESP_LOGI(TAGLOG, "Start updating " + type);
      ESP_LOGI(TAGLOG, "setup ArduinoOTA.onStart CPU ");
      ESP_LOGI(TAGLOG, xPortGetCoreID());
      // Stop the camera since OTA will crash the module if it is running.
      // the unit will need rebooting to restart it, either by OTA on success, or manually by the user
      ESP_LOGI(TAGLOG, "Stopping Camera");
      esp_err_t err = esp_camera_deinit();
      critERR = "<h1>OTA Has been started</h1><hr><p>Camera has Halted!</p>";
      critERR += "<p>Wait for OTA to finish and reboot, or <a href=\"control?var=reboot&val=0\" title=\"Reboot Now (may interrupt OTA)\">reboot manually</a> to recover</p>";
    })
    .onEnd([]() {
      ESP_LOGI(TAGLOG, "\r\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      ESP_LOGI(TAGLOG, "Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      ESP_LOGI(TAGLOG, "Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) ESP_LOGI(TAGLOG, "Auth Failed");
      else if (error == OTA_BEGIN_ERROR) ESP_LOGI(TAGLOG, "Begin Failed");
      else if (error == OTA_CONNECT_ERROR) ESP_LOGI(TAGLOG, "Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) ESP_LOGI(TAGLOG, "Receive Failed");
      else if (error == OTA_END_ERROR) ESP_LOGI(TAGLOG, "End Failed");
    });
    ArduinoOTA.begin();
  } else {
    ESP_LOGI(TAGLOG, "OTA is disabled");

    if (!MDNS.begin(mdnsName)) {
      ESP_LOGI(TAGLOG, "Error setting up MDNS responder!");
    }
    ESP_LOGI(TAGLOG, "mDNS responder started");
  }

  //MDNS Config -- note that if OTA is NOT enabled this needs prior steps!
  MDNS.addService("http", "tcp", 80);
  ESP_LOGI(TAGLOG, "Added HTTP service to MDNS server");

  MDNS.addService("webserial", "tcp", 8514);
  ESP_LOGI(TAGLOG, "Added webserial service to MDNS server");

  // Set time via NTP server when enabled
  if (haveTime) {
    ESP_LOGI(TAGLOG, "Time: ");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime(true);
  } else {
    ESP_LOGI(TAGLOG, "Time functions disabled");
  }

  // Gather static values used when dumping status; these are slow functions, so just do them once during startup
  sketchSize = ESP.getSketchSize();
  sketchSpace = ESP.getFreeSketchSpace();
  sketchMD5 = ESP.getSketchMD5();

  // Initialise and set the lamp
  if (lampVal != -1) {
#if defined(LAMP_PIN)
    ledcSetup(lampChannel, pwmfreq, pwmresolution);  // configure LED PWM channel
    ledcAttachPin(LAMP_PIN, lampChannel);            // attach the GPIO pin to the channel
    if (autoLamp) setLamp(0);                        // set default value
    else setLamp(lampVal);
#endif
  } else {
    ESP_LOGI(TAGLOG, "No lamp, or lamp disabled in config");
  }


  // As a final init step chomp out the serial buffer in case we have recieved mis-keys or garbage during startup
  while (Serial.available()) Serial.read();
  xTaskCreatePinnedToCore(TaskCore0, "TaskCore0",  1000,  NULL,  tskIDLE_PRIORITY + 1, &TaskCore0Handle,  0);
  int pvPeriodtimer = 10;
  xTaskCreatePinnedToCore(asyncPos,  "asyncPos",   1000,  ( void* ) &pvPeriodtimer,  tskIDLE_PRIORITY, &asyncPos1Handle,  1);
}


void loop() {
  /*
      Just loop forever, reconnecting Wifi As necesscary in client mode
     The stream and URI handler processes initiated by the startCameraServer() call at the
     end of setup() will handle the camera and UI processing from now on.
  */
  if (accesspoint) {
    // Accespoint is permanently up, so just loop, servicing the captive portal as needed
    // Rather than loop forever, follow the watchdog, in case we later add auto re-scan.
    unsigned long start = millis();
    while (millis() - start < WIFI_WATCHDOG ) {
      //delay(100);
      vTaskDelay( xDelay );
      if (otaEnabled) ArduinoOTA.handle();
      handleSerial();
      if (captivePortal) dnsServer.processNextRequest();
    }
  } else {
    // client mode can fail; so reconnect as appropriate
    static bool warned = false;
    if (WiFi.status() == WL_CONNECTED) {
      // We are connected, wait a bit and re-check
      if (warned) {
        // Tell the user if we have just reconnected
        ESP_LOGI(TAGLOG, "WiFi reconnected");
        warned = false;
      }
      // loop here for WIFI_WATCHDOG, turning debugData true/false depending on serial input..
      unsigned long start = millis();
      while (millis() - start < WIFI_WATCHDOG ) {
        delay(100);
        if (otaEnabled) ArduinoOTA.handle();
        handleSerial();
      }
    } else {
      // disconnected; attempt to reconnect
      if (!warned) {
        ESP_LOGI(TAGLOG, "loop CPU ");
        ESP_LOGI(TAGLOG, xPortGetCoreID());
        // Tell the user if we just disconnected
        WiFi.disconnect();  // ensures disconnect is complete, wifi scan cleared
        ESP_LOGI(TAGLOG, "WiFi disconnected, retrying");
        warned = true;
      }
      WifiSetup();
    }
  }
}
