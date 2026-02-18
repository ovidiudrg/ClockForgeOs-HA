/*
      ClockForgeOS v1.xx - Smart clock firmware for Nixie, VFD, LED, and Numitron displays on ESP8266 or ESP32
      Supports Nixie, VFD, LED, and Numitron displays on ESP8266 or ESP32
      Featuring optional Dallas Thermometer, DS3231 RTC, Neopixel integration, GPS, and more...
      This program is free software: you can redistribute it and/or modify
      it under the terms of the GNU General Public License as published by
      the Free Software Foundation, either version 3 of the License, or
      (at your option) any later version.

      This program is distributed in the hope that it will be useful,
      but WITHOUT ANY WARRANTY; without even the implied warranty of
      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
      GNU General Public License <http://www.gnu.org/licenses/> for more details.

      PLATFORM CONFIGURATION:
      - Reference ESP8266 GPIO pinout: https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
      - Reference ESP32 GPIO pinout: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
      - ESP32 Setup Guide: https://lastminuteengineers.com/esp32-arduino-ide-tutorial/

      FILESYSTEM DEPLOYMENT:
      Web interface files located in the "data" folder must be uploaded to SPIFFS:
      - ESP8266 Filesystem Uploader: https://randomnerdtutorials.com/install-esp8266-filesystem-uploader-arduino-ide/
      - ESP32 Filesystem Uploader: https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/
*/

// ============================= FORWARD DECLARATIONS =============================
// Function declarations for cross-file usage
void doAnimationMakuna();
void writeDisplay2();
void getLightSensor();
void doAnimationPWM();
void alarmSound();
void doReset();
void factoryReset();
void updateRTC();
void saveRGBSettingsToEEPROM();
void setupGestureSensor();
void processGestureSensor();
bool isGestureSensorPresent();
void executeGestureMappedAction(uint8_t direction);
void cycleFixedColorPalette(int step);
bool isPrintableString(const String& s);

// ============================= INCLUDES =============================
#include <stddef.h>
//#define byte uint8_t

// ClockForgeOS Configuration Files
#include "clocks.h"    // REQUIRED: Define your clock setup in this file (e.g., CLOCK_64)
#include "settings.h"  // REQUIRED: Application settings and configuration

// Platform-specific includes
#if defined(ESP32)
  #include "driver/gpio.h"
  #include "esp_system.h"
#endif

// ============================= PLATFORM DETECTION =============================
#if defined(ESP8266)
  #if defined(ARDUINO_ESP8266_RELEASE_2_6_0) || \
    defined(ARDUINO_ESP8266_RELEASE_2_6_1) || \
    defined(ARDUINO_ESP8266_RELEASE_2_6_2) || \
    defined(ARDUINO_ESP8266_RELEASE_2_6_3) || \
    defined(ARDUINO_ESP8266_RELEASE_2_7_0) || \
    defined(ARDUINO_ESP8266_RELEASE_2_7_1) || \
    defined(ARDUINO_ESP8266_RELEASE_2_7_2) || \
    defined(ARDUINO_ESP8266_RELEASE_2_7_3) || \
    defined(ARDUINO_ESP8266_RELEASE_2_7_4)
    #define ESP8266_CORE_2xx
    #pragma message "8266_Core 2.xx"
  #else
    #pragma message "8266_Core_3.xx"
    #define ICACHE_RAM_ATTR IRAM_ATTR
  #endif
#endif
/*_______________________________ USABLE PARAMETERS ____________________________________________
  #define DEBUG  //Enable Serial Monitor, 115200baud (only, if TX pin is not used anywhere!!!)
  //---------------------------- CLOCK EXTRA OPTION PARAMETERS ---------------------------------
  //#define USE_DALLAS_TEMP //TEMP_DALLAS_PIN is used to connect DS18B20 temperature sensors
  //#define USE_DHT_TEMP    //TEMP_DHT_PIN is sensor pin 
  //#define USE_BME280      //I2C Temperature + humidity + pressure, SDA+SCL I2C pins are used!   
  //#define USE_BMP280      //I2C Temperature + barometric pressure, SDA+SCL I2C pins are used!   
  //#define USE_AHTX0       //I2C Temperature + humidity, SDA+SCL I2C pins are used!   
  //#define USE_SHT21       //I2C Temperature + humidity, SDA+SCL I2C pins are used!   
  //#define USE_BH1750      //I2C Luxmeter sensor, SDA+SCL I2C pins are used!   
  //#define USE_RTC         //DS3231 realtime clock, SDA+SCL I2C pins are used!   
  //#define USE_APDS9960_GESTURE //APDS-9960 gesture sensor (I2C) for hand up/down/left/right actions
  //#define USE_GPS         //use for standalone clock, without wifi internet access
  //#define USE_NEOPIXEL    //WS2812B led stripe, for tubes backlight. Don't forget to define tubePixels[] !
  //#define USE_PWMLEDS     //WWM led driver on 3 pins for RG
  //#define USE_MQTT        //Home Assistant integration: https://www.home-assistant.io/
  //#define USE_WIFIMANAGER //if no WiFi defined, use Wifimanager
  //----- DRIVER SELECTION ------ Use only 1 driver from the following options in the clocks.h file!
  //#define MULTIPLEX74141_ESP32  //4..8 Nixie tubes generic driver for ESP32
  //#define MAX6921_ESP32         //4..8 VFD tubes (IV18) driver for ESP8232
  //#define HV5122                //4..8 Nixie driver - development in progress
  //-------------- 8266 clock drivers --------------------------------------------------
  //#define MULTIPLEX74141        //4..8 Nixie tubes generic driver for ESP8266
  //#define MAX6921               //4..8 VFD tubes (IV18) driver for ESP8266
  //#define NO_MULTIPLEX74141     //4..6 Nixie tubes, serial latch driver, 74141 for each tube
  //#define MM5450                //6..8 LEDS
  //#define MAX7219CNG            //4..8 LED
  //#define Numitron_4511N        //Numitron 4x tube clock
  //#define SN75512               //4..8 VFD tubes
  //#define samsung               //samsung serial display
  //#define PCF_74141             //PCF pin expander for tube selection
  //#define PT6355                //VFD clock - development in progress

  //--------------------- PINOUT & PIN PARAMETERS ---------------------------------------
  //#define PIN_SDA xx             // you can set the used SDA and SCL pins
  //#define PIN_SCL xx             // if it is not default value
  //#define COLON_PIN   -1        //Blinking Colon pin.  If not used, SET TO -1
  //#define TEMP_DALLAS_PIN -1    //Dallas DS18B20 temp sensor pin.  If not used, SET TO -1
  //#define TEMP_DHT_PIN -1       //DHT temp sensor pin.  If not used, SET TO -1
  //#define DHTTYPE DHT22         //DHT sensor type, if used...
  //#define LED_SWITCH_PIN -1     //external led backlight ON/OFF.  If not used, SET TO -1
  //#define DECIMALPOINT_PIN -1   //Nixie decimal point between digits. If not used, SET TO -1
  //#define ALARMSPEAKER_PIN -1   //Alarm buzzer pin
  //#define ALARMBUTTON_PIN -1    //Alarm switch off button pin
  //#define ALARM_ON HIGH         //HIGH or LOW level is needed to switch ON the buzzer?
  //#define NEOPIXEL_PIN 3        //8266 Neopixel LEDstripe pin is always the RX pin!!!
  //#define RGB_MIN_BRIGHTNESS 8   //Neopixel leds minimum brightness
  //#define RGB_MAX_BRIGHTNESS 255 //Neopixel leds maximum brightness
  //#define RADAR_PIN -1          //Radar sensor pin
  //#define RADAR_TIMEOUT 5     //Automatic switch off tubes (without radar detecting somebody) after xxx min
  //#define TUBE_POWER_PIN -1     //Filament or HV switch ON/OFF pin
  //#define TUBE_POWER_ON HIGH    //HIGH or LOW level is needed to switch ON the TUBE POWER?
  //#define LIGHT_SENSOR_PIN -1   //Environment light sensor, only ADC pins are usable! ESP32 for example: 34,35,36,39... 8266: only A0
  //#define REF_RESISTANCE    10000.0         // Resistor value is 10k, between LDR sensor and GND
  //#define MAXIMUM_LUX 100                   //Lux level for maximum tube brightness
  //#define LUX_CALC_SCALAR   12518931 * 1.2  //Calibrate here the LDR sensor
  //#define LUX_CALC_EXPONENT -1.405          //LDR sensor characteristic
  //#define PWM1_PIN -1  //red   channel if USE_PWMLEDS is defined
  //#define PWM2_PIN -1  //green channel if USE_PWMLEDS is defined
  //#define PWM3_PIN -1  //blue  channel if USE_PWMLEDS is defined
  //#define MAXBRIGHTNESS 10      //Do not change this value!

  //Display temperature and date in every minute between START..END seconds
  //#define ENABLE_CLOCK_DISPLAY true  //false, if no clock display is needed (for example: thermometer + humidity only)
  //#define SHIFT_TUBES_LEFT_BY_1   //shift leftIP address by 1 tube the display, if a thermometer is used with spec tube
  //#define LEFTDECIMAL false      //set true (Z574M), if decimal point is on the left side on the tube. Else set false (Z573M)!
  //#define DATE_REPEAT_MIN   1    //show date only every xxx minute. If zero, datum is never displayed!  
  //#define DOUBLE_BLINK          //both separator points are blinking (6 or 8 tubes VFD clock)
  //#define TEMP_START  35        //Temperature display start..end
  //#define TEMP_END    40
  //#define HUMID_START 40        //Humidity% display start..end
  //#define HUMID_END   45
  //#define DATE_START  45        //Date is displayed start..end
  //#define DATE_END    50
  //#define ANIMSPEED   50        //Animation speed in millisec
  //#define TEMP_CHARCODE 15      //Thermometer "C", set to -1 to disable
  //#define GRAD_CHARCODE 16      //Thermometer grad, set to -1 to disable
  //#define PERCENT_CHARCODE 17   //Humidity %
  //#define AP_NAME "UNICLOCK"    //Access Point name
  //#define AP_PASSWORD ""        //AP password
  //#define WEBNAME "ClockForgeOS"  // Device name displayed on web interface
  //#define DEFAULT_SSID ""       //factoryReset default value for WiFi
  //#define DEFAULT_PSW ""        //factoryReset default value  for WiFi password
  //#define FACTORYRESET_PIN -1   //if defined, make rest, if button is pushed     
*/
//#define USE_MDNS
#define TIMESERVER_REFRESH 7200000     //2h, Refresh time in millisec   86400000 = 24h
unsigned long timeserverErrors = 0;        //timeserver refresh errors

int timerON=0;   //for debugging
int timerOFF=0;      
unsigned long intCounter = 0;   //for testing only, interrupt counter

byte c_MinBrightness = RGB_MIN_BRIGHTNESS;     //minimum LED brightness
byte c_MaxBrightness = RGB_MAX_BRIGHTNESS;     //maximum LED brightness
//--------------------------------------------------------------------------------------------------
#define FIRMWARE_SERVER ""
#ifndef FACTORY_DEFAULT_AP_NAME
  #define FACTORY_DEFAULT_AP_NAME "NixieClock"
#endif
#ifndef FACTORY_DEFAULT_AP_PASSWORD
  #define FACTORY_DEFAULT_AP_PASSWORD "18273645"
#endif
String usedPinsStr;
String driverSetupStr;

#if defined(ESP8266)
  #define DRAM_ATTR     //https://docs.espressif.com/projects/esp-idf/en/v4.3-beta2/esp32/api-reference/storage/spi_flash_concurrency.html
  #ifndef WEBNAME
    #define WEBNAME "ClockForgeOS 1.0"
  #endif
  #ifndef AP_NAME
    #define AP_NAME "CLOCKFORGEOS"
  #endif
  #ifndef AP_PASSWORD
    #define AP_PASSWORD ""
  #endif
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClient.h>
  #include "ESPAsyncTCP.h"
  #include "FS.h"
  #include <ESP8266httpUpdate.h>
  #ifdef USE_MDNS
    #include <ESP8266mDNS.h>
  #endif  

  
  const char ESPpinout[] = {"OOOOOO      OOOOOI"};   //GPIO 0..5, 12..16, A0)  usable pins

#elif defined(ESP32)
  #ifndef WEBNAME
    #define WEBNAME "ClockForgeOS 1.0"
  #endif
  #ifndef AP_NAME
    #define AP_NAME "CLOCKFORGEOS"
  #endif
  #ifndef AP_PASSWORD
    #define AP_PASSWORD "18273645"
  #endif
  #include <WiFi.h>  
  #include <esp_wifi.h>
  #include <WiFiClient.h>
  #include <HTTPClient.h>
  #include <ESP32httpUpdate.h>

  #include "AsyncTCP.h"
  #include "SPIFFS.h"
  #include "FS.h"  
  #include "soc/soc.h"
  #include "soc/rtc_cntl_reg.h"
  #ifdef USE_MDNS
    #include <ESPmDNS.h>
  #endif
  hw_timer_t * ESP32timer = NULL;
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  #define PRESCALER 15   //multiplex timer prescaler, about 80Hz for 6 tubes
  
  const char ESPpinout[] = {"OOOOOO      OOOOOOOO OOO OOO    OOIII  I"};   //GPIO 0..5, 12..19, 21..23, 25..27, 32..33, Input:34..36, 39)  usable pins 
#else
  #error "Board is not supported!"
#endif

#include <DNSServer.h>
DNSServer dnsServer;
#include "ESPAsyncWebServer.h"
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <TimeLib.h>
#include <Timezone.h>
#include <Wire.h>
#include <EEPROM.h>
#include "ArduinoJson.h"

#if defined(USE_NEOPIXEL) || defined(WORDCLOCK)
  #include <NeoPixelBrightnessBus.h>
#endif

extern void ICACHE_RAM_ATTR writeDisplay();
extern void writeDisplaySingle();
extern void setup_pins();
extern void clearTubes();
extern int maxDigits;
extern char tubeDriver[];

//Web handlers (forward declarations)
void handleConfigChanged(AsyncWebServerRequest *request);
void handleSendConfig(AsyncWebServerRequest *request);
void handleSendPublicConfig(AsyncWebServerRequest *request);
void handleSendCurrentInfos(AsyncWebServerRequest *request);
void handleSendClockDetails(AsyncWebServerRequest *request);
void handleSendSystemInfo(AsyncWebServerRequest *request);
void handleScanWifi(AsyncWebServerRequest *request);
void handleConnectWifi(AsyncWebServerRequest *request);
void handleConnectWifiPost(AsyncWebServerRequest *request);
void handleWifiStatus(AsyncWebServerRequest *request);
void handleNotFound(AsyncWebServerRequest *request);
void handleSetManualTime(AsyncWebServerRequest *request);
void handleAuthLogin(AsyncWebServerRequest *request);

#ifndef WEB_ADMIN_PASSWORD
  #define WEB_ADMIN_PASSWORD "ChangeMeNow!"  // Fallback only; set WEB_ADMIN_PASSWORD in clocks.h profile
#endif

static String webAuthToken = "";
static unsigned long webAuthExpireMs = 0;
static const unsigned long WEB_AUTH_TTL_MS = 30UL * 60UL * 1000UL;

static String generateWebAuthToken() {
  static const char hex[] = "0123456789abcdef";
  char out[33];
  for (int i = 0; i < 32; i++) {
    #if defined(ESP32)
      uint8_t v = (uint8_t)(esp_random() & 0x0F);
    #else
      uint8_t v = (uint8_t)(random(0, 16));
    #endif
    out[i] = hex[v];
  }
  out[32] = '\0';
  return String(out);
}

static bool isWebAuthValidToken(const String& token) {
  if (token.length() == 0 || webAuthToken.length() == 0) return false;
  if (token != webAuthToken) return false;
  if ((long)(millis() - webAuthExpireMs) >= 0) {
    webAuthToken = "";
    webAuthExpireMs = 0;
    return false;
  }
  return true;
}

static bool requireWebAuth(AsyncWebServerRequest *request) {
  String token = "";
  if (request->hasHeader("X-Auth-Token")) {
    AsyncWebHeader* h = request->getHeader("X-Auth-Token");
    if (h) token = h->value();
  }
  if (token.length() == 0 && request->hasParam("authToken", true)) {
    token = request->getParam("authToken", true)->value();
  }
  if (token.length() == 0 && request->hasParam("authToken")) {
    token = request->getParam("authToken")->value();
  }

  if (!isWebAuthValidToken(token)) {
    request->send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return false;
  }

  webAuthExpireMs = millis() + WEB_AUTH_TTL_MS;
  return true;
}

bool isPrintableString(const String& s) {
  if (s.length() == 0) return false;
  for (size_t i = 0; i < s.length(); i++) {
    if (!isPrintable(s[i])) return false;
  }
  return true;
}

static bool isValidTemperatureReading(float value) {
  return (value > -40.0f) && (value < 85.0f) && !((value > -0.15f) && (value < 0.15f));
}

static bool isValidHumidityReading(float value) {
  return (value >= 1.0f) && (value <= 100.0f);
}

static int countValidTemperatureSensors();
static int countValidHumiditySensors();

char webName[] = WEBNAME;
AsyncWebServer server(80);
AsyncEventSource events("/events");
WiFiServer telnetServer(23);
WiFiClient telnetClients[2];

bool uiDebugEnabled = false;
String wifiScanResults = "";
volatile bool wifiScanInProgress = false;

static const uint32_t HEAP_WARN_BYTES = 28UL * 1024UL;
static const uint32_t HEAP_RECOVER_BYTES = 36UL * 1024UL;
static const uint32_t HEAP_CRITICAL_BYTES = 18UL * 1024UL;
static bool heapProtectionActive = false;
static bool heapAutoDisabledUiDebug = false;
static uint32_t heapMinFreeBytes = 0xFFFFFFFFUL;
static unsigned long lastHeapGuardMs = 0;
static unsigned long lastHeapCleanupMs = 0;

static bool wifiSwitchPending = false;
static bool wifiSwitchSaveOnSuccess = false;
static bool wifiSwitchCanRollback = false;
static bool wifiSwitchRollbackRunning = false;
static bool wifiRecoveryApMode = false;
static unsigned long wifiSwitchStartMs = 0;
static const unsigned long WIFI_SWITCH_TIMEOUT_MS = 20000UL;
static String wifiSwitchTargetSsid = "";
static String wifiSwitchTargetPsw = "";
static String wifiSwitchPrevSsid = "";
static String wifiSwitchPrevPsw = "";
static String wifiSwitchLastResult = "idle";

static String currentInfosCache = "";
static unsigned long currentInfosCacheMs = 0;
static String systemInfoCache = "";
static unsigned long systemInfoCacheMs = 0;

const uint32_t DISPLAY_TOGGLE_DEBOUNCE_MS = 250;
uint32_t lastManualDisplayToggleMs = 0;

const uint32_t EEPROM_SAVE_MIN_INTERVAL_MS = 3000;
uint32_t lastEepromSaveMs = 0;
bool eepromSavePending = false;

// Ring buffer (last N lines) for web reconnect
static const uint16_t LOG_LINES = 120;
static String logRing[LOG_LINES];
static uint16_t logHead = 0;
static bool logFilled = false;

static void pushRing(const String& line) {
  logRing[logHead] = line;
  logHead = (logHead + 1) % LOG_LINES;
  if (logHead == 0) logFilled = true;
}

static void telnetHousekeeping() {
  if (telnetServer.hasClient()) {
    for (auto &c : telnetClients) {
      if (!c || !c.connected()) {
        c.stop();
        c = telnetServer.available();
        if (c) {
          c.setNoDelay(true);
          c.println("ClockForgeOS Telnet Log connected");
        }
        return;
      }
    }
    WiFiClient reject = telnetServer.available();
    reject.stop();
  }
  for (auto &c : telnetClients) {
    if (c && !c.connected()) c.stop();
  }
}

static uint8_t telnetActiveConnections() {
  uint8_t n = 0;
  for (auto &c : telnetClients) {
    if (c && c.connected()) n++;
  }
  return n;
}

static void telnetBroadcast(const String& line) {
  for (auto &c : telnetClients) {
    if (c && c.connected()) c.println(line);
  }
}

static void debugLogLine(const String& line) {
  pushRing(line);

#ifdef DEBUG
  Serial.println(line);
#endif

  if (!uiDebugEnabled) return;
// SSE (rate-limited to avoid async_tcp queue overflow / WDT)
  static uint32_t sseWindowMs = 0;
  static uint16_t sseCountInWindow = 0;
  static bool sseDropNoticeSent = false;

  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - sseWindowMs) >= 1000) {
    sseWindowMs = nowMs;
    sseCountInWindow = 0;
    sseDropNoticeSent = false;
  }

  // Only send if at least one client is connected
  if (events.count() > 0) {
    // Keep this VERY low. On ESP32 (especially with AsyncTCP), bursts of queued
    // SSE messages can starve the async_tcp task and trigger the task watchdog.
    // 5 msg/s is enough for debugging without destabilizing the web stack.
    if (sseCountInWindow < 5) {
      events.send(line.c_str(), "log", nowMs);
      sseCountInWindow++;
    } else if (!sseDropNoticeSent) {
      events.send("[WARN] SSE log rate limited (dropping messages)", "log", nowMs);
      sseDropNoticeSent = true;
    }
  }
  // Telnet
  telnetBroadcast(line);
}

static void debugLogf(const char* fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  debugLogLine(String(buf));
}

// A Print sink that routes all existing DPRINT/DPRINTLN/DPRINTF to remote logs
class RemoteDebugPrint : public Print {
public:
  void begin(unsigned long baud) { Serial.begin(baud); }
  size_t write(uint8_t c) override {
    // single char write: buffer until newline to avoid spam
    if (c == '\r') return 1;
    if (c == '\n') { flushLine(); return 1; }
    lineBuf += (char)c;
    if (lineBuf.length() > 240) flushLine();
    return 1;
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    for (size_t i=0;i<size;i++) write(buffer[i]);
    return size;
  }
  void printf(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    // support embedded newlines by splitting
    String s(buf);
    int start = 0;
    while (true) {
      int nl = s.indexOf('\n', start);
      if (nl < 0) { debugLogLine(s.substring(start)); break; }
      debugLogLine(s.substring(start, nl));
      start = nl + 1;
      if (start >= (int)s.length()) break;
    }
  }
  void flushLine() {
    if (lineBuf.length()) {
      debugLogLine(lineBuf);
      lineBuf = "";
    }
  }
private:
  String lineBuf;
};

RemoteDebugPrint DebugOut;
// ======================================================================


#define CACHE_MAX_AGE "max-age=31536000" //maximum is: 31536000

#ifdef USE_WIFIMANAGER
  #include <ESPAsyncWiFiManager.h>
  AsyncWiFiManager myWiFiManager(&server,&dnsServer);
#endif

//______________ WiFi finding variables ____________________________________________
byte bestChn;         //the recent found best wifi AP's channel No.
uint8_t bestBssid[6]; //the recent found best wifi AP's BSSID (mac address)
uint8_t oldBssid[6];  //the current wifi AP's BSSID (mac address)
int bestRSSI = -200;  //the recent found best wifi AP's RSSI value
unsigned long lastWifiScan = 0;  //last wifi scan timestamp in CPU msec millis()
boolean wifiOK = false;
char myIp[20]="";

//Display buffers
#define BUFSIZE 12
byte DRAM_ATTR digit[BUFSIZE];
byte DRAM_ATTR newDigit[BUFSIZE];
byte DRAM_ATTR oldDigit[BUFSIZE];
boolean DRAM_ATTR digitDP[BUFSIZE];   //actual value to put to display
boolean digitsOnly = true;  //only 0..9 numbers are possible to display?
byte DRAM_ATTR animMask[BUFSIZE];     //0 = no animation mask is used


// --- Soft blanking wrapper (Clock64 safe) ---
// Forward declaration (tubesPowerState is defined later in the file).
extern bool tubesPowerState;
#ifdef USE_NEOPIXEL
void darkenNeopixels();
extern volatile uint16_t neoAppliedCurrentmA;
extern volatile uint8_t neoAppliedBrightness;
#endif

// When tubesPowerState is false, we temporarily blank digits before pushing them to the driver.
static inline void writeDisplaySingleGuarded() {
  auto rawWriteFn = &writeDisplaySingle; // keep original driver function

#ifdef USE_NEOPIXEL
  // Track whether we've already forced LEDs off while the tubes are off,
  // to avoid calling darkenNeopixels() on every multiplex refresh.
  static bool ledsForcedOff = false;
#endif

  if (!tubesPowerState) {
#ifdef USE_NEOPIXEL
    if (!ledsForcedOff) {
      darkenNeopixels();
      ledsForcedOff = true;
    }
#endif
    byte saved[BUFSIZE];
    boolean savedDP[BUFSIZE];
    memcpy(saved, digit, sizeof(saved));
    memcpy(savedDP, digitDP, sizeof(savedDP));
    memset(digit, 10, sizeof(digit));   // 10 = BLANK in this firmware
    memset(digitDP, 0, sizeof(digitDP));
    rawWriteFn();
    memcpy(digit, saved, sizeof(saved));
    memcpy(digitDP, savedDP, sizeof(savedDP));
  } else {
#ifdef USE_NEOPIXEL
    ledsForcedOff = false; // tubes are on again; allow normal LED programs to run
#endif
    rawWriteFn();
  }
}

// Wrapper used by legacy code paths to push current digit buffer to HV driver.
// This calls the guarded variant so tubes-off state blanks the driver correctly.
static inline void rawWrite() {
  writeDisplaySingleGuarded();
}

volatile boolean dState = false;
volatile unsigned long lastDisable = 0;
volatile boolean EEPROMsaving = false; //saving in progress - stop display refresh

#define MAGIC_VALUE 305   //EEPROM version


//#define THERMOMETER_CLOCK     //it means, first digit is +- sign, last digit is C/F, 4 digit is for numbers
#ifdef THERMOMETER_CLOCK    //it means 6 tubes:  first digit is +- sign, last digit is C/F, and only 4 digit are for numbers
  boolean thermometerClock = true;
#else
  boolean thermometerClock = false;
#endif

// 8266 internal pin registers
// https://github.com/esp8266/esp8266-wiki/wiki/gpio-registers
// example: https://github.com/mgo-tec/OLED_1351/blob/master/src/OLED_SSD1351.cpp


#ifdef DEBUG    //Macros are usually in all capital letters.
#define DPRINT(...)       DebugOut.print(__VA_ARGS__)
#define DPRINTLN(...)     DebugOut.println(__VA_ARGS__)
#define DPRINTF(...)      DebugOut.printf(__VA_ARGS__)
#define DPRINTBEGIN(...)  DebugOut.begin(__VA_ARGS__)
#else
#define DPRINT(...)     //now defines a blank line
#define DPRINTLN(...)   //now defines a blank line
#define DPRINTF(...)     //now defines a blank line
#define DPRINTBEGIN(...)   //now defines a blank line
#endif
// ===================== EXTENDED DEBUG (Clock_64) =====================
// This block provides a stable, extensible serial debug snapshot (1 line / second)
// plus event logs (motion edge, display ON/OFF edge). Keep it in the project long-term.
#define DEBUG_ENABLED 1
#define DEBUG_INTERVAL_MS 1000UL

#if DEBUG_ENABLED
struct DebugState {
  bool motion;
  bool tubesSleep;
  bool wakeOnMotionEnabled;
  bool manualDisplayOff;
  bool enableTimeDisplay;
  bool mqttRadarON;
  bool radarAllowed;
  bool tubesPowerState;
  uint16_t tubesWakeSeconds;
  uint16_t wakeLeftSeconds;
  const char* timeSource;
  unsigned long nowMs;
  unsigned long lastMotionMs;
  unsigned long wakeMs;
};
static DebugState dbg;

static const char* dbgTimeSourceStr(const char* ts) { return ts ? ts : "Unknown"; }

// Call this once per loop after checkTubePowerOnOff() updated dbg fields.
static void debugSnapshot() {
  if (!uiDebugEnabled) return;
  static unsigned long last = 0;
  const unsigned long now = millis();
  if (now - last < DEBUG_INTERVAL_MS) return;
  last = now;

  DPRINTF("[DBG] motion=%d sleep=%d wakeOnMotion=%d manualOff=%d enable=%d mqttRadar=%d radarAllowed=%d tubes=%d "
          "wake=%us left=%us wakeMs=%lu now=%lu lastMotion=%lu time=%s\n",
          (int)dbg.motion, (int)dbg.tubesSleep, (int)dbg.wakeOnMotionEnabled, (int)dbg.manualDisplayOff,
          (int)dbg.enableTimeDisplay, (int)dbg.mqttRadarON, (int)dbg.radarAllowed, (int)dbg.tubesPowerState,
          (unsigned)dbg.tubesWakeSeconds, (unsigned)dbg.wakeLeftSeconds, dbg.wakeMs,
          dbg.nowMs, dbg.lastMotionMs, dbgTimeSourceStr(dbg.timeSource));
}
#else
static void debugSnapshot() {}
#endif
// =================== END EXTENDED DEBUG ===================


#define PIN_OUT_SET PERIPHS_GPIO_BASEADDR + 4
#define PIN_OUT_CLEAR PERIPHS_GPIO_BASEADDR + 8

bool colonBlinkState = false;
boolean radarON = true;
boolean mqttRadarON = true;
unsigned long radarLastOn = 0;
boolean makeFirmwareUpdate = false;
boolean makeCathodeProtect = false;
volatile boolean stopCathodeProtect = false;
int cathProtMin = 5;
unsigned long lastTimeUpdate = 0;  //last time refresh from GPS or internet timeserver
boolean RTCexist = false;
boolean RTCisPCF8563 = false;
boolean GPSexist = false;
boolean BME280exist = false;
boolean BMP280exist = false;
boolean AHTX0exist = false;
boolean SHT21exist = false;
boolean BH1750exist = false;
boolean LDRexist = false;

byte useDallasTemp = 0;   //number of Dallas temperature sensors: 0,1,2
byte useTemp = 0;         //Total number of any temperature sensors: 0..6
byte usePress = 0;        //Total number of pressure sensors
byte useHumid = 0;        //Total number of humidity sensors
byte useLux = 0;
float temperature[6] = {0,0,0,0,0,0};
float humid[6] = {0,0,0,0,0,0};  
float pressur[6] = {0,0,0,0,0,0};  
int lx = 0;               //Enviroment LUX value, set by Light Sensor
boolean autoBrightness = false; //Enable automatic brightness levels

static int countValidTemperatureSensors() {
  int count = 0;
  for (int i = 0; i < useTemp; i++) {
    if (isValidTemperatureReading(temperature[i])) count++;
  }
  return count;
}

static int countValidHumiditySensors() {
  int count = 0;
  for (int i = 0; i < useHumid; i++) {
    if (isValidHumidityReading(humid[i])) count++;
  }
  return count;
}

//----------------- EEPROM addresses -------------------------------------------------------------------
const int EEPROM_addr = 0;

// Generic struct for external MQTT device support
struct ExternalMqttDevice {
  char name[32];
  char topic[96];
  uint8_t sensorSlot;
  float lastValue;
};
ExternalMqttDevice externalDevices[4]; // Example: support up to 4 external devices

// Integration with sensor arrays
void updateExternalDeviceSensor(uint8_t slot, float value) {
  if (slot < 6) {
    temperature[slot] = value;
    // Optionally update humidity, pressure, etc. based on device type
  }
}

struct {
//Main screen settings _____________________________________________________________________________________ 
  boolean alarmEnable = false;     //Yes or No
  byte alarmHour = 7;              //Alarm time
  byte alarmMin = 0;
  byte alarmPeriod = 15;           //Alarm length, sec
//RGB settings ____________________________________________________________________________________________
  byte rgbEffect = 1;              //0=OFF, 1=FixColor
  byte rgbBrightness = 100;        // 0..255
  unsigned int rgbFixColor = 150;  //0..255
  byte rgbSpeed = 50;              //0..255msec / step
  boolean rgbDir = false;          //false = right, true = left
//Wifi / ip settings _______________________________________________________________________________________ 
  boolean wifiMode = true; 
  char wifiSsid[20];
  char wifiPsw[20];
  char ApSsid[20];
  char ApPsw[20];
  char NtpServer[30];
  char mqttBrokerAddr[30]; 
  char mqttBrokerUser[20] = "mqtt";
  char mqttBrokerPsw[20] = "mqtt";
  int mqttBrokerRefresh = 10;  //sec
  boolean mqttEnable = false;
  char firmwareServer[78];
//Tube settings  ______________________________________________________________________________________
  int tempRepeatMin = 1;             //temperature and humidity display repaeat (min)
  int utc_offset = 1;              //time zone offset
  bool enableDST = true;           // Flag to enable DST (summer time...)
  bool set12_24 = true;            // Flag indicating 12 vs 24 hour time (false = 12, true = 24);
  bool showZero = true;            // Flag to indicate whether to show zero in the hour ten's place
  bool enableBlink = true;         // Flag to indicate whether center colon should blink
  int  interval = 15;              // prm.interval in minutes, with 0 = off
  bool enableAutoShutoff = true;   // Flag to enable/disable nighttime shut off
  byte dayHour = 8;                // start of daytime
  byte dayMin = 0;
  byte nightHour = 22;             // start of night time
  byte nightMin = 0;
  byte dayBright = MAXBRIGHTNESS;  // display daytime brightness
  byte nightBright = 5;            // display night brightness
  byte animMode = 7;               //0=no anim,  if 1 or 2 is used, animation, when a digit changes
  byte dateMode = 2;               // 0:dd/mm/yyyy 1:mm/dd/yyyy 2:yyyy/mm/dd
  boolean tempCF = false;          //Temperature Celsius=false / Fahrenheit=true
  boolean enableTimeDisplay;       //ENABLE_CLOCK_DISPLAY
  byte dateStart;                  //Date is displayed start..end
  byte dateEnd;    
  byte tempStart;                  //Temperature display start..end
  byte tempEnd;  
  byte humidStart;                 //Humidity% display start..end
  byte humidEnd;
  byte dateRepeatMin;              //show date only every xxx minute. If zero, datum is never displayed!  
  boolean enableDoubleBlink;       //both separator points are blinking (6 or 8 tubes VFD clock)
  boolean enableAutoDim = false;   //Automatic dimming by luxmeter
  boolean enableRadar = false;     //Radar sensor
  int radarTimeout = 5;          //min
  uint16_t tubesWakeSeconds = 10;   // Wake duration in seconds when motion is detected (Clock64)
   uint16_t maxLedmA = 350;          // Max LED current budget (mA) for NeoPixel limiter (0=disabled)
  bool manualDisplayOff = false;    // Manual Display OFF override (UI)
  bool wakeOnMotionEnabled = true; // Wake on motion enabled (false = display always ON)
  uint8_t touchShortAction = 0;      // ESP32 touch GPIO33 short press action
  uint8_t touchDoubleAction = 0;     // ESP32 touch GPIO33 double press action
  uint8_t touchLongAction = 0;       // ESP32 touch GPIO33 long press action
  float corrT0 = 0;
  float corrT1 = 0;
  float corrH0 = 0;
  float corrH1 = 0;
//____________________________________________________________________________________________________  
    //Manual time fallback (used when no NTP/GPS/RTC available)
  bool manualTimeValid = false;
  uint32_t manualEpoch = 0;   //local epoch seconds
int magic = MAGIC_VALUE;        //magic value, to check EEPROM version when starting the clock
  // --- New (v8): Manual tubes sleep mode (wake on motion) ---
  bool tubesSleep = false;       //if true, tubes are normally OFF and wake on motion
} prm;

#define EEPROM_SIZE (sizeof(prm) + sizeof(Settings))

static const int SETTINGS_EEPROM_ADDR = EEPROM_addr + (int)sizeof(prm);
Settings settings;
volatile bool settingsDirty = false;
//-------------------------------------------------------------------------------------------------------

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, prm.NtpServer, 0, TIMESERVER_REFRESH); // Refresh time in millisec
IPAddress ip;

// Set timezone rules.  Offsets set to zero, since they will be loaded from EEPROM
TimeChangeRule myDST = {"DST", Last, Sun, Mar, 2, 0};
TimeChangeRule mySTD = {"STD", First, Sun, Nov, 2, 0};
Timezone myTZ(myDST, mySTD);
time_t prevTime = 0;
time_t protectTimer = 0;
bool displayON = true;
bool manualOverride = false;

// --- New (v8): Tubes sleep / wake-on-motion ---
bool tubesPowerState = true;         // current HV power state (ON/OFF)
unsigned long tubesLastMotionMs = 0; //if tubesSleep is enabled, tubes stay ON until this time (millis)
bool startupMuteRgb = false;         // suppress RGB animations during selected boot phases

bool onboardLedState = false;

#ifndef ONBOARD_LED_PIN
  #if defined(LED_BUILTIN)
    #define ONBOARD_LED_PIN LED_BUILTIN
  #else
    #define ONBOARD_LED_PIN 2
  #endif
#endif

#ifndef ONBOARD_LED_ACTIVE_LOW
  #define ONBOARD_LED_ACTIVE_LOW 0
#endif

static inline void applyOnboardLedState() {
  #if ONBOARD_LED_PIN >= 0
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    const uint8_t levelOn = ONBOARD_LED_ACTIVE_LOW ? LOW : HIGH;
    const uint8_t levelOff = ONBOARD_LED_ACTIVE_LOW ? HIGH : LOW;
    digitalWrite(ONBOARD_LED_PIN, onboardLedState ? levelOn : levelOff);
  #endif
}

static inline void forceOnboardLedOffEarly() {
  #if ONBOARD_LED_PIN >= 0
    pinMode(ONBOARD_LED_PIN, OUTPUT);
    const uint8_t levelOff = ONBOARD_LED_ACTIVE_LOW ? HIGH : LOW;
    digitalWrite(ONBOARD_LED_PIN, levelOff);
  #endif
}

bool initProtectionTimer = false;  // Set true at the top of the hour to synchronize protection timer with clock
  // Set true at the top of the hour to synchronize protection timer with clock
bool decimalpointON = false;
bool alarmON = false;             //Alarm in progress
unsigned long alarmStarted = 0;   //Start timestamp millis()

enum TouchAction : uint8_t {
  TOUCH_ACTION_NONE = 0,
  TOUCH_ACTION_ALARM_OFF = 1,
  TOUCH_ACTION_COLOR_CHANGE = 2,
  TOUCH_ACTION_DISPLAY_OFF = 3,
  TOUCH_ACTION_DISPLAY_TOGGLE = 4,
  TOUCH_ACTION_COLOR_PREV = 5
};

#if defined(ESP32)
  #ifndef TOUCH_BUTTON_PIN
    #define TOUCH_BUTTON_PIN 33
  #endif

  static const uint32_t TOUCH_LONG_PRESS_MS = 700;
  static const uint32_t TOUCH_DOUBLE_PRESS_MS = 350;
  static const uint32_t TOUCH_LOG_INTERVAL_MS = 120;

  static bool touchPressed = false;
  static bool touchShortPending = false;
  static uint32_t touchPressedAtMs = 0;
  static uint32_t touchLastReleaseMs = 0;
  static uint32_t touchLastLogMs = 0;
  static uint16_t touchBaseline = 0;
  static uint16_t touchThreshold = 0;
#endif

boolean showClock = false;
boolean showDate = false;
boolean showTemp0 = false;
boolean showTemp1 = false;
boolean showHumid0 = false;
boolean showHumid1 = false;
boolean showPress0 = false;
int lastCathodeProt = -1;
boolean cathodeProtRunning = false;
boolean ipShowRunning = false;
boolean wifiConnectRunning = false;
boolean editorRunning = false;

static void clearLogRing() {
  for (uint16_t i = 0; i < LOG_LINES; i++) {
    logRing[i] = "";
  }
  logHead = 0;
  logFilled = false;
}

static void closeTelnetClients() {
  for (auto &c : telnetClients) {
    if (c) c.stop();
  }
}

static void heapGuardTick() {
  const unsigned long nowMs = millis();
  if ((nowMs - lastHeapGuardMs) < 5000UL) return;
  lastHeapGuardMs = nowMs;

  const uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  if (freeHeap < heapMinFreeBytes) heapMinFreeBytes = freeHeap;

  if (freeHeap <= HEAP_WARN_BYTES) {
    if (!heapProtectionActive) {
      heapProtectionActive = true;
      DPRINTLN("[HEAP] Low memory protection enabled.");
    }

    if (uiDebugEnabled) {
      uiDebugEnabled = false;
      heapAutoDisabledUiDebug = true;
      DPRINTLN("[HEAP] UI debug stream disabled to reduce memory load.");
    }

    wifiScanResults = "";

    if ((freeHeap <= HEAP_CRITICAL_BYTES) || ((nowMs - lastHeapCleanupMs) > 30000UL)) {
      clearLogRing();
      closeTelnetClients();
      lastHeapCleanupMs = nowMs;
      DPRINTLN("[HEAP] Cleanup applied (log ring + telnet clients)." );
    }
    return;
  }

  if (heapProtectionActive && freeHeap >= HEAP_RECOVER_BYTES) {
    heapProtectionActive = false;
    DPRINTLN("[HEAP] Memory recovered. Protection relaxed.");
    if (heapAutoDisabledUiDebug && settings.debugEnabled) {
      uiDebugEnabled = true;
      DPRINTLN("[HEAP] UI debug stream restored.");
    }
    heapAutoDisabledUiDebug = false;
  }
}

#define MAX_PIN sizeof(ESPpinout)-1
#define PIN_TXT_LEN 30
char pinTxt[MAX_PIN][PIN_TXT_LEN+1];

void regPin(byte p,const char * txt) {  //register used pins
  
  DPRINT("- "); DPRINT(txt); DPRINT(": GPIO"); DPRINT(p); 
  if (p == 255) { DPRINT("  (unused pin)"); DPRINTLN(" "); return; }
  if (p >= MAX_PIN) {
    DPRINT("  ERROR: PIN# DOESN'T EXIST.");
    return;
  }
  else if (ESPpinout[p]==' ') {
    DPRINT("  ERROR: RESERVED PIN.");
  }
  else if (ESPpinout[p]=='I') {
    DPRINT("  Warning: input only pin");
  }
  DPRINTLN(" ");
  if (strlen(pinTxt[p])>0) {
    DPRINT("*** ERROR *** "); DPRINT(txt); DPRINT(" on PIN#"); DPRINT(p);  
    DPRINT(" ALREADY DEFINED AS "); DPRINTLN(pinTxt[p]); 
    strncpy(pinTxt[p],"Error:MULTI DEF",PIN_TXT_LEN);
    pinTxt[p][PIN_TXT_LEN] = '\0';
  }
  else {
    strncpy(pinTxt[p],txt,PIN_TXT_LEN);
    pinTxt[p][PIN_TXT_LEN] = '\0';
  }
}

void listPins() {
  DPRINTLN("_______________________");
  DPRINTLN("___ USED CLOCK PINS ___");
  #if defined(ESP32)
    usedPinsStr = String("ESP32 used pins:<br>");
  #else
    usedPinsStr = String("ESP8266 used pins:<br>");
  #endif  
  for (byte i=0;i<MAX_PIN;i++) {
    if (strlen(pinTxt[i])>0) {
      usedPinsStr += String(i) + ": " + String(pinTxt[i]) + "<br>";
      DPRINT(i); DPRINT(": ");  DPRINTLN(pinTxt[i]);
    }
  }
  DPRINTLN("_______________________");
  //DPRINT("MAX_PIN"); DPRINTLN(MAX_PIN);
}

void startTimer() {   //ESP_INTR_FLAG_IRAM
#if defined(ESP8266)
  timer1_attachInterrupt(writeDisplay);
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_SINGLE);
  timer1_write(15000);
  DPRINTLN("Starting 8266 timer...");

#elif defined(ESP32)
  //  https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
  ESP32timer = timerBegin(0, PRESCALER, true); //set prescaler to 80 -> 1MHz signal, true = edge generated signal
  timerAttachInterrupt(ESP32timer, &writeDisplay, true);
  timerAlarmWrite(ESP32timer, 1000, true);   //100millisec, no repeat
  timerAlarmEnable(ESP32timer);
  //DPRINTLN("Starting ESP32 timer...");
#else
#error "Unknown controller board!"
#endif
}

void stopTimer() {
#if defined(ESP8266)
  timer1_detachInterrupt();
#endif
}

#ifdef USE_WIFIMANAGER
void configModeCallback (AsyncWiFiManager *myWiFiManager) {
  DPRINTLN("Switch to AP config mode...");
  DPRINTLN("To configure Wifi,  ");
  DPRINT("connect to Wifi network "); DPRINTLN(prm.wifiSsid);
  DPRINTLN("and open 192.168.4.1 in web browser");
}
#endif

void clearDigits() {
  memset(oldDigit, 10, sizeof(oldDigit));
  memset(digit, 10, sizeof(digit));
  memset(newDigit, 10, sizeof(newDigit));
  memset(digitDP, 0, sizeof(digitDP));
}

void Fdelay(unsigned long d) {
  unsigned long dStart = millis();
  bool neopixelsForcedOff = false;

  #ifdef USE_NEOPIXEL
  if (tubesPowerState) {
    doAnimationMakuna();
  } else {
    darkenNeopixels();
    neopixelsForcedOff = true;
  }
  #else
  doAnimationMakuna();
  #endif
  while ((millis() - dStart) < d) {
    if (WiFi.getMode() == 2) dnsServer.processNextRequest();
    enableDisplay(2000);
    writeDisplay2();
    getLightSensor();
    #ifdef USE_NEOPIXEL
    if (tubesPowerState) {
      neopixelsForcedOff = false;
      doAnimationMakuna();
    } else {
      if (!neopixelsForcedOff) {
        darkenNeopixels();
        neopixelsForcedOff = true;
      }
    }
    #else
    doAnimationMakuna();
    #endif
    if (tubesPowerState) {
      doAnimationPWM();
    }
    alarmSound();
    #ifdef USE_GPS
      smartDelay(1);  //feed GPS
    #endif  
    processPendingEepromSave();
    yield();
  }
}

float round1(float in) {
  return round(10.0 * in)/10.0; 
}


static bool parseBool(const String &v, bool defaultValue=false) {
  // Accept common forms from web UIs: true/false, 1/0, on/off, yes/no
  String s = v;
  s.trim();
  s.toLowerCase();
  if (s == "true" || s == "1" || s == "on" || s == "yes")  return true;
  if (s == "false" || s == "0" || s == "off" || s == "no") return false;
  return defaultValue;
}

//--------------------------------------------------------------------------------------------------------------------

void enableDisplay(unsigned long timeout) {
  // Display multiplex is continuously running on ESP32 in this fork.
  // Keep the function for compatibility with upstream code paths.
  (void)timeout;
  dState = true;
  EEPROMsaving = false;
}

void disableDisplay()  {
  // Deprecated in this project version (kept for compatibility).
  dState = true;
  EEPROMsaving = false;
}

boolean findBestWifi() {
  int n;
  int newRssi;
  boolean found = false;
  
  if (WiFi.status() == WL_CONNECTED) {  //save wifi settings to prm
    WiFi.disconnect();
  }
  WiFi.scanNetworks(false);
  n = WiFi.scanComplete();
  DPRINT(F("Scanning WiFi, found networks:")); DPRINTLN(n); 
  if (n>=0){
    bestRSSI = -200;
    memset(bestBssid,0,sizeof(bestBssid));
    for (int i = 0; i < n; ++i){
      if (WiFi.SSID(i) == String(prm.wifiSsid)) {
        newRssi = WiFi.RSSI(i);
        DPRINT(F("scan: RSSI")); DPRINT(newRssi);
        if (bestRSSI<newRssi) {  //better rsssi found...
          bestChn = WiFi.channel(i);
          bestRSSI = newRssi;
          memcpy(bestBssid,WiFi.BSSID(i),sizeof(bestBssid));
          DPRINT(F("  FOUND: Channel:")); DPRINT(bestChn);
          DPRINT(F("  BSSID:")); DPRINTLN(WiFi.BSSIDstr(i));
          found = true;
          lastWifiScan = millis();
        }
        else {
          DPRINTLN(" ");
        }
      }
    }
    WiFi.scanDelete();
    }  //end else
    else {
     // WiFi.scanNetworks(false);
    }
    return(found);
}

void wifiManager() {
  #ifdef USE_WIFIMANAGER
  AsyncWiFiManager MyWifiManager(&server, &dnsServer);
  MyWifiManager.setAPCallback(configModeCallback);
  const char* portalSsid = (strlen(prm.ApSsid) > 0) ? prm.ApSsid : FACTORY_DEFAULT_AP_NAME;
  const char* portalPsw = (strlen(prm.ApPsw) >= 8) ? prm.ApPsw : FACTORY_DEFAULT_AP_PASSWORD;
  //MyWifiManager.setConfigPortalTimeout(180);
  for (int i = 0; i < 5; i++) {
    if (!MyWifiManager.autoConnect(portalSsid, portalPsw)) {
      DPRINT("Retry to Connect:"); DPRINTLN(i);
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      delay(1000);
      WiFi.mode(WIFI_STA);
      #if defined(ESP32)
        WiFi.setHostname(webName);
      #else
        WiFi.hostname(webName);
      #endif
      if (i == 4) {
        MyWifiManager.autoConnect(portalSsid, portalPsw);
      }
    }
    else
      break;
  }  //end for

  if (WiFi.status() == WL_CONNECTED) {  //save wifi settings to prm
    String s = WiFi.SSID();
    String p = WiFi.psk();
    memcpy(oldBssid,WiFi.BSSID(),sizeof(oldBssid));
    s.toCharArray(prm.wifiSsid,sizeof(prm.wifiSsid));
    p.toCharArray(prm.wifiPsw,sizeof(prm.wifiPsw));
    requestSaveEEPROM();
    ip = WiFi.localIP();
    WiFi.setAutoReconnect(true);
    enableDisplay(100);
  }
  #endif
}

void startNewWifiMode() { // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
 if (((millis()-lastWifiScan)<90000) && (bestRSSI > -95) && (WiFi.status() != WL_CONNECTED)) {
  DPRINTLN(F("_____________ Connecting WiFi ___________________"));
  if (strlen(prm.wifiSsid)==0) return;
  WiFi.disconnect();
  delay(200);
  WiFi.mode(WIFI_OFF);
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  #if defined(ESP32)
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setHostname(webName);
    WiFi.setSleep(false);
  #else  
    WiFi.hostname(webName);
  #endif
  
  char tmp[20];
  sprintf(tmp,"%02X:%02X:%02X:%02X:%02X:%02X",bestBssid[0],bestBssid[1],bestBssid[2],bestBssid[3],bestBssid[4],bestBssid[5]);
  DPRINT(F("Connecting to best WiFi AP:")); DPRINT(prm.wifiSsid);  DPRINT(F("  PSW:")); DPRINT(prm.wifiPsw);
  DPRINT(F(" BSSID:")); DPRINT(tmp); DPRINT(" Chn:"); DPRINTLN(bestChn); 
  WiFi.begin(prm.wifiSsid, prm.wifiPsw, bestChn, bestBssid, true);

  Fdelay(500);
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    DPRINT('.');
    Fdelay(3000);
    if (counter++>3) {
      DPRINTLN("Connecting to WiFi failed.");
      return;
    }  
  }    
  ip = WiFi.localIP();
  sprintf(myIp,"%d.%d.%d.%d",ip[0],ip[1],ip[2],ip[3]);
  DPRINT("\nConnected to ");  DPRINT(WiFi.SSID());   
  DPRINT(". LocalIP:"); DPRINT(myIp);     
  DPRINT(" BSSID:"); DPRINT(WiFi.BSSIDstr());    
  DPRINT(" RSSI:"); DPRINT(WiFi.RSSI());   
  DPRINT(" CHN:"); DPRINTLN(WiFi.channel());   
  DPRINTLN("\n\nNetwork Configuration:");
  DPRINTLN("----------------------");
  DPRINT("         SSID: "); DPRINTLN(WiFi.SSID());
  DPRINT("  Wifi Status: "); DPRINTLN(WiFi.status());
  DPRINT("Wifi Strength: "); DPRINT(WiFi.RSSI()); DPRINTLN(" dBm");
  DPRINT("          MAC: "); DPRINTLN(WiFi.macAddress());
  DPRINT("           IP: "); DPRINTLN(myIp);
  DPRINT("       Subnet: "); DPRINTLN(WiFi.subnetMask());
  DPRINT("      Gateway: "); DPRINTLN(WiFi.gatewayIP());
  DPRINT("        DNS 1: "); DPRINTLN(WiFi.dnsIP(0));
  DPRINT("        DNS 2: "); DPRINTLN(WiFi.dnsIP(1));
  DPRINT("        DNS 3: "); DPRINTLN(WiFi.dnsIP(2));
  DPRINTLN("_____________________________________________________________________________"); 
  memcpy(oldBssid,WiFi.BSSID(),sizeof(oldBssid));
}
  DPRINTLN("\r\n");
}


void startWifiMode() {
  wifiRecoveryApMode = false;
  disableDisplay();
  DPRINTLN("Starting Clock in WiFi Mode!");
  if (strlen(prm.wifiSsid)==0) {
    DPRINTLN("WiFi SSID not defined!");
    wifiManager();
    return;
  }
  WiFi.disconnect(true);
  //WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  delay(100);
  //Fdelay(1000);
  #if defined(ESP32)
    WiFi.setHostname(webName);
    WiFi.setSleep(false);
  #else
    WiFi.hostname(webName);
  #endif
  #if defined(ESP32)
    esp_wifi_set_ps (WIFI_PS_NONE);  //power saving disable!
  #endif
  //if (WiFi.status() == WL_CONNECTED) return;

  DPRINT("\nConnecting to WiFi SSID:("); DPRINT(prm.wifiSsid); DPRINT(")  PSW:("); DPRINT(prm.wifiPsw); DPRINTLN(")");
  int counter = 0;
  if (strlen(prm.wifiPsw)!=0)
    WiFi.begin(prm.wifiSsid, prm.wifiPsw);
  else  
    WiFi.begin(prm.wifiSsid);
  while (WiFi.status() != WL_CONNECTED) {
    DPRINT('.');
    //playTubes();
    //Fdelay(3000);
    delay(1000);
    if (counter++>10) {
      wifiManager();
      return;
    }
  }
  DPRINTLN(" ");
  ip = WiFi.localIP();
  WiFi.setAutoReconnect(true);
  enableDisplay(100);
}


boolean updateTimefromTimeserver() {  //true, if successful
  static unsigned long lastTimeFailure = 0;
  boolean res = false;
  int count = 1;
  
  if (((millis()-lastTimeUpdate)<TIMESERVER_REFRESH) && (lastTimeUpdate!=0))
    return(res);
    
  if ((lastTimeFailure>0) && ((millis()-lastTimeFailure)<300000)) return(res);   //wait for 5min to retry, if no success
  
  if (WiFi.status() == WL_CONNECTED) {
    while (true) {
      DPRINT("Connecting to timeserver: "); DPRINTLN(count);
      enableDisplay(1000);
      res = timeClient.forceUpdate();
      if (res) {
        mySTD.offset = prm.utc_offset * 60;
        myDST.offset = mySTD.offset;
        if (prm.enableDST) {
          myDST.offset += 60;
        }
        myTZ = Timezone(myDST, mySTD);  
        setTime(myTZ.toLocal(timeClient.getEpochTime()));
        lastTimeUpdate = millis();
        //NTP is authoritative: clear any manual fallback time.
        if (prm.manualTimeValid) {
          prm.manualTimeValid = false;
          prm.manualEpoch = 0;
          requestSaveEEPROM();
        }
        DPRINT("Timeserver date:"); DPRINT(year()); DPRINT("/"); DPRINT(month()); DPRINT("/"); DPRINT(day());      
        DPRINT(" time:");   DPRINT(hour()); DPRINT(":"); DPRINT(minute()); DPRINT(":"); DPRINTLN(second());  
        DPRINTLN("Clock refreshed from timeserver.");
        lastTimeFailure = 0;
      }
      count ++; 
      if (res) break;  //success!!!
      Fdelay(1000);
      if (count > 5) {  //failure, but stop trying
        lastTimeFailure = millis();
        break;
      }
    } //end while
  }
  return (res);
}

#ifndef MDNSNAME
    #define MDNSNAME "clockforgeos"
#endif

void startMDNS() {
  static boolean mdnsStarted = false;

  if (mdnsStarted) return;   //running and not changed
  #ifdef USE_MDNS
    if (!MDNS.begin(MDNSNAME)) {   //mdns_free() switch off
      DPRINTLN(F("Error setting up MDNS responder!"));
      mdnsStarted = false;
    }
    else {
      DPRINT(F("Started mDns service:")); DPRINTLN(MDNSNAME);
      MDNS.addService("http", "tcp", 80);  
      mdnsStarted = true;
    }
  #endif  
}

void startStandaloneMode() {
  wifiRecoveryApMode = true;
  DPRINTLN("Starting Clock in Standalone Mode!");
  DPRINT("Clock's AP SSID:"); DPRINT(prm.ApSsid);
  DPRINT("   PSW:"); DPRINTLN(prm.ApPsw);
  DPRINTLN("IP: 192.168.4.1");
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  boolean nwState;
  if (strlen(prm.ApPsw)) {
    nwState =  WiFi.softAP(prm.ApSsid, prm.ApPsw);
  }
  else {
    nwState =  WiFi.softAP(prm.ApSsid);
  }
  Fdelay(2000);   //info: https://github.com/espressif/arduino-esp32/issues/2025
  
  DPRINT("AP status:"); DPRINTLN(nwState ? "Ready" : "Failed!");  //channel
  DPRINT("Mac address:"); DPRINTLN(WiFi.softAPmacAddress());
  ip = WiFi.softAPIP();
  DPRINT("SOFT_AP IP:"); DPRINTLN(WiFi.softAPIP());

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());

  enableDisplay(0);
}

void doFirmwareUpdate(){
    if (WiFi.status() != WL_CONNECTED) {
      DPRINTLN("Wifi disconnected. FirmwareUpdate failed.");
      return;
    }

    DPRINTLN("Webserver stopped...");
    server.reset();  //Stop server
    delay(2000);
    disableDisplay();
    yield();
    String fname = String(prm.firmwareServer)+"/"+String(FW)+".bin";
    DPRINT("Update firmware: "); DPRINTLN(fname);
    t_httpUpdate_return ret,ret2;
    boolean succ = false;
    WiFiClient client;
    ESPhttpUpdate.rebootOnUpdate(false);
    #if defined(ESP32) || defined(ESP8266_CORE_2xx)
      ret = ESPhttpUpdate.update(fname);    //ESP32 and old 8266 Core
    #else
      ret = ESPhttpUpdate.update(client,fname);   //new 8266 core
    #endif  
    switch(ret) {
            case HTTP_UPDATE_FAILED:
                DPRINTF("HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                DPRINTLN(" ");
                break;

            case HTTP_UPDATE_NO_UPDATES:
                DPRINTLN("HTTP_UPDATE_NO_UPDATES");
                break;

            case HTTP_UPDATE_OK:
                DPRINTLN("HTTP_UPDATE_OK");
                succ = true;
                break;
    }

    fname = String(prm.firmwareServer)+"/"+String(FW)+".spiffs.bin";
    DPRINT("Update SPIFFS: "); DPRINTLN(fname);
    #if defined(ESP32)|| defined(ESP8266_CORE_2xx)
      ret2 = ESPhttpUpdate.updateSpiffs(fname);    //ESP32 or old 8266 Core
    #else  
      ret2 = ESPhttpUpdate.updateFS(client,fname);   //new 8266 core
    #endif  
            switch(ret2) {
                case HTTP_UPDATE_FAILED:
                    DPRINTF("HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                    DPRINTLN(" ");
                    break;

                case HTTP_UPDATE_NO_UPDATES:
                    DPRINTLN("HTTP_UPDATE_NO_UPDATES");
                    break;

                case HTTP_UPDATE_OK:
                    DPRINTLN("HTTP_UPDATE_OK");
                    succ = true;
                    break;
            }
 
    if (succ) {
      DPRINTLN(" ");
      delay(1000);
      doReset();
    }
    DPRINTLN(" ");
    startServer();  //restart webserver
}

void doCathodeProtect() {
  unsigned long started = millis();
  byte num =0;
  int db = prm.dayBright;  //save brightness values
  int nb = prm.nightBright;
  boolean ab = autoBrightness;
  byte onOff = 0;
  
  prm.dayBright=MAXBRIGHTNESS;  
  prm.nightBright = MAXBRIGHTNESS;
  autoBrightness = false;
  cathodeProtRunning = true;
  stopCathodeProtect = false;
  
  DPRINT("Cathode Protect is running for "); DPRINT(cathProtMin); DPRINTLN(" minutes.");
  memset(digitDP, 0, sizeof(digitDP));
  memset(animMask,0,sizeof(animMask));
  while (true) {
   for (int i=0;i<maxDigits;i++)  {  
    if (i%2) digit[i] = num;
    else     digit[i] = 9-num;
    newDigit[i] = digit[i];
    digitDP[i] = (onOff%2 == i%2);
   } 
   writeDisplaySingleGuarded();
   Fdelay(100);
   num++; if (num>9) num = 0;
   onOff++;
   if (stopCathodeProtect) {
     DPRINTLN("Cathode Protect manually stopped.");
     break;
   }
   if ((millis()-started)>uint32_t(cathProtMin)*60000l) break;
  } //end while
  
  prm.dayBright = db;  //restore brightness values
  prm.nightBright = nb;
  autoBrightness = ab; 
  cathodeProtRunning = false;
  stopCathodeProtect = false;
}

void startServer() {

// SSE live logs endpoint
server.addHandler(&events);
events.onConnect([](AsyncEventSourceClient *client){
  if (!uiDebugEnabled) {
    client->send("debug_disabled", "status", millis());
    client->close();
    return;
  }
  client->send("connected", "status", millis());
  // Do NOT dump the ring buffer on connect.
  // On some boards this creates a burst of queued SSE messages that can
  // starve AsyncTCP and trip the task watchdog.
});

  DPRINTLN("Starting Async Webserver...");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if(SPIFFS.exists("/index.html")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/index.html", "text/html"); 
      response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
      request->send(response);
    }
    else {
    request->send( 204, "text/html", "File Not Found" );
    DPRINTLN("/index.html not found");
    }
  });

  server.on("/jquery_351.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/jquery_351.js")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/jquery_351.js", "application/javascript");   
      response->addHeader("Cache-Control", CACHE_MAX_AGE);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DPRINTLN("/jquery_351.js not found");
    }
  });

  server.on("/jquery_360.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/jquery_360.js")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/jquery_360.js", "application/javascript");   
      response->addHeader("Cache-Control", CACHE_MAX_AGE);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DPRINTLN("/jquery_360.js not found");
    }
  });

  // Compatibility alias: some cached pages request /jquery.js
  server.on("/jquery.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/jquery_360.js")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/jquery_360.js", "application/javascript");
      request->send(response);
    } else if (SPIFFS.exists("/jquery_351.js")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/jquery_351.js", "application/javascript");
      request->send(response);
    } else {
      request->send(404, "text/plain", "File Not Found");
    }
  });

  // Captive-portal probes (iOS/Android) - avoid spamming the logs and wasting AsyncTCP queue.
  server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(204);
  });

  server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(204);
  });


  server.on("/page.js", HTTP_GET, [](AsyncWebServerRequest * request) {

    disableDisplay();
    if (SPIFFS.exists("/page.js")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/page.js", "application/javascript");   
      response->addHeader("Cache-Control", CACHE_MAX_AGE);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DPRINTLN("/page.js not found");
    }
  });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/favicon.ico")){
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/favicon.ico", "image/png");
      response->addHeader("Cache-Control", CACHE_MAX_AGE);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DPRINTLN("/favicon.ico not found");
    }    
  });

  server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    AsyncWebServerResponse *response = request->beginResponse(302); //
    response->addHeader("Location", String("http://") + WiFi.softAPIP().toString().c_str());
    request->send(response);
    /*
       //request->send(SPIFFS, "/index.html", "text/html");
       AsyncResponseStream *response = request->beginResponseStream("text/html");
       response->print("<!DOCTYPE html><html><head><title>UniClock</title></head><body>");
       response->print("<p><H1>");  response->print(webName);
       response->print("</p><p>Please, login to network and </p>");
       response->printf("<p>try opening <a href='http://%s'>%s</a> </p>", WiFi.softAPIP().toString().c_str(),WiFi.softAPIP().toString().c_str());
       //response->printf("192.168.4.1");
       response->print("<H1></body></html>");
       request->send(response);
    */
  });

//____________________________________________________________________________
  server.on("/site.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/site.css")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/site.css", "text/css");   
      response->addHeader("Cache-Control", CACHE_MAX_AGE);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DPRINTLN("/site.css not found");
    }     
  });
//____________________________________________________________________________
  server.on("/reset", HTTP_POST, [] (AsyncWebServerRequest *request) {
    if (!requireWebAuth(request)) return;
    DPRINTLN("/reset:");
    
    request->send(200, "text/plain", "Reset: Restarting the Box!");
    delay(200);
    doReset();
  });
//____________________________________________________________________________
  server.on("/factoryreset", HTTP_POST, [] (AsyncWebServerRequest *request) {
    if (!requireWebAuth(request)) return;
    DPRINTLN("/factoryreset:");

    request->send(200, "text/plain", "Factory Reset!");
    delay(200);
    factoryReset();
    doReset();
  });

//____________________________________________________________________________
  server.on("/firmwareupdate", HTTP_POST, [](AsyncWebServerRequest * request) {
    if (!requireWebAuth(request)) return;
    makeFirmwareUpdate = true;
    request->send(200, "application/json", "{\"header\":\"Firmware update\",\"content\":\"Update task queued. Device will download and reboot if successful.\",\"canClose\":true}");
  });
//____________________________________________________________________________
  server.on("/cathodeProtect", HTTP_POST, [](AsyncWebServerRequest * request) {
    if (!requireWebAuth(request)) return;
    if (cathodeProtRunning) {
      stopCathodeProtect = true;
      request->send(200, "application/json", "{\"header\":\"Cathode Protect\",\"content\":\"Stop requested. Procedure will end shortly.\",\"canClose\":true,\"cathodeProtRunning\":true}");
    }
    else {
      makeCathodeProtect = true;
      request->send(200, "application/json", "{\"header\":\"Cathode Protect\",\"content\":\"Procedure started. Press the button again to stop.\",\"canClose\":true,\"cathodeProtRunning\":true}");
    }
  });

  server.on("/auth/login", HTTP_POST, handleAuthLogin);

  server.on("/saveSetting", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!requireWebAuth(request)) return;
    handleConfigChanged(request);
  });
  server.on("/getPublicConfig", HTTP_GET, handleSendPublicConfig);
  server.on("/setManualTime", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!requireWebAuth(request)) return;
    handleSetManualTime(request);
  });
  server.on("/getConfiguration", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!requireWebAuth(request)) return;
    handleSendConfig(request);
  });
  server.on("/getSystemInfo", HTTP_GET, handleSendSystemInfo);
  server.on("/scanWifi", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!requireWebAuth(request)) return;
    handleScanWifi(request);
  });
  // Quick connect for open networks (GET, no save)
  server.on("/connectWifi", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!requireWebAuth(request)) return;
    handleConnectWifi(request);
  });
  // Secure connect + optional save (POST)
  server.on("/connectWifi", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!requireWebAuth(request)) return;
    handleConnectWifiPost(request);
  });
  // WiFi status probe
  server.on("/wifiStatus", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!requireWebAuth(request)) return;
    handleWifiStatus(request);
  });
  server.on("/getCurrentInfos", HTTP_GET, handleSendCurrentInfos);
  server.on("/getClockDetails", HTTP_GET, handleSendClockDetails);
  server.onNotFound(handleNotFound);

  server.begin();

}  //end of procedure

void handleNotFound(AsyncWebServerRequest *request) {
  auto getMimeType = [](const String& path) -> const char* {
    if (path.endsWith(".html") || path.endsWith(".htm")) return "text/html";
    if (path.endsWith(".css")) return "text/css";
    if (path.endsWith(".js") || path.endsWith(".mjs")) return "application/javascript";
    if (path.endsWith(".json")) return "application/json";
    if (path.endsWith(".svg")) return "image/svg+xml";
    if (path.endsWith(".png")) return "image/png";
    if (path.endsWith(".jpg") || path.endsWith(".jpeg")) return "image/jpeg";
    if (path.endsWith(".gif")) return "image/gif";
    if (path.endsWith(".ico")) return "image/x-icon";
    if (path.endsWith(".txt") || path.endsWith(".log")) return "text/plain";
    return "application/octet-stream";
  };

  String urlPath = request->url();
  if (urlPath == "/") {
    urlPath = "/index.html";
  }

  if (SPIFFS.exists(urlPath)) {
    disableDisplay();
    AsyncWebServerResponse *response = request->beginResponse(SPIFFS, urlPath, getMimeType(urlPath));
    response->addHeader("Cache-Control", CACHE_MAX_AGE);
    request->send(response);
    return;
  }

  int params = request->params();
  for (int i = 0; i < params; i++) {
    #ifdef DEBUG
    AsyncWebParameter* p = request->getParam(i);
    if (p->isFile()) { //p->isPost() is also true
      Serial.printf("FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
    } else if (p->isPost()) {
      Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
    } else {
      Serial.printf("GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
    }
    #endif
  }

  String message = "File Not Found\n\n";
  message += "URL: ";
  message += request->url();
  message += "\nMethod: ";
  message += ( request->method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += request->args();
  message += "\n";

  for ( uint8_t i = 0; i < request->args(); i++ ) {
    message += " " + request->argName ( i ) + ": " + request->arg ( i ) + "\n";
  }

  DPRINTLN(message);
  request->send( 204, "text/html", "URL Not Found" );
}

void handleAuthLogin(AsyncWebServerRequest *request) {
  if (!request->hasParam("password", true)) {
    request->send(400, "application/json", "{\"error\":\"missing_password\"}");
    return;
  }

  String password = request->getParam("password", true)->value();
  if (password != String(WEB_ADMIN_PASSWORD)) {
    request->send(401, "application/json", "{\"error\":\"invalid_password\"}");
    return;
  }

  webAuthToken = generateWebAuthToken();
  webAuthExpireMs = millis() + WEB_AUTH_TTL_MS;

  DynamicJsonDocument doc(256);
  doc["ok"] = true;
  doc["token"] = webAuthToken;
  doc["ttlSec"] = (WEB_AUTH_TTL_MS / 1000UL);
  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

void handleConfigChanged(AsyncWebServerRequest *request) {
  bool doSave = true; // added: allow skipping EEPROM save for runtime-only settings

  if (request->hasParam("key", true) && request->hasParam("value", true)) {

    //int args = request->args();

    //for(int i=0;i<args;i++){
    //  Serial.printf("ARG[%s]: %s\n", request->argName(i).c_str(), request->arg(i).c_str());
    //}
    boolean oldDST = prm.enableDST;
    int old_utc_offset = prm.utc_offset;
    
    String key = request->getParam("key", true)->value();
    String value = request->getParam("value", true)->value();
    DPRINT(key); DPRINT(" = "); DPRINTLN(value);

    boolean paramFound = true;

    if (key == "utc_offset")    {
      prm.utc_offset = value.toInt();
      if (old_utc_offset != prm.utc_offset) {   //Change time zone
        setTime(now()+(prm.utc_offset-old_utc_offset)*3600);
        updateRTC();
      }
    }
    else if (key == "set12_24") {
      prm.set12_24 = (value == "true");
      DPRINT("set12_24:");
      DPRINTLN(prm.set12_24);
    }
    else if (key == "showZero") {
      prm.showZero = (value == "true");
      DPRINT("showZero:");
      DPRINTLN(prm.showZero);
    }
    else if (key == "enableBlink") {
      prm.enableBlink = (value == "true");
    }
    else if (key == "enableDST")  {
      prm.enableDST = (value == "true");
      if (oldDST && !prm.enableDST) {   //Switching off DST
        setTime(now()-3600);
        updateRTC();
      }
      if (!oldDST && prm.enableDST) {   //Switching on DST
        setTime(now()+3600);
        updateRTC();
      }
    }
    else if (key == "interval")   {
      prm.interval = value.toInt();
    }
    else if (key == "enableAutoShutoff") {
      prm.enableAutoShutoff = parseBool(value);
      // If auto day/night is disabled, force DAY mode unless user explicitly overrides
      if (!prm.enableAutoShutoff) {
        manualOverride = false;
        displayON = true;
      }
    }

    else if (key == "tubesSleep") {
      prm.tubesSleep = parseBool(value);
      if (!prm.tubesSleep) {
        tubesLastMotionMs = 0;
      }
      // When entering sleep mode, turn tubes off immediately.
      if (prm.tubesSleep) {
        tubesLastMotionMs = 0;
      }
    }

    else if (key == "tubesWakeSeconds") {
      // Wake duration in seconds for motion-based tubes sleep (Clock64)
      prm.tubesWakeSeconds = (uint16_t)value.toInt();
      if (prm.tubesWakeSeconds < 1) prm.tubesWakeSeconds = 1;
      if (prm.tubesWakeSeconds > 3600) prm.tubesWakeSeconds = 3600;
    }
    else if (key == "wakeOnMotionEnabled") {
      prm.wakeOnMotionEnabled = parseBool(value, true);
    }

    else if (key == "touchShortAction") {
      int action = value.toInt();
      if (action < TOUCH_ACTION_NONE) action = TOUCH_ACTION_NONE;
      if (action > TOUCH_ACTION_DISPLAY_TOGGLE) action = TOUCH_ACTION_DISPLAY_TOGGLE;
      if (action == TOUCH_ACTION_DISPLAY_OFF) action = TOUCH_ACTION_DISPLAY_TOGGLE;
      prm.touchShortAction = (uint8_t)action;
    }

    else if (key == "touchDoubleAction") {
      int action = value.toInt();
      if (action < TOUCH_ACTION_NONE) action = TOUCH_ACTION_NONE;
      if (action > TOUCH_ACTION_DISPLAY_TOGGLE) action = TOUCH_ACTION_DISPLAY_TOGGLE;
      if (action == TOUCH_ACTION_DISPLAY_OFF) action = TOUCH_ACTION_DISPLAY_TOGGLE;
      prm.touchDoubleAction = (uint8_t)action;
    }

    else if (key == "touchLongAction") {
      int action = value.toInt();
      if (action < TOUCH_ACTION_NONE) action = TOUCH_ACTION_NONE;
      if (action > TOUCH_ACTION_DISPLAY_TOGGLE) action = TOUCH_ACTION_DISPLAY_TOGGLE;
      if (action == TOUCH_ACTION_DISPLAY_OFF) action = TOUCH_ACTION_DISPLAY_TOGGLE;
      prm.touchLongAction = (uint8_t)action;
    }

    else if (key == "gestureUpAction") {
      int action = value.toInt();
      if (action < TOUCH_ACTION_NONE) action = TOUCH_ACTION_NONE;
      if (action > TOUCH_ACTION_COLOR_PREV) action = TOUCH_ACTION_COLOR_PREV;
      settings.gestureUpAction = (uint8_t)action;
      settingsMarkDirty();
    }

    else if (key == "gestureDownAction") {
      int action = value.toInt();
      if (action < TOUCH_ACTION_NONE) action = TOUCH_ACTION_NONE;
      if (action > TOUCH_ACTION_COLOR_PREV) action = TOUCH_ACTION_COLOR_PREV;
      settings.gestureDownAction = (uint8_t)action;
      settingsMarkDirty();
    }

    else if (key == "gestureLeftAction") {
      int action = value.toInt();
      if (action < TOUCH_ACTION_NONE) action = TOUCH_ACTION_NONE;
      if (action > TOUCH_ACTION_COLOR_PREV) action = TOUCH_ACTION_COLOR_PREV;
      settings.gestureLeftAction = (uint8_t)action;
      settingsMarkDirty();
    }

    else if (key == "gestureRightAction") {
      int action = value.toInt();
      if (action < TOUCH_ACTION_NONE) action = TOUCH_ACTION_NONE;
      if (action > TOUCH_ACTION_COLOR_PREV) action = TOUCH_ACTION_COLOR_PREV;
      settings.gestureRightAction = (uint8_t)action;
      settingsMarkDirty();
    }

else if (key == "debugEnabled") {
  const bool en = parseBool(value, false);
  uiDebugEnabled = en;
  settings.debugEnabled = en;
  settingsMarkDirty();
  requestSaveEEPROM();
  debugLogf("Debug %s", uiDebugEnabled ? "ENABLED" : "DISABLED");
}

    else if (key == "onboardLed") {
      onboardLedState = parseBool(value, false);
      applyOnboardLedState();
      doSave = false; // runtime-only
    }

    else if (key == "displayPower") {
      const uint32_t nowMs = millis();
      if ((nowMs - lastManualDisplayToggleMs) >= DISPLAY_TOGGLE_DEBOUNCE_MS) {
        lastManualDisplayToggleMs = nowMs;
        // Manual display power from UI (true=ON, false=OFF)
        bool on = parseBool(value, true);
        prm.manualDisplayOff = !on;
      } else {
        // ignore rapid toggles (do not mark settings dirty)
        doSave = false;
      }
    }
    else if (key == "manualDisplayOff") {
      const uint32_t nowMs = millis();
      if ((nowMs - lastManualDisplayToggleMs) >= DISPLAY_TOGGLE_DEBOUNCE_MS) {
        lastManualDisplayToggleMs = nowMs;
        // Alternative key name (newer UI)
        prm.manualDisplayOff = parseBool(value, false);
      } else {
        doSave = false;
      }
    }
else if (key == "dayTimeHours")   {
      prm.dayHour = value.toInt();
    }
    else if (key == "dayTimeMinutes") {
      prm.dayMin = value.toInt();
    }
    else if (key == "nightTimeHours") {
      prm.nightHour = value.toInt();
    }
    else if (key == "nightTimeMinutes") {
      prm.nightMin = value.toInt();
    }
    else if (key == "dayBright") {
      prm.dayBright = value.toInt();
      // If user sets brightness manually, disable lux-based auto dimming
      prm.enableAutoDim = false;
      autoBrightness = false;
    }
    else if (key == "nightBright") {
      prm.nightBright = value.toInt();
      // If user sets brightness manually, disable lux-based auto dimming
      prm.enableAutoDim = false;
      autoBrightness = false;
    }
    else if (key == "animMode")     {
      prm.animMode = value.toInt();
    }
    else if (key == "manualOverride") {
      boolean v = value == "false";
      if (v != displayON) {
        manualOverride = true;
        displayON = v;
      }
    }
    else if (key == "wifiMode")      {
      prm.wifiMode = (value == "true");
    }    
    else if (key == "alarmEnable")      {
      prm.alarmEnable = (value == "true");
      settingsDirty = true;
      requestSaveEEPROM();
    }
      else if (key == "showTimeDate") {
        prm.enableTimeDisplay = (value == "true");
        settingsDirty = true;
        requestSaveEEPROM();
      }
      else if (key == "showTemperature") {
        settings.enableTempDisplay = (value == "true");
        settingsMarkDirty();
        requestSaveEEPROM();
      }
      else if (key == "showHumidity") {
        settings.enableHumidDisplay = (value == "true");
        settingsMarkDirty();
        requestSaveEEPROM();
      }
      else if (key == "showPressure") {
        settings.enablePressDisplay = (value == "true");
        settingsMarkDirty();
        requestSaveEEPROM();
      }
    else if (key == "alarmTimeHours")   {
      prm.alarmHour = value.toInt();
      settingsDirty = true;
      requestSaveEEPROM();
    }
    else if (key == "alarmTimeMinutes") {
      prm.alarmMin = value.toInt();
      settingsDirty = true;
      requestSaveEEPROM();
    }
    else if (key == "alarmPeriod")      {
      prm.alarmPeriod = value.toInt();
      settingsDirty = true;
      requestSaveEEPROM();
    }

    //RGB LED values
    else if (key == "rgbEffect")     {
      prm.rgbEffect = value.toInt();
    }
    else if (key == "rgbBrightness") {
      prm.rgbBrightness = value.toInt();
    }
      else if (key == "rgbAnimationSpeed") {
        prm.rgbSpeed = value.toInt();
      }
    else if (key == "maxLedmA") {
      int v = value.toInt();
      if (v < 0) v = 0;
      if (v > 2000) v = 2000;
      prm.maxLedmA = (uint16_t)v;
    }
    else if (key == "rgbFixR") {
      settings.rgbFixR = (uint8_t)constrain(value.toInt(), 0, 255);
      // Auto-sync: when user sets fixed RGB color, ensure effect is set to 1 (fixed color mode)
      if (prm.rgbEffect != 1) {
        prm.rgbEffect = 1;
        debugLogf("[RGB] Auto-sync rgbEffect -> 1 (fixed color)");
      }
      // Batch RGB updates: only save to EEPROM if all values are set
      static uint8_t rgbBatchCount = 0;
      rgbBatchCount++;
      if (rgbBatchCount >= 3) {
        saveRGBSettingsToEEPROM();
        rgbBatchCount = 0;
        debugLogf("[RGB] R/G/B set, saved to EEPROM");
      }
    }
    else if (key == "rgbFixG") {
      settings.rgbFixG = (uint8_t)constrain(value.toInt(), 0, 255);
      if (prm.rgbEffect != 1) {
        prm.rgbEffect = 1;
        debugLogf("[RGB] Auto-sync rgbEffect -> 1 (fixed color)");
      }
      static uint8_t rgbBatchCount = 0;
      rgbBatchCount++;
      if (rgbBatchCount >= 3) {
        saveRGBSettingsToEEPROM();
        rgbBatchCount = 0;
        debugLogf("[RGB] R/G/B set, saved to EEPROM");
      }
    }
    else if (key == "rgbFixB") {
      settings.rgbFixB = (uint8_t)constrain(value.toInt(), 0, 255);
      if (prm.rgbEffect != 1) {
        prm.rgbEffect = 1;
        debugLogf("[RGB] Auto-sync rgbEffect -> 1 (fixed color)");
      }
      static uint8_t rgbBatchCount = 0;
      rgbBatchCount++;
      if (rgbBatchCount >= 3) {
        saveRGBSettingsToEEPROM();
        rgbBatchCount = 0;
        debugLogf("[RGB] R/G/B set, saved to EEPROM");
      }
    }
else if (key == "rgbSpeed")      {
      prm.rgbSpeed = value.toInt();
    }
    else if (key == "rgbDir")        {
      prm.rgbDir = (value == "true");
    }
    else if (key == "rgbMinBrightness") {
      c_MinBrightness = value.toInt();
    }
    else if (key == "wifiSsid") {
      value.toCharArray(prm.wifiSsid,sizeof(prm.wifiSsid));
    }
    else if (key == "wifiPsw") {
      value.toCharArray(prm.wifiPsw,sizeof(prm.wifiPsw));
    }
    else if (key == "ApSsid") {
      for (int i=0;i<(int)strlen(prm.ApSsid);i++) {  //repair bad chars in AP SSID
        if ((prm.ApSsid[i]<32) || (prm.ApSsid[i]>126)) prm.ApSsid[i]='_';
      }
      value.toCharArray(prm.ApSsid,sizeof(prm.ApSsid));
    }
    else if (key == "ApPsw") {
      value.toCharArray(prm.ApPsw,sizeof(prm.ApPsw));
    }
    else if (key == "NtpServer") {
      value.toCharArray(prm.NtpServer,sizeof(prm.NtpServer));
    }
    else if (key == "firmware") {
      value.toCharArray(prm.firmwareServer,sizeof(prm.firmwareServer));
    }
      else if (key == "mqttBrokerAddr") {
        value.toCharArray(prm.mqttBrokerAddr,sizeof(prm.mqttBrokerAddr));
      }
      else if (key == "mqttBrokerUser") {
        value.toCharArray(prm.mqttBrokerUser,sizeof(prm.mqttBrokerUser));
      }
      else if (key == "mqttBrokerPsw") {
        value.toCharArray(prm.mqttBrokerPsw,sizeof(prm.mqttBrokerPsw));
      }         
      else if (key == "mqttBrokerRefresh") {
        prm.mqttBrokerRefresh = value.toInt();
      }     
      else if (key == "mqttEnable") {
        prm.mqttEnable = (value == "true");
      }
      else if (key.startsWith("extDev") && key.length() >= 8) {
        int idx = key.substring(6, 7).toInt();
        String field = key.substring(7);
        if (idx < 0 || idx >= 4) {
          paramFound = false;
        } else if (field == "Name") {
          value.toCharArray(externalDevices[idx].name, sizeof(externalDevices[idx].name));
        } else if (field == "Topic") {
          value.toCharArray(externalDevices[idx].topic, sizeof(externalDevices[idx].topic));
        } else if (field == "Slot") {
          int slot = value.toInt();
          if (slot < 0) slot = 0;
          if (slot > 5) slot = 5;
          externalDevices[idx].sensorSlot = (uint8_t)slot;
        } else {
          paramFound = false;
        }
      }
    else if (key == "dateMode") {
      prm.dateMode = value.toInt();
    }
    else if (key == "dateStart") {
      prm.dateStart = value.toInt();
    }
    else if (key == "dateEnd") {
      prm.dateEnd = value.toInt();
    }
    else if (key == "tempStart") {
      prm.tempStart = value.toInt();
    }
    else if (key == "tempEnd") {
      prm.tempEnd = value.toInt();
    }                
    else if (key == "humidStart") {
      prm.humidStart = value.toInt();
    }
    else if (key == "humidEnd") {
      prm.humidEnd = value.toInt();
    }
    else if (key == "pressureStart") {
      int v = value.toInt();
      if (v < 0) v = 0;
      if (v > 59) v = 59;
      settings.pressureStart = (uint8_t)v;
      settingsMarkDirty();
    }
    else if (key == "pressureEnd") {
      int v = value.toInt();
      if (v < 0) v = 0;
      if (v > 59) v = 59;
      settings.pressureEnd = (uint8_t)v;
      settingsMarkDirty();
    }
    else if (key == "dateRepeatMin") {
      prm.dateRepeatMin = value.toInt();
      if (prm.dateRepeatMin>10) prm.dateRepeatMin = 10;
    } 
    else if (key == "tempRepeatMin") {
      prm.tempRepeatMin = value.toInt();
      if (prm.tempRepeatMin>10) prm.tempRepeatMin = 10;
    }     
    else if (key == "enableDoubleBlink")  {
      prm.enableDoubleBlink = (value == "true");
    }
    else if (key == "enableTimeDisplay") {
      prm.enableTimeDisplay = (value == "true");
    }
    else if (key == "enableTempDisplay") {
      settings.enableTempDisplay = (value == "true");
      settingsMarkDirty();
    }
    else if (key == "enableHumidDisplay") {
      settings.enableHumidDisplay = (value == "true");
      settingsMarkDirty();
    }
    else if (key == "enablePressDisplay") {
      settings.enablePressDisplay = (value == "true");
      settingsMarkDirty();
    }
    else if (key == "enableAutoDim" || key == "autoDim" || key == "autoDimming" || key == "enableAutoDimming" || key == "enableAutoDimmingByLux") {
      prm.enableAutoDim = parseBool(value, prm.enableAutoDim);
      autoBrightness = prm.enableAutoDim;
      // When switching auto-dimming OFF from the web UI, immediately release brightness control
      if (!prm.enableAutoDim) {
        lx = MAXIMUM_LUX;
      }
    }
    else if (key == "enableRadar")  {
      prm.enableRadar = (value == "true");
      radarLastOn = millis();
      radarON = true;
    }
    else if (key == "radarTimeout"){
      prm.radarTimeout = value.toInt();
      if (prm.radarTimeout>60) prm.radarTimeout = 60;
      if (prm.radarTimeout<1) prm.radarTimeout = 1;
      radarLastOn = millis();
      radarON = true;
    }  
    else if (key == "tempCF") {
      prm.tempCF = (value == "true");
    }   
    else if (key == "corrT0") {
      prm.corrT0 = value.toFloat();
    } 
    else if (key == "corrT1") {
      prm.corrT1 = value.toFloat();
    } 
    else if (key == "corrH0") {
      prm.corrH0 = value.toFloat();
    } 
    else if (key == "corrH1") {
      prm.corrH1 = value.toFloat();
    }        
    else if (key == "cathProtMin") {
      cathProtMin = value.toInt();
    }
    // === UI Customization Settings (persisted to EEPROM) ===
    else if (key == "uiWidth") {
      // uiWidth: "100%" -> 0xFFFF, otherwise pixels as number
      if (value == "100%") {
        settings.uiWidth = 0xFFFF;
      } else {
        settings.uiWidth = (uint16_t)constrain(value.toInt(), 400, 1600);
      }
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Width set to %s", (settings.uiWidth == 0xFFFF) ? "100%" : String(settings.uiWidth).c_str());
    }
    else if (key == "uiBgColor") {
      // Parse hex color string (e.g., "#RRGGBB")
      uint32_t col = strtol(value.c_str() + (value[0] == '#' ? 1 : 0), NULL, 16);
      settings.uiBgR = (uint8_t)((col >> 16) & 0xFF);
      settings.uiBgG = (uint8_t)((col >> 8) & 0xFF);
      settings.uiBgB = (uint8_t)(col & 0xFF);
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Background set to #%02X%02X%02X", settings.uiBgR, settings.uiBgG, settings.uiBgB);
    }
    else if (key == "uiPanelColor") {
      // Parse hex color string (e.g., "#RRGGBB")
      uint32_t col = strtol(value.c_str() + (value[0] == '#' ? 1 : 0), NULL, 16);
      settings.uiPanelR = (uint8_t)((col >> 16) & 0xFF);
      settings.uiPanelG = (uint8_t)((col >> 8) & 0xFF);
      settings.uiPanelB = (uint8_t)(col & 0xFF);
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Panel set to #%02X%02X%02X", settings.uiPanelR, settings.uiPanelG, settings.uiPanelB);
    }
    else if (key == "uiAccentColor") {
      // Parse hex color string (e.g., "#RRGGBB")
      uint32_t col = strtol(value.c_str() + (value[0] == '#' ? 1 : 0), NULL, 16);
      settings.uiAccentR = (uint8_t)((col >> 16) & 0xFF);
      settings.uiAccentG = (uint8_t)((col >> 8) & 0xFF);
      settings.uiAccentB = (uint8_t)(col & 0xFF);
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Accent set to #%02X%02X%02X", settings.uiAccentR, settings.uiAccentG, settings.uiAccentB);
    }
    else if (key == "uiTextColor") {
      // Parse hex color string (e.g., "#RRGGBB")
      uint32_t col = strtol(value.c_str() + (value[0] == '#' ? 1 : 0), NULL, 16);
      settings.uiTextR = (uint8_t)((col >> 16) & 0xFF);
      settings.uiTextG = (uint8_t)((col >> 8) & 0xFF);
      settings.uiTextB = (uint8_t)(col & 0xFF);
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Text set to #%02X%02X%02X", settings.uiTextR, settings.uiTextG, settings.uiTextB);
    }    
    else  {
      paramFound = false;
    }
    if (paramFound) {
      requestSaveEEPROM();
      request->send(200, "text/plain", "Ok");
    }
    else {
      request->send(404, "text/plain", "404: Parameter not found");
    }
  }
  else {
    request->send(400, "text/plain", "400: Invalid Request. Parameters: key and value");
  }
}


void handleSetManualTime(AsyncWebServerRequest *request) {
  // Accept either:
  //  - epoch (seconds) in POST param "epoch"
  //  - date "YYYY-MM-DD" in param "date" and time "HH:MM[:SS]" in param "time"
  if (!request->hasParam("epoch", true) && !(request->hasParam("date", true) && request->hasParam("time", true))) {
    request->send(400, "text/plain", "400: Invalid Request. Parameters: epoch OR (date and time)");
    return;
  }

  time_t epoch = 0;

  if (request->hasParam("epoch", true)) {
    String v = request->getParam("epoch", true)->value();
    epoch = (time_t) v.toInt();
  }
  else {
    String d = request->getParam("date", true)->value(); // YYYY-MM-DD
    String t = request->getParam("time", true)->value(); // HH:MM or HH:MM:SS

    int yy=0, mm=0, dd=0, hh=0, mi=0, ss=0;
    if (sscanf(d.c_str(), "%d-%d-%d", &yy, &mm, &dd) != 3) {
      request->send(400, "text/plain", "400: Invalid date format (YYYY-MM-DD)");
      return;
    }
    int n = sscanf(t.c_str(), "%d:%d:%d", &hh, &mi, &ss);
    if (n < 2) {
      request->send(400, "text/plain", "400: Invalid time format (HH:MM or HH:MM:SS)");
      return;
    }
    tmElements_t tm;
    tm.Year   = CalendarYrToTm(yy);
    tm.Month  = (uint8_t)mm;
    tm.Day    = (uint8_t)dd;
    tm.Hour   = (uint8_t)hh;
    tm.Minute = (uint8_t)mi;
    tm.Second = (uint8_t)ss;

    epoch = makeTime(tm);

    // Apply timezone offset and optional DST similarly to NTP logic (store local time)
    // makeTime assumes tm is local time already; we keep it as local epoch.
  }

  if (epoch < 100000) {
    request->send(400, "text/plain", "400: Epoch too small / invalid");
    return;
  }

  // Apply immediately
  setTime(epoch);
  lastTimeUpdate = millis();

  // Persist as fallback
  prm.manualTimeValid = true;
  prm.manualEpoch = (uint32_t)epoch;
  requestSaveEEPROM();

  // If RTC exists, sync it too
  if (RTCexist) updateRTC();

  request->send(200, "text/plain", "Ok");
}

void handleSendPublicConfig(AsyncWebServerRequest *request) {
  StaticJsonDocument<512> doc;

  doc["version"] = webName;
  doc["FW"] = FW;
  doc["tubeDriver"] = tubeDriver;
  doc["maxDigits"] = maxDigits;
  doc["maxBrightness"] = MAXBRIGHTNESS;

  if (settings.uiWidth == 0xFFFF) {
    doc["uiWidth"] = "100%";
  } else {
    doc["uiWidth"] = settings.uiWidth;
  }

  char uiBgHex[8];
  sprintf(uiBgHex, "#%02X%02X%02X", settings.uiBgR, settings.uiBgG, settings.uiBgB);
  doc["uiBgColor"] = uiBgHex;

  char uiPanelHex[8];
  sprintf(uiPanelHex, "#%02X%02X%02X", settings.uiPanelR, settings.uiPanelG, settings.uiPanelB);
  doc["uiPanelColor"] = uiPanelHex;

  char uiAccentHex[8];
  sprintf(uiAccentHex, "#%02X%02X%02X", settings.uiAccentR, settings.uiAccentG, settings.uiAccentB);
  doc["uiAccentColor"] = uiAccentHex;

  char uiTextHex[8];
  sprintf(uiTextHex, "#%02X%02X%02X", settings.uiTextR, settings.uiTextG, settings.uiTextB);
  doc["uiTextColor"] = uiTextHex;

  String output;
  serializeJson(doc, output);
  request->send(200, "application/json", output);
}

void handleSendConfig(AsyncWebServerRequest *request) {
  StaticJsonDocument<2048> doc;
  char buf[20];  //conversion buffer

  DPRINTLN("Sending configuration to web client.");

  //Global data
  doc["version"] = webName;
  doc["FW"] = FW;
  doc["tubeDriver"] = tubeDriver;
  doc["maxDigits"] = maxDigits;   //number of digits (tubes)
  doc["maxBrightness"] = MAXBRIGHTNESS; //Maximum tube brightness usually 10, sometimes 12

  //Actual time and environment data
  sprintf(buf, "%4d.%02d.%02d", year(), month(), day());
  doc["currentDate"] = buf;
  sprintf(buf, "%02d:%02d", hour(), minute());
  doc["currentTime"] = buf;

  // Time source status for UI
  const char* ts = "Unknown";
  if (WiFi.status() == WL_CONNECTED) {
    ts = "NTP/WiFi";
  } else if (RTCexist) {
    ts = "RTC";
  } else if (prm.manualTimeValid) {
    ts = "Manual";
  }
  doc["timeSource"] = ts;
  doc["wakeOnMotionEnabled"] = prm.wakeOnMotionEnabled;
  doc["tubesWakeSeconds"] = prm.tubesWakeSeconds;
  doc["manualDisplayOff"] = prm.manualDisplayOff;
  doc["onboardLed"] = onboardLedState;
  //DPRINT("useTemp:"); DPRINT(useTemp); DPRINT("useHumid:"); DPRINT(useHumid);
  if (useTemp > 0) {
    doc["temperature"] = round1(temperature[0] + prm.corrT0);  
  }
  else
    doc["temperature"] = 255;

  const float temp2corr = temperature[1] + prm.corrT1;
  if ((useTemp > 1) && isValidTemperatureReading(temp2corr))
    doc["temperature2"] = round1(temp2corr);
  else
    doc["temperature2"] = 255;

  if (useHumid>0) {
    doc["humidity"] = round1(humid[0] + prm.corrH0);
  }
  else
    doc["humidity"] = 255;
 
  const float humid2corr = humid[1] + prm.corrH1;
  if ((useHumid>1) && isValidHumidityReading(humid2corr))
    doc["humidity2"] = round1(humid2corr);
  else
    doc["humidity2"] = 255;

  if (usePress>0) {
    doc["pressure"] = pressur[0];
  }
  else
    doc["pressure"] = 255;
    
  if (useLux>0) {
    doc["lux"] = lx;
  }
  else
    doc["lux"] = 255;

  //Clock calculation and display parameters
  doc["utc_offset"] = prm.utc_offset;
  doc["enableDST"] = prm.enableDST;         // Flag to enable DST (summer time...)
  doc["set12_24"] = prm.set12_24;           // Flag indicating 12 vs 24 hour time (false = 12, true = 24);
  doc["showZero"] = prm.showZero;           // Flag to indicate whether to show zero in the hour ten's place
  doc["enableBlink"] = prm.enableBlink;     // Flag to indicate whether center colon should blink
  doc["interval"] = prm.interval;           // doc["interval in minutes, with 0 = off

  //Day/Night dimmer parameters
  doc["enableAutoShutoff"] = prm.enableAutoShutoff;  // Flag to enable/disable nighttime shut off
  doc["tubesSleep"] = prm.tubesSleep;              // Manual tubes sleep mode (wake on motion)
  doc["tubesPower"] = tubesPowerState;                       // Status: tubes currently ON/OFF
  doc["displayPower"] = !prm.manualDisplayOff;             // Manual display power (true=ON)
  doc["manualDisplayOff"] = prm.manualDisplayOff;
  doc["wakeOnMotionEnabled"] = prm.wakeOnMotionEnabled;
  doc["tubesWakeSeconds"] = prm.tubesWakeSeconds;                   // Motion wake duration (seconds)
  doc["touchShortAction"] = prm.touchShortAction;
  doc["touchDoubleAction"] = prm.touchDoubleAction;
  doc["touchLongAction"] = prm.touchLongAction;
  doc["gestureUpAction"] = settings.gestureUpAction;
  doc["gestureDownAction"] = settings.gestureDownAction;
  doc["gestureLeftAction"] = settings.gestureLeftAction;
  doc["gestureRightAction"] = settings.gestureRightAction;
  doc["gestureSensorPresent"] = isGestureSensorPresent();
  sprintf(buf, "%02d:%02d", prm.dayHour, prm.dayMin);
  doc["dayTime"] = buf;
  sprintf(buf, "%02d:%02d", prm.nightHour, prm.nightMin);
  doc["nightTime"] = buf;
  doc["dayBright"] = prm.dayBright;
  doc["nightBright"] = prm.nightBright;
  doc["animMode"] = prm.animMode;  //Tube animation
  doc["manualOverride"] = manualOverride;
//Alarm values
  doc["alarmEnable"] = prm.alarmEnable;   //1 = ON, 0 = OFF
  sprintf(buf, "%02d:%02d", prm.alarmHour, prm.alarmMin);
  doc["alarmTime"] = buf;
  doc["alarmPeriod"] = prm.alarmPeriod;

    // Home Assistant exposed settings
    doc["showTimeDate"] = prm.enableTimeDisplay;
    doc["showTemperature"] = settings.enableTempDisplay;
    doc["showHumidity"] = settings.enableHumidDisplay;
    doc["showPressure"] = settings.enablePressDisplay;
    doc["rgbAnimationSpeed"] = prm.rgbSpeed;

  //RGB LED values
  #if defined(USE_NEOPIXEL) || defined(USE_PWMLEDS)
    doc["rgbEffect"] = prm.rgbEffect;   // if 255, no RGB exist!
  #else
    doc["rgbEffect"] = 255;   //Not installed Neopixels!!!
  #endif
  
  doc["rgbBrightness"] = prm.rgbBrightness; // c_MinBrightness..255
  doc["maxLedmA"] = prm.maxLedmA;      // 0 disables limiter
  doc["rgbFixR"] = settings.rgbFixR;   // Fixed color R component (persisted)
  doc["rgbFixG"] = settings.rgbFixG;   // Fixed color G component (persisted)
  doc["rgbFixB"] = settings.rgbFixB;   // Fixed color B component (persisted)
  doc["rgbSpeed"] = prm.rgbSpeed;       // 1..255
  doc["rgbDir"] = prm.rgbDir;          // 0 = right direction, 1 = left direction
  doc["rgbMinBrightness"] = c_MinBrightness;  //minimum brightness for range check!!
  
  // === UI Customization Settings (persisted to EEPROM) ===
  // Page width (0xFFFF = 100%, otherwise pixels)
  if (settings.uiWidth == 0xFFFF) {
    doc["uiWidth"] = "100%";
  } else {
    doc["uiWidth"] = settings.uiWidth;
  }
  // Background color as hex string
  char uiBgHex[8];
  sprintf(uiBgHex, "#%02X%02X%02X", settings.uiBgR, settings.uiBgG, settings.uiBgB);
  doc["uiBgColor"] = uiBgHex;
  // Panel/Card color as hex string
  char uiPanelHex[8];
  sprintf(uiPanelHex, "#%02X%02X%02X", settings.uiPanelR, settings.uiPanelG, settings.uiPanelB);
  doc["uiPanelColor"] = uiPanelHex;
  // Accent color as hex string
  char uiAccentHex[8];
  sprintf(uiAccentHex, "#%02X%02X%02X", settings.uiAccentR, settings.uiAccentG, settings.uiAccentB);
  doc["uiAccentColor"] = uiAccentHex;
  // Text color as hex string
  char uiTextHex[8];
  sprintf(uiTextHex, "#%02X%02X%02X", settings.uiTextR, settings.uiTextG, settings.uiTextB);
  doc["uiTextColor"] = uiTextHex;
  
  doc["wifiMode"] = prm.wifiMode; 
  doc["wifiSsid"] = prm.wifiSsid;
  doc["wifiPsw"] = prm.wifiPsw;
  doc["ApSsid"] = prm.ApSsid;
  doc["ApPsw"] = prm.ApPsw;  
  doc["NtpServer"] = prm.NtpServer;
  doc["firmware"] = prm.firmwareServer;

  doc["mqttBrokerAddr"] = prm.mqttBrokerAddr;
  doc["mqttBrokerUser"] = prm.mqttBrokerUser;
  doc["mqttBrokerPsw"] = prm.mqttBrokerPsw;
  doc["mqttEnable"] = prm.mqttEnable;
  #if defined(USE_MQTT)
    doc["mqttBrokerRefresh"] = prm.mqttBrokerRefresh;  
  #else
    doc["mqttBrokerRefresh"] = 0;  
  #endif
  for (int i = 0; i < 4; i++) {
    doc[String("extDev") + i + "Name"] = externalDevices[i].name;
    doc[String("extDev") + i + "Topic"] = externalDevices[i].topic;
    doc[String("extDev") + i + "Slot"] = externalDevices[i].sensorSlot;
  }
  doc["dateMode"] = prm.dateMode; 
  doc["dateRepeatMin"] = prm.dateRepeatMin;   
  doc["tempRepeatMin"] = prm.tempRepeatMin;  
  doc["tempCF"] = prm.tempCF;   
  doc["enableTimeDisplay"] = prm.enableTimeDisplay; 
  doc["enableTempDisplay"] = settings.enableTempDisplay;
  doc["enableHumidDisplay"] = settings.enableHumidDisplay;
  doc["enablePressDisplay"] = settings.enablePressDisplay;
  doc["enableDoubleBlink"] = prm.enableDoubleBlink;
  doc["dateStart"] = prm.dateStart; 
  doc["dateEnd"] = prm.dateEnd; 
  doc["tempStart"] = prm.tempStart; 
  doc["tempEnd"] = prm.tempEnd; 
  doc["humidStart"] = prm.humidStart; 
  doc["humidEnd"] = prm.humidEnd; 
  doc["pressureStart"] = settings.pressureStart;
  doc["pressureEnd"] = settings.pressureEnd;
  doc["enableAutoDim"] = prm.enableAutoDim;
  doc["autoDim"] = prm.enableAutoDim; //alias for older/newer webpages
  doc["autoDimming"] = prm.enableAutoDim; //alias
doc["enableRadar"] = prm.enableRadar;    
  #if (RADAR_PIN >= 0) || defined(USE_MQTT)
    if (prm.radarTimeout<1) prm.radarTimeout = 1;
    doc["radarTimeout"] = prm.radarTimeout;   
  #else
    doc["radarTimeout"] = 0;
  #endif  
    doc["tubesWakeSeconds"] = prm.tubesWakeSeconds; // Motion wake duration (seconds)
  doc["displayPower"] = !prm.manualDisplayOff;             // Manual display power (true=ON)
doc["corrT0"] = prm.corrT0;
  doc["corrT1"] = prm.corrT1;
  doc["corrH0"] = prm.corrH0;
  doc["corrH1"] = prm.corrH1;
  doc["cathProtMin"] = 3;   //default value for cathProtMin slider
  String json;
  
// --- extras for MAIN status / time source ---
doc["manualDisplayOff"] = prm.manualDisplayOff;
doc["wakeOnMotionEnabled"] = prm.wakeOnMotionEnabled;
doc["tubesWakeSeconds"] = prm.tubesWakeSeconds;
doc["onboardLed"] = onboardLedState;

  // (removed duplicate timeSource/wakeSecondsLeft block)

  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

void handleSendClockDetails(AsyncWebServerRequest *request) {
  StaticJsonDocument<200> doc;
  //StaticJsonDocument<512> doc;
  String dat;
  const int validTempSensors = countValidTemperatureSensors();
  const int validHumidSensors = countValidHumiditySensors();

  DPRINTLN("SendClockDetails");
  dat = String("<br><br><strong>FirmwareID:") + FW + "<br>";
  dat += String("  Tube driver:") + tubeDriver + "<br>";
  dat += String("  Mac:") + String(WiFi.macAddress()) + "</strong><br>";
  dat += String("MAXBRIGHTNESS:") + String(MAXBRIGHTNESS) + "<br>";
  if (RTCexist) dat += String("RTC exist: YES <br>");
  if (validTempSensors > 0) dat += String("Temperature sensors:") + validTempSensors + "<br>";
  if (validHumidSensors > 0) dat += String("Humidity sensors:") + validHumidSensors + "<br>";
  if (usePress>0) dat += String("Pressure sensors:") + usePress + "<br>";
  dat += usedPinsStr + "<br>";
  dat += driverSetupStr + "<br>";
  DPRINTLN(dat);
  request->send(200, "text/html", dat);
}

void handleSendSystemInfo(AsyncWebServerRequest *request) {
  const unsigned long nowMs = millis();
  if (systemInfoCache.length() > 0 && (nowMs - systemInfoCacheMs) < 1000UL) {
    request->send(200, "application/json", systemInfoCache);
    return;
  }

  DynamicJsonDocument doc(12288);
  const int validTempSensors = countValidTemperatureSensors();
  const int validHumidSensors = countValidHumiditySensors();

  DPRINTLN("SendSystemInfo");
  
  // ESP32 Status
  doc["cpuTemp"] = (int)temperatureRead();  // Internal temperature sensor, returns Celsius
  doc["cpuCores"] = ESP.getChipCores();
  doc["cpuFreqMHz"] = ESP.getCpuFreqMHz();
  doc["chipModel"] = ESP.getChipModel();
  
  // Uptime in hours
  unsigned long uptimeMs = millis();
  uint32_t uptimeHours = uptimeMs / 1000 / 60 / 60;
  uint32_t uptimeMinutes = uptimeMs / 1000 / 60;
  doc["uptime"] = uptimeHours;
  doc["uptimeMinutes"] = uptimeMinutes;
  
  // Memory info
  uint32_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  uint32_t totalHeap = ESP.getHeapSize();
  uint32_t largestHeapBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  doc["freeHeap"] = freeHeap;
  doc["totalHeap"] = totalHeap;
  doc["heapUsedPercent"] = (int)(((totalHeap - freeHeap) * 100) / totalHeap);
  doc["largestFreeBlock"] = largestHeapBlock;
  doc["minFreeHeap"] = (heapMinFreeBytes == 0xFFFFFFFFUL) ? freeHeap : heapMinFreeBytes;
  doc["heapProtection"] = heapProtectionActive;
  
  // Network Info
  if (WiFi.status() == WL_CONNECTED) {
    String mac = WiFi.macAddress();
    String ssid = WiFi.SSID();
    String ip = WiFi.localIP().toString();
    String gw = WiFi.gatewayIP().toString();
    String sn = WiFi.subnetMask().toString();
    if (!isPrintableString(mac) || mac.indexOf(':') == -1) mac = "N/A";
    if (!isPrintableString(ip) || ip == "0.0.0.0") ip = "N/A";
    if (!isPrintableString(gw) || gw == "0.0.0.0") gw = "N/A";
    if (!isPrintableString(sn) || sn == "0.0.0.0") sn = "N/A";

    doc["wifiSignal"] = WiFi.RSSI();
    doc["wifiStatus"] = "Connected";
    doc["wifiSsid"] = ssid;
    doc["wifiIP"] = ip;
    doc["macAddress"] = mac;
    doc["ipAddress"] = ip;
    doc["gateway"] = gw;
    doc["subnet"] = sn;
  } else {
    doc["wifiSignal"] = -100;
    doc["wifiStatus"] = "Disconnected";
    doc["wifiSsid"] = "";
    doc["wifiIP"] = "--";
    doc["macAddress"] = "N/A";
    doc["ipAddress"] = "N/A";
    doc["gateway"] = "N/A";
    doc["subnet"] = "N/A";
  }

  doc["wifiSwitchPending"] = wifiSwitchPending;
  doc["wifiSwitchRollbackRunning"] = wifiSwitchRollbackRunning;
  doc["wifiSwitchResult"] = wifiSwitchLastResult;
  doc["wifiSwitchTargetSsid"] = wifiSwitchTargetSsid;
  doc["wifiSwitchRollbackSsid"] = wifiSwitchPrevSsid;

  // MQTT Info
  extern bool mqttConnected;
  doc["mqttStatus"] = mqttConnected ? "Connected" : "Disconnected";
  
  // Storage Info (SPIFFS)
  #ifdef USE_LITTLEFS
    doc["totalBytes"] = LittleFS.totalBytes();
    doc["usedBytes"] = LittleFS.usedBytes();
    doc["freeBytes"] = LittleFS.totalBytes() - LittleFS.usedBytes();
  #else
    doc["totalBytes"] = SPIFFS.totalBytes();
    doc["usedBytes"] = SPIFFS.usedBytes();
    doc["freeBytes"] = SPIFFS.totalBytes() - SPIFFS.usedBytes();
  #endif
  
  // EEPROM Info
  doc["eepromSize"] = EEPROM_SIZE;
  doc["eepromUsed"] = EEPROM_SIZE;  // Total used (both prm + settings structs)
  
  // Firmware Info
  doc["osVersion"] = "ClockForgeOS 1.0";
  doc["firmwareID"] = FW;
  doc["tubeDriver"] = tubeDriver;
  doc["maxDigits"] = maxDigits;
  doc["maxBrightness"] = MAXBRIGHTNESS;

  // Sensor counts
  doc["temperatureSensors"] = validTempSensors;
  doc["humiditySensors"] = validHumidSensors;
  doc["pressureSensors"] = usePress;

  String installedSensors = "";
  auto addInstalledSensor = [&](const char* name) {
    if (installedSensors.length() > 0) installedSensors += ", ";
    installedSensors += name;
  };
  if (useDallasTemp > 0) addInstalledSensor("Dallas DS18B20");
  if (BME280exist) addInstalledSensor("BME280");
  if (BMP280exist) addInstalledSensor("BMP280");
  if (AHTX0exist) addInstalledSensor("AHT10/AHT20");
  if (SHT21exist) addInstalledSensor("SHT21");
  if (BH1750exist) addInstalledSensor("BH1750");
  if (RTCexist) addInstalledSensor(RTCisPCF8563 ? "RTC PCF8563" : "RTC DS3231");
  if (LDRexist) addInstalledSensor("LDR");
  if (isGestureSensorPresent()) addInstalledSensor("APDS-9960 Gesture");
  doc["installedSensors"] = installedSensors.length() ? installedSensors : "None detected";
  doc["physicalSensors"] = installedSensors.length() ? installedSensors : "None detected";
  doc["gestureSensorPresent"] = isGestureSensorPresent();

  String virtualSensors = "";
  auto addVirtualSensor = [&](const char* name) {
    if (virtualSensors.length() > 0) virtualSensors += ", ";
    virtualSensors += name;
  };

  #ifdef USE_MQTT
    extern bool mqttMasterTempEnabled;
    extern bool mqttMasterHumidEnabled;
    extern bool mqttMasterRadarEnabled;
    extern bool mqttMasterLuxEnabled;
    extern bool mqttMasterPressureEnabled;

    if (mqttMasterTempEnabled) addVirtualSensor("MQTT Temp");
    if (mqttMasterHumidEnabled) addVirtualSensor("MQTT Humidity");
    if (mqttMasterRadarEnabled) addVirtualSensor("MQTT Radar");
    if (mqttMasterLuxEnabled) addVirtualSensor("MQTT Lux");
    if (mqttMasterPressureEnabled) addVirtualSensor("MQTT Pressure");

    for (int i = 0; i < 4; ++i) {
      if (strlen(externalDevices[i].topic) > 0) {
        String label = String("MQTT Ext") + String(i + 1);
        addVirtualSensor(label.c_str());
      }
    }
  #endif

  doc["virtualSensors"] = virtualSensors.length() ? virtualSensors : "None";

  // Used pins (as array of {num, label})
  JsonArray pinsArr = doc.createNestedArray("usedPins");
  for (byte i = 0; i < MAX_PIN; i++) {
    if (strlen(pinTxt[i]) > 0) {
      JsonObject p = pinsArr.createNestedObject();
      p["num"] = i;
      p["label"] = String(pinTxt[i]);
    }
  }

  // HV5122 pin settings (as array of arrays)
  #if defined(HV5122) || defined(NEWHV5122)
  extern byte digitPins[][10];
  JsonArray hvArr = doc.createNestedArray("hv5122Pins");
  for (int i = 0; i < maxDigits + 1; i++) {
    JsonArray row = hvArr.createNestedArray();
    for (int j = 0; j < 10; j++) {
      row.add(digitPins[i][j]);
    }
  }
  #endif

  String json;
  serializeJson(doc, json);
  systemInfoCache = json;
  systemInfoCacheMs = nowMs;
  request->send(200, "application/json", json);
}

void handleScanWifi(AsyncWebServerRequest *request) {
  if (wifiSwitchPending) {
    DynamicJsonDocument doc(192);
    doc["status"] = "busy";
    doc["reason"] = "wifi_connect_in_progress";
    doc["targetSsid"] = wifiSwitchTargetSsid;
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
    return;
  }

  // Return already prepared results first.
  if (wifiScanResults.length() > 0) {
    String r = wifiScanResults;
    wifiScanResults = "";
    DPRINTLN("Returning cached WiFi scan results");
    debugLogf("[WiFiScan] returning cached results");
    request->send(200, "application/json", r);
    return;
  }

  // If a scan is in progress, poll completion without disrupting WiFi link.
  if (wifiScanInProgress) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
      request->send(200, "application/json", String("{\"status\":\"scanning\"}"));
      return;
    }
    if (n < 0) {
      DPRINTLN("WiFi scan failed");
      debugLogf("[WiFiScan] scan failed (code=%d)", n);
      wifiScanInProgress = false;
      WiFi.scanDelete();
      request->send(200, "application/json", String("[]"));
      return;
    }

    DynamicJsonDocument doc(8192);
    JsonArray arr = doc.to<JsonArray>();
    for (int i = 0; i < n; ++i) {
      JsonObject it = arr.createNestedObject();
      it["ssid"] = WiFi.SSID(i);
      it["rssi"] = WiFi.RSSI(i);
      it["enc"] = WiFi.encryptionType(i);
    }

    String json;
    serializeJson(doc, json);
    wifiScanResults = json;
    wifiScanInProgress = false;
    WiFi.scanDelete();
    debugLogf("[WiFiScan] completed: %d network(s)", n);

    String r = wifiScanResults;
    wifiScanResults = "";
    request->send(200, "application/json", r);
    return;
  }

  // Start asynchronous scan and keep current WiFi/AP link alive.
  // AP scan is allowed because AP onboarding requires it.
  WiFiMode_t scanMode = WiFi.getMode();
  if (scanMode == WIFI_OFF) {
    WiFi.mode(WIFI_STA);
  } else if (scanMode == WIFI_AP) {
    WiFi.mode(WIFI_AP_STA);
    delay(20);
  }

  DPRINTLN("Starting async WiFi scan (non-disruptive)");
  debugLogf("[WiFiScan] start async scan (mode=%d)", (int)scanMode);
  int startResult = WiFi.scanNetworks(true, false);
  if (startResult == WIFI_SCAN_FAILED) {
    DPRINTLN("Failed to start WiFi scan");
    debugLogf("[WiFiScan] failed to start async scan");
    request->send(200, "application/json", String("[]"));
    return;
  }

  wifiScanInProgress = true;
  request->send(200, "application/json", String("{\"status\":\"scanning\"}"));
}

static inline void setWifiModeForSwitchKeepAp() {
  WiFiMode_t wm = WiFi.getMode();
  if ((wm == WIFI_AP) || (wm == WIFI_AP_STA)) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_STA);
  }
}

static void startWifiSwitchAttempt(const String& ssid, const String& psw, bool saveOnSuccess, AsyncWebServerRequest *request) {
  if (wifiSwitchPending) {
    DynamicJsonDocument doc(192);
    doc["status"] = "busy";
    doc["reason"] = "wifi_connect_in_progress";
    doc["targetSsid"] = wifiSwitchTargetSsid;
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
    return;
  }

  if (WiFi.status() == WL_CONNECTED && WiFi.SSID() == ssid) {
    String json = String("{\"status\":\"already_connected\",\"ssid\":\"") + ssid + String("\",\"ip\":\"") + WiFi.localIP().toString() + String("\"}");
    request->send(200, "application/json", json);
    return;
  }

  if (wifiScanInProgress) {
    WiFi.scanDelete();
    wifiScanInProgress = false;
  }

  wifiSwitchPrevSsid = "";
  wifiSwitchPrevPsw = "";
  wifiSwitchCanRollback = false;

  if (WiFi.status() == WL_CONNECTED) {
    wifiSwitchPrevSsid = WiFi.SSID();
    wifiSwitchPrevPsw = WiFi.psk();
    wifiSwitchCanRollback = (wifiSwitchPrevSsid.length() > 0);
  } else if (strlen(prm.wifiSsid) > 0) {
    wifiSwitchPrevSsid = String(prm.wifiSsid);
    wifiSwitchPrevPsw = String(prm.wifiPsw);
    wifiSwitchCanRollback = true;
  }

  wifiSwitchTargetSsid = ssid;
  wifiSwitchTargetPsw = psw;
  wifiSwitchSaveOnSuccess = saveOnSuccess;
  wifiSwitchRollbackRunning = false;
  wifiSwitchStartMs = millis();
  wifiSwitchPending = true;
  wifiSwitchLastResult = "connecting";
  wifiRecoveryApMode = false;

  setWifiModeForSwitchKeepAp();
  delay(50);
  if (psw.length() > 0) {
    DPRINTF("WiFi.begin(%s, %s)\n", ssid.c_str(), psw.c_str());
    WiFi.begin(ssid.c_str(), psw.c_str());
  } else {
    DPRINTF("WiFi.begin(%s)\n", ssid.c_str());
    WiFi.begin(ssid.c_str());
  }

  DynamicJsonDocument doc(256);
  doc["status"] = "connecting";
  doc["targetSsid"] = ssid;
  doc["rollback"] = wifiSwitchCanRollback;
  if (wifiSwitchCanRollback) doc["rollbackSsid"] = wifiSwitchPrevSsid;
  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

void handleConnectWifi(AsyncWebServerRequest *request) {
  String ssid = "";
  String psw = "";
  if (request->hasParam("ssid")) ssid = request->getParam("ssid")->value();
  if (request->hasParam("psw")) psw = request->getParam("psw")->value();
  DPRINT("ConnectWifi: "); DPRINTLN(ssid);
  if (ssid.length() == 0) {
    request->send(400, "text/plain", "Missing ssid");
    return;
  }
  startWifiSwitchAttempt(ssid, psw, false, request);
}

void handleConnectWifiPost(AsyncWebServerRequest *request) {
  // Expect POST form parameters: ssid, psw, save (optional: "true")
  String ssid = "";
  String psw = "";
  bool save = false;
  if (request->hasParam("ssid", true)) ssid = request->getParam("ssid", true)->value();
  if (request->hasParam("psw", true)) psw = request->getParam("psw", true)->value();
  if (request->hasParam("save", true)) save = (request->getParam("save", true)->value() == "true");

  DPRINT("ConnectWifiPost: "); DPRINTLN(ssid);
  if (ssid.length() == 0) {
    request->send(400, "text/plain", "Missing ssid");
    return;
  }

  startWifiSwitchAttempt(ssid, psw, save, request);
}

void handleWifiStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<512> doc;
  int st = WiFi.status();
  doc["status"] = st;
  const char* sstr = "Disconnected";
  switch (st) {
    case WL_CONNECTED: sstr = "Connected"; break;
    case WL_CONNECT_FAILED: sstr = "ConnectFailed"; break;
    case WL_CONNECTION_LOST: sstr = "ConnectionLost"; break;
    case WL_NO_SSID_AVAIL: sstr = "NoSsidAvailable"; break;
    default: break;
  }
  doc["statusStr"] = sstr;
  if (st == WL_CONNECTED) {
    doc["rssi"] = WiFi.RSSI();
    doc["ip"] = WiFi.localIP().toString();
    doc["ssid"] = WiFi.SSID();
  } else {
    doc["rssi"] = 0;
    doc["ip"] = "0.0.0.0";
    doc["ssid"] = "";
  }
  doc["wifiSwitchPending"] = wifiSwitchPending;
  doc["wifiSwitchRollbackRunning"] = wifiSwitchRollbackRunning;
  doc["wifiSwitchResult"] = wifiSwitchLastResult;
  doc["wifiSwitchTargetSsid"] = wifiSwitchTargetSsid;
  doc["wifiSwitchRollbackSsid"] = wifiSwitchPrevSsid;
  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

void handleSendCurrentInfos(AsyncWebServerRequest *request) {
  const unsigned long nowMs = millis();
  if (currentInfosCache.length() > 0 && (nowMs - currentInfosCacheMs) < 500UL) {
    request->send(200, "application/json", currentInfosCache);
    return;
  }

  StaticJsonDocument<896> doc;
  //StaticJsonDocument<512> doc;
  char buf[20];  //conversion buffer
  //Actual time and environment data
  sprintf(buf, "%4d.%02d.%02d", year(), month(), day());
  doc["currentDate"] = buf;
  sprintf(buf, "%02d:%02d", hour(), minute());
  doc["currentTime"] = buf;
  //DPRINT("useTemp:"); DPRINT(useTemp); DPRINT("useHumid:"); DPRINT(useHumid);
  if (useTemp > 0) {
    doc["temperature1"] = temperature[0] + prm.corrT0;
    doc["temperature"] = temperature[0] + prm.corrT0;  //for compatibility with the old web page
  }
  else
    doc["temperature1"] = 255;

  const float temp2corr = temperature[1] + prm.corrT1;
  if ((useTemp > 1) && isValidTemperatureReading(temp2corr))
    doc["temperature2"] = temp2corr;
  else
    doc["temperature2"] = 255;

  if (useHumid>0) {
    doc["humidity1"] = humid[0] + prm.corrH0;
    doc["humidity"] = humid[0] + prm.corrH0;  //for compatibility with the old web page
  }
  else
    doc["humidity1"] = 255;
 
  const float humid2corr = humid[1] + prm.corrH1;
  if ((useHumid>1) && isValidHumidityReading(humid2corr))
    doc["humidity2"] = humid2corr;
  else
    doc["humidity2"] = 255;
  
  if (usePress>0) {
    doc["pressure"] = pressur[0];
  }
  else
    doc["pressure"] = 255;

  if (useLux>0) {
    doc["lux"] = lx;
  }
  else
    doc["lx"] = 255;
  extern bool mqttConnected;
  doc["rssi"] = WiFi.RSSI();
  doc["telnetClients"] = telnetActiveConnections();
  doc["mqttClients"] = mqttConnected ? 1 : 0;
  doc["mqttStatus"] = mqttConnected ? "Connected" : "Disconnected";
  doc["tubesPower"] = tubesPowerState;
  doc["cathodeProtRunning"] = cathodeProtRunning;
  // Effective day/night mode indicator for UI (right side = NIGHT)
  doc["dayNightIsNight"] = !displayON;
  doc["dayNightMode"] = displayON ? "DAY" : "NIGHT";

// --- MAIN UI extras (time source + sleep status) ---
doc["manualDisplayOff"] = prm.manualDisplayOff;
doc["wakeOnMotionEnabled"] = prm.wakeOnMotionEnabled;
doc["tubesWakeSeconds"] = prm.tubesWakeSeconds;

#ifdef USE_NEOPIXEL
doc["ledCurrentmA"] = neoAppliedCurrentmA;
doc["ledAppliedBrightness"] = neoAppliedBrightness;
#else
doc["ledCurrentmA"] = 0;
doc["ledAppliedBrightness"] = 0;
#endif
doc["ledLimitmA"] = prm.maxLedmA;
doc["onboardLed"] = onboardLedState;

// Compute wake seconds left based on last motion and configured wake time
uint16_t wakeLeft = 0;
unsigned long wakeMs = 0;
if (prm.tubesWakeSeconds > 0) wakeMs = (unsigned long)prm.tubesWakeSeconds * 1000UL;
else if (prm.radarTimeout > 0) wakeMs = (unsigned long)prm.radarTimeout * 60UL * 1000UL;
else wakeMs = 45000UL;

if (prm.wakeOnMotionEnabled && tubesLastMotionMs != 0 && wakeMs > 0) {
  unsigned long nowMs = millis();
  unsigned long elapsedMs = nowMs - tubesLastMotionMs;
  if (elapsedMs < wakeMs) {
    wakeLeft = (uint16_t)((wakeMs - elapsedMs + 999UL) / 1000UL);
  }
}
doc["wakeSecondsLeft"] = wakeLeft;

// Time source label (what MAIN shows)
const char* timeSrc = "Not Set";
if (WiFi.status() == WL_CONNECTED) timeSrc = "NTP/WiFi";
else if (RTCexist) timeSrc = "RTC";
else if (prm.manualTimeValid) timeSrc = "Manual";
doc["timeSource"] = timeSrc;

  String json;
  serializeJson(doc, json);
  currentInfosCache = json;
  currentInfosCacheMs = nowMs;
  request->send(200, "application/json", json);
}

void setup() {
  bool bootNeedsInitialSetup = false;
  #if defined(DISABLE_BROWNOUT) && defined(ESP32)
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector 
  #endif  
  forceOnboardLedOffEarly();
  delay(1000);
  WiFi.mode(WIFI_OFF);
  EEPROM.begin(EEPROM_SIZE);
  memset(pinTxt,0,sizeof(pinTxt));
  DPRINTBEGIN(115200); DPRINTLN(" ");
  DPRINTLN(F("================================================="));
  DPRINT("Starting "); DPRINTLN(webName);
  DPRINT("Firmware code:"); DPRINT(FW); DPRINT("   Tube driver:"); DPRINTLN(tubeDriver);
  DPRINT("MAXBRIGHTNESS:"); DPRINTLN(MAXBRIGHTNESS);
#if defined(ESP32)
  auto resetReasonToText = [](esp_reset_reason_t rr) -> const char* {
    switch (rr) {
      case ESP_RST_POWERON:   return "POWERON";
      case ESP_RST_EXT:       return "EXTERNAL";
      case ESP_RST_SW:        return "SOFTWARE";
      case ESP_RST_PANIC:     return "PANIC";
      case ESP_RST_INT_WDT:   return "INT_WDT";
      case ESP_RST_TASK_WDT:  return "TASK_WDT";
      case ESP_RST_WDT:       return "WDT";
      case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
      case ESP_RST_BROWNOUT:  return "BROWNOUT";
      case ESP_RST_SDIO:      return "SDIO";
      default:                return "UNKNOWN";
    }
  };
  esp_reset_reason_t rr = esp_reset_reason();
  DPRINT("Reset reason: ");
  DPRINT(resetReasonToText(rr));
  DPRINT(" (");
  DPRINT((int)rr);
  DPRINTLN(")");
#endif
  DPRINTLN(F("================================================="));
  clearDigits();
  onboardLedState = false;
  applyOnboardLedState();
  setup_pins();
  #if ALARMSPEAKER_PIN >= 0
    pinMode(ALARMSPEAKER_PIN, OUTPUT); regPin(ALARMSPEAKER_PIN,"ALARMSPEAKER_PIN");
    digitalWrite(ALARMSPEAKER_PIN,!ALARM_ON);
    DPRINT("  - ON state:");
    if (ALARM_ON == HIGH) {DPRINTLN("HIGH"); }
    else {DPRINTLN("LOW"); }
  #endif
  #if ALARMBUTTON_PIN >= 0
    pinMode(ALARMBUTTON_PIN, INPUT_PULLUP); regPin(ALARMBUTTON_PIN,"ALARMBUTTON_PIN");
  #endif
  #if COLON_PIN >= 0
    pinMode(COLON_PIN, OUTPUT); regPin(COLON_PIN,"COLON_PIN");
  #endif
  #if LED_SWITCH_PIN >= 0
    pinMode(LED_SWITCH_PIN, OUTPUT); regPin(LED_SWITCH_PIN,"LED_SWITCH_PIN");
  #endif
  #if ONBOARD_LED_PIN >= 0
    regPin(ONBOARD_LED_PIN,"ONBOARD_LED_PIN");
  #endif
  #if defined(ESP32) && (TOUCH_BUTTON_PIN >= 0)
    regPin(TOUCH_BUTTON_PIN,"TOUCH_BUTTON_PIN");
  #endif
  #if DECIMALPOINT_PIN >= 0
    pinMode(DECIMALPOINT_PIN, OUTPUT); regPin(DECIMALPOINT_PIN,"DECIMALPOINT_PIN");
  #endif
  #if RADAR_PIN >= 0
    pinMode(RADAR_PIN, INPUT);  regPin(RADAR_PIN,"RADAR_PIN");
    DPRINT("  - RADAR Timeout:");   DPRINTLN(prm.radarTimeout);
  #endif
  #if TUBE_POWER_PIN >= 0
    pinMode(TUBE_POWER_PIN, OUTPUT); regPin(TUBE_POWER_PIN,"TUBE_POWER_PIN");
    digitalWrite(TUBE_POWER_PIN,TUBE_POWER_ON);
    DPRINT("  - ON state:"); 
    if (TUBE_POWER_ON == HIGH) {DPRINTLN("HIGH"); }
    else {DPRINTLN("LOW"); }
  #endif
  #if LIGHT_SENSOR_PIN >= 0
    pinMode(LIGHT_SENSOR_PIN, INPUT); regPin(LIGHT_SENSOR_PIN,"LIGHT_SENSOR_PIN");
    DPRINT("  - MAXIMUM_LUX for MAXBRIGHTNESS:");  DPRINTLN(MAXIMUM_LUX);
    useLux++;
    LDRexist = true;
  #endif

  #ifdef USE_PWMLEDS
    #if PWM1_PIN >= 0
      pinMode(PWM1_PIN, OUTPUT);  regPin(PWM1_PIN,"PWM1_PIN");
      ledcAttachPin(PWM1_PIN, 0);
      ledcSetup(0, 200, 8); // 12
    #endif
    #if PWM2_PIN >= 0
      pinMode(PWM2_PIN, OUTPUT);  regPin(PWM2_PIN,"PWM2_PIN");
      ledcAttachPin(PWM2_PIN, 1);
      ledcSetup(1, 200, 8); // 12
    #endif
    #if PWM3_PIN >= 0
      pinMode(PWM3_PIN, OUTPUT);  regPin(PWM3_PIN,"PWM3_PIN");
      ledcAttachPin(PWM3_PIN, 2);
      ledcSetup(2, 200, 8); // 12
    #endif
  #endif
  
  #if defined(PIN_EXTINP1)
    if (PIN_EXTINP1>=0)
      pinMode(PIN_EXTINP1, INPUT); regPin(PIN_EXTINP1,"NOT_USED_INPUT");
  #endif
  #if defined(PIN_EXTINP2)
    if (PIN_EXTINP2>=0)
      pinMode(PIN_EXTINP2, INPUT); regPin(PIN_EXTINP2,"NOT_USED_INPUT");
  #endif
      
  #if TEMP_DALLAS_PIN >= 0 
    setupDallasTemp();
  #endif
  #if TEMP_DHT_PIN >= 0
    setupDHTemp();
  #endif
  #ifdef USE_I2CSENSORS
    setupI2Csensors();
  #endif
  setupGestureSensor();
  
  #ifdef USE_RTC
    setupRTC();
  #endif
  
  #ifdef USE_GPS
    setupGPS();
  #endif

  decimalpointON = false;   
  DPRINT("Number of digits:"); DPRINTLN(maxDigits); 

  #ifdef FACTORYRESET_PIN
  if (FACTORYRESET_PIN>=0) {
      DPRINTLN("FactoryReset button is pushed!");
      pinMode(FACTORYRESET_PIN,INPUT_PULLUP);  regPin(FACTORYRESET_PIN,"FACTORYRESET_PIN");
      if (digitalRead(FACTORYRESET_PIN)==LOW) {
        factoryReset();
        bootNeedsInitialSetup = true;
      }
  }
  #endif
  loadEEPROM();
  if (prm.magic != MAGIC_VALUE) {
    factoryReset();
    bootNeedsInitialSetup = true;
  }

  if (strlen(prm.wifiSsid) == 0) {
    bootNeedsInitialSetup = true;
  }

  applyOnboardLedState();

  // Keep persisted RGB effect across reboots; only sanitize invalid EEPROM values.
  if (prm.rgbEffect > 11) {
    prm.rgbEffect = 1;
    debugLogf("[RGB] Boot: Invalid rgbEffect in EEPROM, fallback -> 1");
  }

  bool touchMappingMigrated = false;
  if (prm.touchShortAction == TOUCH_ACTION_DISPLAY_OFF) {
    prm.touchShortAction = TOUCH_ACTION_DISPLAY_TOGGLE;
    touchMappingMigrated = true;
  }
  if (prm.touchDoubleAction == TOUCH_ACTION_DISPLAY_OFF) {
    prm.touchDoubleAction = TOUCH_ACTION_DISPLAY_TOGGLE;
    touchMappingMigrated = true;
  }
  if (prm.touchLongAction == TOUCH_ACTION_DISPLAY_OFF) {
    prm.touchLongAction = TOUCH_ACTION_DISPLAY_TOGGLE;
    touchMappingMigrated = true;
  }
  if (touchMappingMigrated) {
    debugLogf("[Touch33] migrated legacy Display OFF action -> Display Toggle");
    requestSaveEEPROM();
  }

  // Apply persistent debug setting after loading EEPROM
  uiDebugEnabled = settings.debugEnabled;
  //If user previously set the time manually (e.g. no internet), apply it immediately.
  if (prm.manualTimeValid && (prm.manualEpoch > 100000)) {
    setTime((time_t)prm.manualEpoch);
    DPRINTLN("Manual time loaded from EEPROM.");
  }

  // NTPClient is constructed at global scope, so at that time prm.NtpServer may still be empty
  // (EEPROM not loaded yet). Ensure we set the pool server name now, after EEPROM/factory defaults.
  if (strlen(prm.NtpServer) == 0) {
    strncpy(prm.NtpServer, "pool.ntp.org", sizeof(prm.NtpServer));
    prm.NtpServer[sizeof(prm.NtpServer) - 1] = '\0';
  }
  timeClient.setPoolServerName(prm.NtpServer);
  timeClient.setUpdateInterval(TIMESERVER_REFRESH);
  
  setupNeopixel();
  listPins();
  writeAlarmPin(ALARM_ON); writeAlarmPin(!ALARM_ON);
  getDHTemp();  //get the first DHT temp+humid measure
    
  byte saveMode = prm.animMode;
  prm.animMode = 2;
  if (tubesPowerState) {
    doAnimationPWM();
  } else {
    // Display is OFF: keep ambient LEDs OFF and stop PWM animation.
  }
  testTubes(300);
  startupMuteRgb = true; // Keep LEDs OFF between tube test and end of IP display

  clearDigits();
  disableDisplay();
  Fdelay(200);
  if (!SPIFFS.begin()) {
    DPRINTLN("An Error has occurred while mounting SPIFFS");
  }
  else {
    #if defined(ESP32)
      DPRINT("SPIFFS Total bytes:    "); DPRINTLN(SPIFFS.totalBytes());
      DPRINT("SPIFFS Used bytes:     "); DPRINTLN(SPIFFS.usedBytes());
    #endif
    DPRINTLN("SPIFFS started.");
    if(!SPIFFS.exists("/index.html")) {DPRINTLN("/index.html not found!");}
    if(!SPIFFS.exists("/site.css")) {DPRINTLN("/site.css not found!");}
    if(!SPIFFS.exists("/page.js")) {DPRINTLN("/page.js not found!");}
    if(!SPIFFS.exists("/jquery_360.js")) {DPRINTLN("/jquery_360.js not found!");}
  }
  memset(digit,10,sizeof(digit));   //clear display
  memset(newDigit,10,sizeof(newDigit));
  memset(oldDigit,10,sizeof(oldDigit));
  
  wifiConnectRunning = true;
  writeDisplay2();
  if (bootNeedsInitialSetup) {
    DPRINTLN("Initial setup mode: starting standalone AP for configuration.");
    startStandaloneMode();
  }
  else if (prm.wifiMode) {
    //findBestWifi();
    startWifiMode();

   #ifdef USE_WIFIMANAGER
   int counter = 0;
    while (WiFi.status() != WL_CONNECTED) {   //try to use wifiManeger, if enabled
      DPRINT('.');
      Fdelay(3000);
      if (counter++>2) {
        wifiManager();
        break;
      }
    }
    DPRINTLN(" ");
   #endif
    
    if (WiFi.status() != WL_CONNECTED) { //failed to connect to wifi
      startStandaloneMode();
    }
    else {
      ip = WiFi.localIP();
      WiFi.setAutoReconnect(true);
      enableDisplay(100);
    }
  }
  else {
    startStandaloneMode();
  }
  wifiConnectRunning = false;

  startMDNS();
  startServer();

telnetServer.begin();
telnetServer.setNoDelay(true);
debugLogLine("Remote logging ready: SSE /events + Telnet :23");

  #ifdef USE_MQTT
    setupMqtt();
  #endif
  
  showMyIp();
  Fdelay(500);
  startupMuteRgb = false;
  clearDigits();
  Fdelay(200);
  enableDisplay(0);
  prm.animMode = saveMode;
  timeClient.begin();
  calcTime();
  timeProgram();

  //newCathodeProtect(5000,0);
}

void calcTime() {
  boolean refreshed = false;
  boolean refreshed2 = false;
  
  if (WiFi.status() == WL_CONNECTED) {        //check wifi connection
    refreshed = updateTimefromTimeserver();  //update time from wifi
  }  
  if (GPSexist) { //update time from GPS, if exist
    refreshed2 = getGPS();
  }
  
  if (RTCexist) {  //update time from RTC, if exist
    if (refreshed || refreshed2) 
      updateRTC();    //update RTC, if needed
    else 
      getRTC();
  }
}


void timeProgram() {
  static unsigned long lastRun = 0;
  
  if ((millis() - lastRun) < 200) return;
  lastRun = millis();
  calcTime();
  if (useDallasTemp > 0) {
    requestDallasTemp(false);  //start measure
    getTemp();                 //get result, if ready
  }
  getDHTemp();
  getI2Csensors();             //check all existing I2C sensors
  
  #ifdef SIMULATE_TEMPERATURE_SENSOR   //for test only
    useTemp = 2;  temperature[0] = 20.4; temperature[1] = 22.8;  //test only
    useHumid = 2; humid[0] = 41.2; humid[1] = 42.7;
  #endif
  
  if (now() != prevTime) { // Update time every second
    prevTime = now();
    evalShutoffTime();     // Check whether display should be turned off (Auto shutoff mode)
    if (!displayON || !prm.enableBlink) colonBlinkState = false;
    else colonBlinkState = (bool)(second() % 2);
    
    showClock = false;
    showDate = (prm.enableTimeDisplay && (prm.dateRepeatMin>0) && ((second() >= prm.dateStart) && (second() < prm.dateEnd)));
    if (showDate && (prm.dateRepeatMin>1)) {
      if ((minute() % prm.dateRepeatMin) != 0) {
        showDate = false;
      }
    }
//useTemp = 1; temperature[0] = -12.34; useHumid = 1; humid[0] = 26.7; //for test only
  float t1corr = temperature[1] + prm.corrT1;
  float h1corr = humid[1] + prm.corrH1;
  bool temp2Valid = isValidTemperatureReading(t1corr);
  bool humid2Valid = isValidHumidityReading(h1corr);
    showTemp0 = settings.enableTempDisplay && (useTemp > 0) && (second() >= prm.tempStart) && (second() < prm.tempEnd);
  showTemp1 = settings.enableTempDisplay && (useTemp > 1) && temp2Valid && (second() >= prm.tempStart + (prm.tempEnd - prm.tempStart) / 2) && (second() < prm.tempEnd);
    showHumid0 = settings.enableHumidDisplay && (useHumid > 0) && (second() >= prm.humidStart) && (second() < prm.humidEnd);
  showHumid1 = settings.enableHumidDisplay && (useHumid >1) && humid2Valid && (second() >= prm.humidStart + (prm.humidEnd-prm.humidStart)/2) && (second() < prm.humidEnd);
    showPress0 = settings.enablePressDisplay && (usePress > 0) && (second() >= settings.pressureStart) && (second() < settings.pressureEnd);
    if ((prm.tempRepeatMin==0) || ((minute() % prm.tempRepeatMin) != 0)) {
        showTemp0 = false;
        showTemp1 = false;
        showHumid0 = false;
        showHumid1 = false;
      showPress0 = false;
    }
    if (maxDigits >= 8)      displayTime8();
    else if (maxDigits == 6) displayTime6();
    else                     displayTime4();
    //   if (COLON_PIN>=0) digitalWrite(COLON_PIN,colonBlinkState);  // Blink colon pin
    #if LED_SWITCH_PIN >= 0
      if (displayON) //Switch on backlight LED only daytime
        digitalWrite(LED_SWITCH_PIN, HIGH);
      else
        digitalWrite(LED_SWITCH_PIN, LOW);
    #endif
    changeDigit();
    printDigits(0);
    
    if (prm.interval > 0) {  // protection is enabled
      // At the first top of the hour, initialize protection logic timer
      if (!initProtectionTimer && (minute() == 0)) {
        protectTimer = 0;   // Ensures protection logic will be immediately triggered
        initProtectionTimer = true;
      }
      if ((now() - protectTimer) >= 60 * prm.interval) {
        protectTimer = now();
        // The current time can drift slightly relative to the protectTimer when NIST time is updated
        // Need to make a small adjustment to the timer to ensure it is triggered at the minute change
        protectTimer -= ((second() + 30) % 60 - 30);
        if (displayON && (millis() > 50000)) newCathodeProtect(maxDigits*1500,random(3)-1);   //dont play in the first 50sec
      }
    }    
  }
}

void loadEEPROM() {
  disableDisplay();
  DPRINT("Loading setting from EEPROM.  Size:");  DPRINT(EEPROM_SIZE);
  
  EEPROM.get(EEPROM_addr, prm);
  // Load separate user settings struct
  settingsLoadFromEEPROM();
  // Backward-compatible sanity for newly added fields (older EEPROM will have 0xFFFF).
  if (prm.maxLedmA == 0xFFFF || prm.maxLedmA > 2000) prm.maxLedmA = 350;
  // 0 disables the limiter intentionally; keep 0 as-is.
  DPRINT("  version:"); DPRINTLN(prm.magic);
  enableDisplay(0);
}

void saveEEPROM() {
  disableDisplay();
  // Write main parameters block
  EEPROM.put(EEPROM_addr, prm);
  // Write separate user settings block
  EEPROM.put(SETTINGS_EEPROM_ADDR, settings);
  EEPROM.commit();
  settingsDirty = false;
  DPRINTLN("Settings saved to EEPROM!");
  enableDisplay(0);
}

//-------------------------------------------------------------------------------------------------
// settings.h implementation (persistent UI/user settings separate from model config)
// Stored in EEPROM at SETTINGS_EEPROM_ADDR (after prm block).
void settingsSetDefaults() {
  settings.schemaVersion = SETTINGS_SCHEMA_VERSION;
  settings.debugEnabled = false;

  // Fixed RGB defaults (white)
  settings.rgbFixR = 255;
  settings.rgbFixG = 255;
  settings.rgbFixB = 255;

  // UI Customization defaults (dark theme with orange accent)
  settings.uiWidth = 800;       // 800px default
  settings.uiBgR = 3;
  settings.uiBgG = 2;
  settings.uiBgB = 2;
  settings.uiPanelR = 15;
  settings.uiPanelG = 6;
  settings.uiPanelB = 6;
  settings.uiAccentR = 0xD9;    // #D96025 (orange accent)
  settings.uiAccentG = 0x60;
  settings.uiAccentB = 0x25;
  settings.uiTextR = 0xDD;      // #DDDDDD (light text)
  settings.uiTextG = 0xDD;
  settings.uiTextB = 0xDD;
  settings.enableTempDisplay = true;
  settings.enableHumidDisplay = true;
  settings.enablePressDisplay = true;
  settings.pressureStart = 50;
  settings.pressureEnd = 55;
  settings.gestureUpAction = TOUCH_ACTION_COLOR_CHANGE;
  settings.gestureDownAction = TOUCH_ACTION_COLOR_PREV;
  settings.gestureLeftAction = TOUCH_ACTION_COLOR_PREV;
  settings.gestureRightAction = TOUCH_ACTION_COLOR_CHANGE;

  settings.mqttInTempTopic[0] = '\0';
  settings.mqttInHumidTopic[0] = '\0';
  settings.mqttInRadarTopic[0] = '\0';
  settings.mqttInLuxTopic[0] = '\0';
  settings.mqttInPressureTopic[0] = '\0';

  // Also reset prm.rgbEffect to fixed color when settings are reset
  prm.rgbEffect = 1;
  
  settingsMarkDirty();
}

bool settingsLoadFromEEPROM() {
  auto sanitizeTopic = [](char* topic, size_t len) {
    if (len == 0) return;
    topic[len - 1] = '\0';
    for (size_t i = 0; i < len && topic[i] != '\0'; i++) {
      unsigned char c = (unsigned char)topic[i];
      if (c < 32 || c > 126) {
        topic[0] = '\0';
        break;
      }
    }
  };

  EEPROM.get(SETTINGS_EEPROM_ADDR, settings);

  // Validate schema
  if (settings.schemaVersion != SETTINGS_SCHEMA_VERSION) {
    settingsSetDefaults();
    debugLogf("settings: schema mismatch (got 0x%04X, want 0x%04X) -> defaults",
              settings.schemaVersion, SETTINGS_SCHEMA_VERSION);
    return false;
  }

  // Backward compatible sanity (older EEPROM may have 0xFF/0x00 noise)
  if (!(settings.debugEnabled == true || settings.debugEnabled == false)) {
    settings.debugEnabled = false;
  }

  sanitizeTopic(settings.mqttInTempTopic, sizeof(settings.mqttInTempTopic));
  sanitizeTopic(settings.mqttInHumidTopic, sizeof(settings.mqttInHumidTopic));
  sanitizeTopic(settings.mqttInRadarTopic, sizeof(settings.mqttInRadarTopic));
  sanitizeTopic(settings.mqttInLuxTopic, sizeof(settings.mqttInLuxTopic));
  sanitizeTopic(settings.mqttInPressureTopic, sizeof(settings.mqttInPressureTopic));

  return true;
}

void settingsSaveToEEPROM() {
  EEPROM.put(SETTINGS_EEPROM_ADDR, settings);
  EEPROM.commit();
  settingsDirty = false;
}

// Immediate RGB save: persist both RGB settings AND rgbEffect mode without display glitch
void saveRGBSettingsToEEPROM() {
  // Save the prm struct (contains rgbEffect)
  EEPROM.put(EEPROM_addr, prm);
  // Save the settings struct (contains rgbFixR/G/B)
  EEPROM.put(SETTINGS_EEPROM_ADDR, settings);
  EEPROM.commit();
  settingsDirty = false;
  eepromSavePending = false;  // Clear any pending general save
}




void requestSaveEEPROM() {
  eepromSavePending = true;
}

void processPendingEepromSave() {
  if (!eepromSavePending) return;

  const uint32_t nowMs = millis();
  // Debounce / rate-limit EEPROM writes
  if ((nowMs - lastEepromSaveMs) < EEPROM_SAVE_MIN_INTERVAL_MS) return;

  lastEepromSaveMs = nowMs;

  // Perform the actual write + commit
  saveEEPROM();

  // Clear the pending flag *after* saving
  eepromSavePending = false;
}


void factoryReset() {
  DPRINTLN("Factory Reset!!!");
  prm.alarmEnable = false;
  prm.alarmHour = 7;
  prm.alarmMin = 0;
  prm.alarmPeriod = 15;
  prm.rgbEffect = 2;
  prm.rgbBrightness = 80;
  prm.maxLedmA = 350;
  prm.rgbFixColor = 150;
  prm.rgbSpeed = 238;
  prm.rgbDir = 0;
  prm.wifiMode = true;
  #ifdef DEFAULT_SSID
    strncpy(prm.wifiSsid,DEFAULT_SSID,sizeof(prm.wifiSsid));
  #else
    strcpy(prm.wifiSsid,"");
  #endif
  #ifdef DEFAULT_PSW
    strncpy(prm.wifiPsw,DEFAULT_PSW,sizeof(prm.wifiPsw));
  #else
    strcpy(prm.wifiPsw,"");
  #endif  
  for (int i=0;i<(int)strlen(prm.ApSsid);i++) {  //repair bad chars in AP SSID
    if ((prm.ApSsid[i]<32) || (prm.ApSsid[i]>126)) prm.ApSsid[i]='_';
  }
  strncpy(prm.ApSsid, FACTORY_DEFAULT_AP_NAME, sizeof(prm.ApSsid) - 1);
  prm.ApSsid[sizeof(prm.ApSsid) - 1] = '\0';
  strncpy(prm.ApPsw, FACTORY_DEFAULT_AP_PASSWORD, sizeof(prm.ApPsw) - 1);
  prm.ApPsw[sizeof(prm.ApPsw) - 1] = '\0';
  strncpy(prm.NtpServer,"pool.ntp.org",sizeof(prm.NtpServer));
  strcpy(prm.mqttBrokerAddr,"10.10.0.202"); 
  strcpy(prm.mqttBrokerUser,"mqtt");
  strcpy(prm.mqttBrokerPsw,"mqtt");
  strncpy(prm.firmwareServer,FIRMWARE_SERVER,sizeof(prm.firmwareServer));
  prm.mqttEnable = false;
  prm.mqttBrokerRefresh = 10; //sec
  prm.utc_offset = 1;
  prm.enableDST = true;          // Flag to enable DST (summer time...)
  prm.set12_24 = true;           // Flag indicating 12 vs 24 hour time (false = 12, true = 24);
  prm.showZero = true;           // Flag to indicate whether to show zero in the hour ten's place
  prm.enableBlink = true;        // Flag to indicate whether center colon should blink
  prm.interval = 15;             // prm.interval in minutes, with 0 = off
  prm.enableAutoShutoff = true;  // Flag to enable/disable nighttime shut off
  prm.dayHour = 7;
  prm.dayMin = 0;
  prm.nightHour = 20;
  prm.nightMin = 0;
  prm.dayBright = (byte)(((int)MAXBRIGHTNESS * 30 + 50) / 100);
  if (prm.dayBright < 1) prm.dayBright = 1;
  prm.nightBright = 3;
  prm.animMode = 7;  
  prm.dateMode = 0;               // 0:dd/mm/yyyy 1:mm/dd/yyyy 2:yyyy/mm/dd
  prm.tempCF = false;               //Temperature Celsius / Fahrenheit
  prm.enableTimeDisplay = ENABLE_CLOCK_DISPLAY;
  prm.dateStart = 35;                          //Date display window: 35..40 sec
  prm.dateEnd = 40;
  prm.tempStart = 40;                          //Temperature display window: 40..45 sec
  prm.tempEnd = 45;
  prm.humidStart = 45;                         //Humidity display window: 45..50 sec
  prm.humidEnd = 50;
  prm.dateRepeatMin = DATE_REPEAT_MIN;         //show date only every xxx minute. If zero, datum is never displayed!  
  prm.tempRepeatMin = 1;
  prm.enableDoubleBlink = true;              //both separator points are blinking (6 or 8 tubes VFD clock)
  prm.enableAutoDim = true;          //Automatic dimming by luxmeter
  prm.enableRadar = false;            //Radar sensor
  prm.radarTimeout = 5;             //min
  prm.tubesSleep = false;            // Motion-based tubes sleep
  prm.tubesWakeSeconds = 10;         // Wake duration (sec)
  prm.manualDisplayOff = false;      // Manual display OFF
  prm.wakeOnMotionEnabled = true;  // Wake on motion enabled by default
  prm.touchShortAction = TOUCH_ACTION_ALARM_OFF;
  prm.touchDoubleAction = TOUCH_ACTION_COLOR_CHANGE;
  prm.touchLongAction = TOUCH_ACTION_DISPLAY_TOGGLE;
  prm.corrT0 = 0;
  prm.corrT1 = 0;
  prm.corrH0 = 0;
  prm.corrH1 = 0;  
  prm.manualTimeValid = false;
  prm.manualEpoch = 0;
  prm.magic = MAGIC_VALUE;              //magic value to check the EEPROM version
  settingsSetDefaults();
  // Factory reset must persist immediately before reboot
  settingsDirty = true;
  saveEEPROM();
  eepromSavePending = false;
  calcTime();
}


void newCathodeProtect(unsigned long t,int dir) {    //t = time in msec, dir = direction -1,0,1    (0=random) 
  byte tmp[10];
  boolean tmpDP[10];
  unsigned long started = millis();
  boolean finish, stopThis;
  byte fin[10];
  byte nextStoppedDigit;
  int sum = 0;
  
  cathodeProtRunning = true;
  stopCathodeProtect = false;
  DPRINT("Cathode Protect running! dir:"); DPRINTLN(dir); 
  memcpy(tmp,digit,sizeof(tmp));
  memcpy(tmpDP,digitDP,sizeof(tmpDP));
  memset(fin,0,sizeof(fin));
  switch (dir) {
    case -1:
      nextStoppedDigit = 0;
      break;
    case 0:
      nextStoppedDigit = random(maxDigits);
      break;
    case 1:
      nextStoppedDigit = maxDigits-1;
      break;            
  }
  nextStoppedDigit = dir>0 ? 0 : maxDigits-1;
  while(true) {
    for (int i=0;i<maxDigits;i++) {
      finish = ((millis()-started) >= t);
      stopThis = (i == nextStoppedDigit); 
        
      if (finish && stopThis) {  //this digit stops
        fin[i] = 1;
        digitDP[i] = tmpDP[i];   //restore original DP value
        digit[i] = tmp[i];
        t +=1000;
        if (dir>0) {  //find next stop digit
          nextStoppedDigit++;
          if (nextStoppedDigit>=maxDigits) nextStoppedDigit =0;
        }
        else if (dir<0) {
          nextStoppedDigit--;
          if (nextStoppedDigit==255) nextStoppedDigit = maxDigits-1;
        }
        else if (sum<=maxDigits-2) {
          while(true) {
          nextStoppedDigit = random(maxDigits);
          if (fin[nextStoppedDigit] ==0) break;  //found a running digit
          }
        }
      }
      else if (fin[i]==0) {  //get next number
        if (dir>=0) 
          digit[i] = (digit[i]+1) % 10;
        else {
          digit[i]--; if (digit[i] == 255) digit[i] = 9;
        } 
        digitDP[i] = ((digit[i]%2) == 0);
      }
  } //end for
   //DPRINT("lastStoppedDigit:"); DPRINT(lastStoppedDigit);
   //DPRINT("  nextStoppedDigit:"); DPRINTLN(nextStoppedDigit);
  
    writeDisplaySingleGuarded();
    Fdelay(100);
    sum = 0;
    for (int i=0;i<maxDigits;i++)  sum += fin[i];
    //DPRINT("Sum:"); DPRINTLN(sum);
    if (sum >= maxDigits)  //all digits ready
      break;  
    if (stopCathodeProtect) {
      DPRINTLN("Cathode Protect manually stopped.");
      break;
    }
    if ((millis()-started) >= (t+5000)) {
      DPRINTLN("Safety exit");
      break;
    }
  } //end while
   
  memcpy(oldDigit, digit, sizeof(oldDigit));
  lastCathodeProt = minute();
  cathodeProtRunning = false;
  stopCathodeProtect = false;
}

void cathodeProtect() {   //original version
  int hour12_24 = prm.set12_24 ? (byte)hour() : (byte)hourFormat12();
  byte hourShow = (byte)hour12_24;
  byte minShow  = (byte)minute();
  byte secShow  = (byte)second();
  byte dh1 = (hourShow / 10), dh2 = (hourShow % 10);
  byte dm1 = (minShow / 10),  dm2 = (minShow % 10);
  byte ds1 = (secShow / 10),  ds2 = (secShow % 10);

  DPRINTLN("Cathode Protect running!");
  // All four digits will increment up at 10 Hz.
  // At T=2 sec, individual digits will stop at
  // the correct time every second starting from
  // the right and ending on the left
  for (int i = 0; i <= 70; i++) {
    if (i >= 20) ds2 = (secShow % 10);
    if (i >= 30) ds1 = (secShow / 10);
    if (i >= 40) dm2 = (minShow % 10);
    if (i >= 50) dm1 = (minShow / 10);
    if (i >= 60) dh2 = (hourShow % 10);
    if (i == 70) dh1 = (hourShow / 10);
    if (maxDigits >= 8) {
      digit[7] = dh1;
      digit[6] = dh2;
      digit[5] = 11;
      digit[4] = dm1;
      digit[3] = dm2;
      digit[2] = 11;
      digit[1] = ds1;
      digit[0] = ds2;
    }
    else if (maxDigits == 6) {
      digit[5] = dh1;
      digit[4] = dh2;
      digit[3] = dm1;
      digit[2] = dm2;
      digit[1] = ds1;
      digit[0] = ds2;
    }
    else {
      digit[3] = dh1;
      digit[2] = dh2;
      digit[1] = dm1;
      digit[0] = dm2;
    }
    incMod10(dh1); incMod10(dh2);
    incMod10(dm1); incMod10(dm2);
    incMod10(ds1); incMod10(ds2);
  for (int d=0;d<maxDigits;d++) {
    digitDP[d] = ((digit[d]%2) == 0);
  }
    writeDisplaySingleGuarded();
    Fdelay(100);
  }
  memcpy(oldDigit, digit, sizeof(oldDigit));
  lastCathodeProt = minute();
}

inline void incMod10(byte &x) {
  x = (x + 1 == 10 ? 0 : x + 1);
};


#ifdef DISPLAYTEMP_ONLY_2DIGITS
void displayTemp(byte ptr) {   //special version, 2 digits only
  float t = temperature[ptr];
  if (ptr==0) t += prm.corrT0;
  if (ptr==1) t += prm.corrT1;
  if (prm.tempCF) {
    t = round1((temperature[ptr] * 9/5)+32);
  }
  int digPtr = 3;   //tube number: first digit of minutes (hours:5,4  minutes:3,2   seconds: 1,0 )
  #ifdef TEMP_START_POSITION
    digPtr = TEMP_START_POSITION;  
  #endif  
  for (int i = 0; i < maxDigits; i++) {   //clear display
    digitDP[i] = false;
    newDigit[i] = 10;
  }

  t = abs(t);
  newDigit[digPtr] = t / 10;
  if (newDigit[digPtr] == 0) newDigit[digPtr] = 10; //BLANK, if zero
  newDigit[--digPtr] = int(t) % 10;

  if (prm.animMode == 0)  memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
  colonBlinkState = false;
  decimalpointON = false;
}

#else
void displayTemp(byte ptr) {
  float t = temperature[ptr];
  if (ptr==0) t += prm.corrT0;
  if (ptr==1) t += prm.corrT1;
  if (prm.tempCF) {
    t = round1((temperature[ptr] * 9/5)+32);
  }
  int digPtr = maxDigits-1;
  #ifdef TEMP_START_POSITION
    digPtr = TEMP_START_POSITION;  
  #endif
  for (int i = 0; i < maxDigits; i++) {
    digitDP[i] = false;
    newDigit[i] = 10;
  }
  #if defined(PLUS_CHARCODE) || defined(MINUS_CHARCODE)   //leading sign 
    if (t>=0) newDigit[digPtr] = PLUS_CHARCODE;
    if (t<0) newDigit[digPtr] = MINUS_CHARCODE;
    digPtr--;
  #endif
  t = abs(t);
  newDigit[digPtr] = t / 10;
  if (newDigit[digPtr] == 0) newDigit[digPtr] = 10; //BLANK!!!
  newDigit[--digPtr] = int(t) % 10;
  digitDP[digPtr] = true;
  newDigit[--digPtr] = int(t * 10) % 10;
  if ((maxDigits > 5) && (GRAD_CHARCODE >= 0)) {
    digPtr -=1;
    newDigit[digPtr] = GRAD_CHARCODE; //grad
  }
  if (TEMP_CHARCODE >= 0) {
    if (prm.tempCF) newDigit[--digPtr] = TEMP_CHARCODE+5; //  'F'
    else            newDigit[--digPtr] = TEMP_CHARCODE;   //  'C'
  }

  if (prm.animMode == 0)  memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
  colonBlinkState = true;
  decimalpointON = true;
}
#endif

#ifdef DISPLAYHUMID_ONLY_2DIGITS
void displayHumid(byte ptr) {   //special version, 2 digits only
  float h = humid[ptr];
  if (ptr==0) h = round(h + prm.corrH0);
  if (ptr==1) h = round(h + prm.corrH1);

  int digPtr = 3;   //tube number: first digit of minutes (hours:5,4  minutes:3,2   seconds: 1,0 )
  #ifdef HUMID_START_POSITION
    digPtr = HUMID_START_POSITION;  
  #endif  
  for (int i = 0; i < maxDigits; i++) {   //clear display
    digitDP[i] = false;
    newDigit[i] = 10;
  }

  newDigit[digPtr] = h / 10;
  if (newDigit[digPtr] == 0) newDigit[digPtr] = 10; //BLANK, if zero
  newDigit[--digPtr] = int(h) % 10;

  if (prm.animMode == 0)  memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
  colonBlinkState = false;
  decimalpointON = false;
}

#else
void displayHumid(byte ptr) {
  int digPtr = maxDigits-1;
  #ifdef HUMID_START_POSITION
    digPtr = HUMID_START_POSITION;  
  #endif
  for (int i = 0; i < maxDigits; i++) {
    digitDP[i] = false;
    newDigit[i] = 10;
  }
  float h = humid[ptr];
  if (ptr==0) h += prm.corrH0;
  if (ptr==1) h += prm.corrH1;
  #if defined(PLUS_CHARCODE) || defined(MINUS_CHARCODE)   //leading sign 
    digPtr--;
  #endif
  newDigit[digPtr] = int(h) / 10;
  if (newDigit[digPtr] == 0) newDigit[digPtr] = 10; //BLANK if zero!!!
  newDigit[--digPtr] = int(h) % 10;
  digitDP[digPtr] = true;
  newDigit[--digPtr] = int(h * 10) % 10;
  if (maxDigits > 5) {
    if (digitsOnly) {
      newDigit[--digPtr] = 10;   //empty character
      newDigit[--digPtr] = PERCENT_CHARCODE;  //  "%"
    }
    else {
      newDigit[--digPtr] = 10;   //empty character
      newDigit[--digPtr] = UPPER_CIRCLE_CHARCODE;   //upper circle = 16
      newDigit[--digPtr] = LOWER_CIRCLE_CHARCODE;  // lower circle = 18
    }
  } //4 tubes only
  else {
    newDigit[--digPtr] = PERCENT_CHARCODE;  //  "%"
  }
     
  if (prm.animMode == 0)  memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
  colonBlinkState = true;
  decimalpointON = true;
}

void displayPressure(byte ptr) {
  int digPtr = maxDigits - 1;

  for (int i = 0; i < maxDigits; i++) {
    digitDP[i] = false;
    newDigit[i] = 10;
  }

  if (maxDigits >= 6) {
    long pScaled = lround(pressur[ptr] * 100.0f); // show 2 decimals on 6/8-digit clocks
    if (pScaled < 0) pScaled = 0;

    int frac = (int)(pScaled % 100L);
    long pInt = pScaled / 100L;
    int intOnesIdx = 2;

    // Fractional part on the right-most pair (seconds position)
    newDigit[0] = frac % 10;
    newDigit[1] = (frac / 10) % 10;

    // Integer part starts at index 2 and grows leftward visually (towards higher indexes)
    int intIdx = 2;
    if (pInt == 0) {
      newDigit[intIdx] = 0;
    } else {
      while ((intIdx < maxDigits) && (pInt > 0)) {
        newDigit[intIdx++] = pInt % 10;
        pInt /= 10;
      }
    }

    // Decimal point between integer and fractional parts
    digitDP[intOnesIdx] = true;
  }
  else {
    int pInt = int(round(pressur[ptr]));
    if (pInt < 0) pInt = 0;

    if (pInt == 0) {
      newDigit[digPtr] = 0;
    } else {
      while ((digPtr >= 0) && (pInt > 0)) {
        newDigit[digPtr--] = pInt % 10;
        pInt /= 10;
      }
    }
  }

  if (prm.animMode == 0)  memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
  colonBlinkState = true;
  decimalpointON = true;
}
#endif

void displayDate()  {
  byte m = prm.dateMode;
  if (m>2) m=2;
  byte p[3][8] = 
  {     //tube# 76543210    543210    3210
    {2,1,0},  //ddmmyyyy    ddmmyy    ddmm
    {2,0,1},  //mmddyyyy    mmddyy    mmdd
    {0,1,2}   //yyyymmdd    yymmdd    mmdd
  };  
  
  int t = 0;
  if (thermometerClock)  t = 1;
  for (int i=0;i<3;i++) {
    if (p[m][i] == 2) {
      if (!thermometerClock) {
        if (maxDigits>4) {
          newDigit[t++] = year() % 10;
          newDigit[t++] = (year() % 100) / 10;
        }
        if (maxDigits>6) {
          newDigit[t++] = (year() % 1000) / 100;
          newDigit[t++] = year() / 1000;
        }      
      }  //!thermometerClock
    }  
    if (p[m][i] == 1) {
      newDigit[t++] = month() % 10;
      newDigit[t++] = month() / 10;
    }
    if (p[m][i] == 0) {
      newDigit[t++] = day() % 10;
      newDigit[t++] = day() / 10;
    }
  } //end for
  if ((maxDigits>6) && (m<2)) {
    digitDP[4] = true;
    digitDP[6] = true;
  }
  else {
    if (thermometerClock) {
      digitDP[3] = true;
      newDigit[0] = 10;
      newDigit[5] = 10;
    }
    else {
      digitDP[2] = true;
      digitDP[4] = true;
    }
  }
  colonBlinkState = true;
  if (prm.animMode == 1)  memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
}

void displayTime4() {
  for (int i = 0; i < maxDigits; i++) digitDP[i] = false;
  digitDP[4] = true;   digitDP[2] = true;
  int hour12_24 = prm.set12_24 ? (byte)hour() : (byte)hourFormat12();
  if      (showDate && prm.enableTimeDisplay) displayDate();
  else if (showTemp1) displayTemp(1);
  else if (showTemp0) displayTemp(0);
  else if (showHumid1) displayHumid(1);
  else if (showHumid0) displayHumid(0);
  else if (showPress0) displayPressure(0);
  else if (prm.enableTimeDisplay) {
    showClock = true;
    newDigit[3] = hour12_24 / 10;
    if ((!prm.showZero)  && (newDigit[3] == 0)) newDigit[3] = 10;
    newDigit[2] = hour12_24 % 10;
    newDigit[1] = minute() / 10;
    newDigit[0] = minute() % 10;
    if (prm.enableBlink && (second() % 2 == 0)) digitDP[2] = false;
  }
}


void displayTime6() {
  for (int i = 0; i < maxDigits; i++) digitDP[i] = false;
  digitDP[4] = true;   digitDP[2] = true;
  int hour12_24 = prm.set12_24 ? (byte)hour() : (byte)hourFormat12();
  if      (showDate && prm.enableTimeDisplay)  displayDate();
  else if (showTemp1) displayTemp(1);
  else if (showTemp0) displayTemp(0);
  else if (showHumid1) displayHumid(1);
  else if (showHumid0) displayHumid(0);
  else if (showPress0) displayPressure(0);
  else if (prm.enableTimeDisplay) {
    showClock = true;
    if (thermometerClock) {
      newDigit[4] = hour12_24 / 10;
      if ((!prm.showZero)  && (newDigit[4] == 0)) newDigit[4] = 10;
      newDigit[3] = hour12_24 % 10;
      newDigit[2] = minute() / 10;
      newDigit[1] = minute() % 10;
      newDigit[0] = 10;
      newDigit[5] = 10;
      if (prm.enableBlink && (second() % 2 == 0)) digitDP[3] = false;
    }
    else {  //normal clock
      newDigit[5] = hour12_24 / 10;
      if ((!prm.showZero)  && (newDigit[5] == 0)) newDigit[5] = 10;
      newDigit[4] = hour12_24 % 10;
      newDigit[3] = minute() / 10;
      newDigit[2] = minute() % 10;
      if (prm.enableBlink && (second() % 2 == 0)){
        digitDP[2] = false;
        if (prm.enableDoubleBlink) {
          digitDP[4] = false;
        }
      }
      newDigit[1] = second() / 10;
      newDigit[0] = second() % 10;
    }
  }
}

void displayTime8() {
  for (int i = 0; i < maxDigits; i++) {
    digitDP[i] = false;
    newDigit[i] = 10;
  }
  int hour12_24 = prm.set12_24 ? (byte)hour() : (byte)hourFormat12();
  if      (showDate && prm.enableTimeDisplay) displayDate();
  else if (showTemp1) displayTemp(1);
  else if (showTemp0) displayTemp(0);
  else if (showHumid1) displayHumid(1);
  else if (showHumid0) displayHumid(0);
  else if (showPress0) displayPressure(0);
  else if (prm.enableTimeDisplay) {
      showClock = true;
      newDigit[8] = 10;  //sign digit = BLANK
      newDigit[7] = hour12_24 / 10;
      if ((!prm.showZero)  && (newDigit[7] == 0)) newDigit[7] = 10;
      newDigit[6] = hour12_24 % 10;
      newDigit[5] = 11;  //- sign
      newDigit[4] = minute() / 10;
      newDigit[3] = minute() % 10;
      newDigit[2] = 11;  //- sign
      if (prm.enableBlink && (second() % 2 == 0)) {
        newDigit[2] = 10; //BLANK
        if (prm.enableDoubleBlink) {
          newDigit[5] = 10; //BLANK
        } 
      }
      newDigit[1] = second() / 10;
      newDigit[0] = second() % 10;
  }
}


void changeDigit() {
  int j = 0;
  byte tmp[50];
  byte space = 4;
  byte anim;
  
  #if defined(TUBE1CLOCK)   //no tube animation possible
    writeDisplaySingleGuarded();
    return;
  #endif  
  anim = prm.animMode;
  #if defined(DISABLE_NIGHT_ANIMATION)
    if (!displayON) anim = 0;   //At NIGHT switch off animation
  #endif
  #if defined(WORDCLOCK)
    anim = 0;   
  #endif
  
  if (anim == 6) {
    static const byte randomModes6[] = {1, 2, 3, 4, 5, 8, 9};
    anim = randomModes6[rand() % (sizeof(randomModes6) / sizeof(randomModes6[0]))];
  }
  else if (anim == 7) {
    static const byte randomModes7[] = {1, 2, 3, 4, 8, 9};
    anim = randomModes7[rand() % (sizeof(randomModes7) / sizeof(randomModes7[0]))];
  }

  if (anim != 5) {
    for (int i = 0; i < maxDigits; i++)
      if ((newDigit[i] > 9) || ((oldDigit[i] > 9) && (newDigit[i] <= 9) )) {
        digit[i] = newDigit[i];    //show special characters ASAP or if special char changes to numbers
        oldDigit[i] = newDigit[i];
      }
    if ((maxDigits > 4) && (newDigit[0] != 0)) j = 1; //if 6 or 8 tube clock, dont play with seconds
  }

  if (memcmp(newDigit + j, oldDigit + j, maxDigits - j) != 0) {
    switch (anim) {
      case 1:
        for (int tube = j; tube < maxDigits; tube++) {
          if ((newDigit[tube] != oldDigit[tube]) && (newDigit[tube] <= 9)) {
            for (int i = oldDigit[tube]; i <= int(newDigit[tube] + 10); i++) {
              digit[tube] = i % 10;
              writeDisplaySingleGuarded();
              Fdelay(ANIMSPEED);
            } //end for i
            writeDisplaySingleGuarded();
          } //endif
        }  //end for tube
        break;
      case 2:
        for (int i = 0; i <= 9; i++) {
          for (int tube = j; tube < maxDigits; tube++) {
            if ((newDigit[tube] != oldDigit[tube]) && (newDigit[tube] <= 9))
              digit[tube] = (oldDigit[tube] + i) % 10;
          } //end for tube
          writeDisplaySingleGuarded();
          Fdelay(ANIMSPEED);
        }  //end for i
        break;
      case 3:
        memcpy(tmp, oldDigit, maxDigits);
        memcpy(tmp + maxDigits + space, newDigit, maxDigits);
        memset(tmp + maxDigits, 11, space); //----
        for (int i = 0; i < maxDigits + space; i++) {
          for (int tube = 0; tube < maxDigits; tube++) {
            digit[tube % maxDigits] = tmp[i + tube];
          } //end for tube
          writeDisplaySingleGuarded();
          Fdelay(ANIMSPEED);
        } //end for i
        break;
      case 4:
        memcpy(tmp, newDigit, maxDigits);
        memcpy(tmp + maxDigits + space, oldDigit, maxDigits);
        memset(tmp + maxDigits, 11, space); //----
        for (int i = maxDigits - 1 + space; i >= 0; i--) {
          for (int tube = 0; tube < maxDigits; tube++) {
            digit[tube % maxDigits] = tmp[i + tube];
          } //end for tube
          writeDisplaySingleGuarded();
          Fdelay(ANIMSPEED);
        } //end for i
        break;
      case 5:
        memset(animMask, 0, sizeof(animMask));
#if defined(MULTIPLEX74141) || defined(NO_MULTIPLEX74141) || defined(MULTIPLEX74141_ESP32) || defined(MULE_V2) || defined(PCF_74141) || defined(NO_MULTIPLEX_ESP32) || defined(VQC10) || defined(NEWHV5122)
        for (int i = 1; i < 20; i++) {
          for (int tube = j; tube < maxDigits; tube++) {
            if (oldDigit[tube] != newDigit[tube]) animMask[tube] = i;  //digit is changed
          }  //end for tube
          writeDisplaySingleGuarded();
          Fdelay(ANIMSPEED);
        }  //end for i
#else
        for (int i = 1; i <= 5; i++) {
          for (int tube = j; tube < maxDigits; tube++) {
            if (oldDigit[tube] != newDigit[tube]) animMask[tube] = i; //digit is changed
          }  //end for tube
          writeDisplaySingleGuarded();
          Fdelay(ANIMSPEED);
        }  //end for i
        memcpy(digit, newDigit, sizeof(digit));
        for (int i = 1; i <= 5; i++) {
          for (int tube = j; tube < maxDigits; tube++) {
            if (oldDigit[tube] != newDigit[tube]) animMask[tube] = 6 - i; //digit is changed
          }  //end for tube
          writeDisplaySingleGuarded();
          Fdelay(ANIMSPEED);
        }  //end for i
#endif
        memset(animMask, 0, sizeof(animMask));
        break;
      case 8: {
        const int maxTubes = 20;
        int changed[maxTubes];
        int changedCount = 0;

        for (int tube = j; tube < maxDigits && changedCount < maxTubes; tube++) {
          if ((newDigit[tube] != oldDigit[tube]) && (newDigit[tube] <= 9)) {
            changed[changedCount++] = tube;
          }
        }

        if (changedCount > 0) {
          int order[maxTubes];
          for (int i = 0; i < changedCount; i++) order[i] = changed[i];

          // Center-out order: minute area first, then wave to seconds and hours.
          float center = (float)(j + maxDigits - 1) * 0.5f;
          for (int i = 0; i < changedCount - 1; i++) {
            int best = i;
            float bestDist = fabsf((float)order[i] - center);
            for (int k = i + 1; k < changedCount; k++) {
              float d = fabsf((float)order[k] - center);
              if (d < bestDist) {
                best = k;
                bestDist = d;
              }
            }
            if (best != i) {
              int tmpIdx = order[i];
              order[i] = order[best];
              order[best] = tmpIdx;
            }
          }

          uint8_t spinValue[maxTubes];
          for (int i = 0; i < changedCount; i++) {
            int t = changed[i];
            int startDigit = oldDigit[t];
            if (startDigit > 9) startDigit = random(0, 10);
            spinValue[i] = (uint8_t)startDigit;
            digit[t] = spinValue[i];
          }

          // Dramatic slot effect: progressively lock center -> outer tubes.
          for (int locked = 0; locked < changedCount; locked++) {
            int spinFrames = 10 + (changedCount - locked) * 3;
            for (int frame = 0; frame < spinFrames; frame++) {
              for (int i = 0; i < changedCount; i++) {
                int t = changed[i];
                bool isLocked = false;
                for (int m = 0; m < locked; m++) {
                  if (order[m] == t) {
                    isLocked = true;
                    break;
                  }
                }

                if (isLocked) {
                  digit[t] = newDigit[t];
                } else {
                  uint8_t step = (random(0, 100) < 90) ? 1 : 2;
                  spinValue[i] = (uint8_t)((spinValue[i] + step) % 10);
                  digit[t] = spinValue[i];
                }
              }
              writeDisplaySingleGuarded();
              Fdelay(max(18, (ANIMSPEED * 3) / 4));
            }

            digit[order[locked]] = newDigit[order[locked]];
            writeDisplaySingleGuarded();
            Fdelay(max(22, ANIMSPEED));
          }
        }
        break;
      }
      case 9: {
        const int maxTubes = 20;
        int start = j;
        int end = maxDigits - 1;
        int span = end - start + 1;

        if (span > 0 && span <= maxTubes) {
          int idxLeft[maxTubes];
          int idxRight[maxTubes];
          int leftCount = 0;
          int rightCount = 0;

          if ((span % 2) == 0) {
            int centerLeft = start + (span / 2) - 1;
            int centerRight = centerLeft + 1;
            for (int t = centerLeft; t >= start; t--) idxLeft[leftCount++] = t;
            for (int t = centerRight; t <= end; t++) idxRight[rightCount++] = t;
          } else {
            int center = start + (span / 2);
            for (int t = center; t >= start; t--) idxLeft[leftCount++] = t;
            for (int t = center + 1; t <= end; t++) idxRight[rightCount++] = t;
          }

          byte tmpLeft[50];
          byte tmpRight[50];

          if (leftCount > 0) {
            for (int i = 0; i < leftCount; i++) tmpLeft[i] = oldDigit[idxLeft[i]];
            memset(tmpLeft + leftCount, 11, space);
            for (int i = 0; i < leftCount; i++) tmpLeft[leftCount + space + i] = newDigit[idxLeft[i]];
          }
          if (rightCount > 0) {
            for (int i = 0; i < rightCount; i++) tmpRight[i] = oldDigit[idxRight[i]];
            memset(tmpRight + rightCount, 11, space);
            for (int i = 0; i < rightCount; i++) tmpRight[rightCount + space + i] = newDigit[idxRight[i]];
          }

          int frames = max(leftCount, rightCount) + space;
          for (int i = 0; i < frames; i++) {
            if (i < leftCount + space) {
              for (int p = 0; p < leftCount; p++) {
                digit[idxLeft[p]] = tmpLeft[i + p];
              }
            }
            if (i < rightCount + space) {
              for (int p = 0; p < rightCount; p++) {
                digit[idxRight[p]] = tmpRight[i + p];
              }
            }
            writeDisplaySingleGuarded();
            Fdelay(ANIMSPEED);
          }
        }
        break;
      }
    }
  } //endif memcmp

  memcpy(digit, newDigit, sizeof(digit));
  memcpy(oldDigit, newDigit, sizeof(oldDigit));
  writeDisplaySingleGuarded();
}

void writeAlarmPin(boolean newState) {
  #if ALARMSPEAKER_PIN>=0
    static boolean oldState = !ALARM_ON; 
    if (oldState != newState) {
      oldState = newState;
      //if (newState == ALARM_ON) DPRINTLN("Alarm ON"); else DPRINTLN("Alarm OFF");
      digitalWrite(ALARMSPEAKER_PIN, newState);
    }
  #endif
}

void alarmSound(void) {
  static const unsigned int t[] = {0, 3000, 6000, 6200, 9000, 9200, 9400, 12000, 12200, 12400, 15000, 15200, 15400};
  static int count = 0;
  static unsigned long nextEvent;
  const int cMax = sizeof(t) / sizeof(t[0]);  //number of time steps

  if (prm.alarmEnable) {
    if ( (!alarmON && prm.alarmHour == hour()) && (prm.alarmMin == minute()) && (second() <= 3)) {  //switch ON alarm sound
      DPRINTLN("Alarm started!");
      alarmON = true;
      alarmStarted = millis();
      count = 0;
      nextEvent = alarmStarted - 1;
    }
  }
  else {
    alarmON = false;
  }

  if (!alarmON) {
    writeAlarmPin(!ALARM_ON);
    return;  //nothing to do
  }

  if ((millis() - alarmStarted) > 1000 * (long)prm.alarmPeriod) {
    alarmON = false;   //alarm period is over
    darkenNeopixels();
    DPRINTLN("Alarm ended.");
  }

  if (ALARMBUTTON_PIN >= 0) { //is button installed?
    if (digitalRead(ALARMBUTTON_PIN) == LOW) { //stop alarm
      DPRINTLN("Alarm stopped by button press.");
      alarmON = false;
    }
  }

  if (!prm.alarmEnable || !alarmON) {  //no alarm, switch off
    writeAlarmPin(!ALARM_ON);
    return;
  }

  //-------- Generate sound --------------------------------

  if (millis() > nextEvent) { //go to the next event
    if (count % 2 == 0) {
      nextEvent += 500;
      writeAlarmPin(ALARM_ON);
      //DPRINT(" Sound ON");  DPRINT("  Next:"); DPRINTLN(nextEvent);
    }
    else {
      writeAlarmPin(!ALARM_ON);
      nextEvent = (count / 2 < cMax) ? alarmStarted +  t[count / 2] : nextEvent + 500;
      //DPRINT("   OFF"); DPRINT("  Next:"); DPRINTLN(nextEvent);
    }
    count++;
  }
}


void evalShutoffTime(void) {  // Determine whether  tubes should be turned to NIGHT mode

  if (!prm.enableAutoShutoff) return;

  int mn = 60 * hour() + minute();
  int mn_on = prm.dayHour * 60 + prm.dayMin;
  int mn_off = prm.nightHour * 60 + prm.nightMin;

  static bool prevShutoffState = true;
  if ( (((mn_off < mn_on) &&  (mn >= mn_off) && (mn < mn_on))) ||  // Nighttime
       ((mn_off > mn_on) && ((mn >= mn_off) || (mn < mn_on)))) {
    if (!manualOverride) displayON = false;
    if (prevShutoffState == true) manualOverride = false;
    prevShutoffState = false;
  }
  else {  // Tubes should be on
    if (!manualOverride) displayON = true;
    if (prevShutoffState == false) manualOverride = false;
    prevShutoffState = true;
  }
  return;
  DPRINT("mn="); DPRINT(mn);
  DPRINT("  mn_on="); DPRINT(mn_on);
  DPRINT("  mn_off="); DPRINT(mn_off);
  DPRINT("  manOverride:"); DPRINT(manualOverride);
  DPRINT("  displayON:"); DPRINTLN(displayON);
  DPRINT("  enableBlink:"); DPRINTLN(prm.enableBlink);
  DPRINT("  blinkState:"); DPRINTLN(colonBlinkState);
}


void writeIpTag(byte iptag) {

#ifdef SHIFT_TUBES_LEFT_BY_1
  byte offset = 1;
#else
  byte offset = 0;
#endif

  memset(newDigit, 10, sizeof(newDigit));
  if (!digitsOnly && (maxDigits >= 6)) {
    newDigit[5] = 19;  //'I'
    newDigit[4] = 14;  //'P'
  }
  //if (iptag >= 100) 
  newDigit[2 + offset] = iptag / 100;
  newDigit[1 + offset] = (iptag % 100) / 10;
  newDigit[0 + offset] = iptag % 10;
  memcpy(digit,newDigit,sizeof(digit));
  printDigits(0);
  changeDigit();
}

void showMyIp(void) {   //at startup, show the web page internet address
  clearDigits();
  #define SPEED 1500
  for (int i=0;i<4;i++) {
    ipShowRunning = true;
    writeIpTag(ip[i]);
    Fdelay(SPEED);
  }
  ipShowRunning = false;
}


void fifteenMinToHM(int &hours, int &minutes, int fifteenMin)
{
  hours = fifteenMin / 4;
  minutes = (fifteenMin % 4) * 15;
}

void resetWiFi(void) {
  static unsigned long lostSinceMs = 0;
  static unsigned long lastAttemptMs = 0;
  static byte failedReconnectAttempts = 0;

  // Never run STA recovery while AP is active (AP or AP+STA).
  // This avoids disrupting web setup clients during scan/connect workflows.
  WiFiMode_t wm = WiFi.getMode();
  if ((wm == WIFI_AP) || (wm == WIFI_AP_STA)) {
    return;
  }

  const unsigned long nowMs = millis();
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiRecoveryApMode) {
      DPRINTLN("WiFi recovered. Leaving AP recovery mode.");
      wifiRecoveryApMode = false;
    }
    lostSinceMs = 0;
    failedReconnectAttempts = 0;
    return;
  }

  if (wifiRecoveryApMode) {
    return;
  }

  if (lostSinceMs == 0) {
    lostSinceMs = nowMs;
    DPRINTLN("WiFi lost: starting recovery.");
  }

  // Throttle attempts so loop timing remains stable.
  if ((nowMs - lastAttemptMs) < 30000UL) return;  // every 30 sec
  lastAttemptMs = nowMs;

  DPRINTLN("Lost WiFi. Reconnect attempt.");
  WiFi.disconnect();
  delay(50);
  WiFi.reconnect();
  delay(50);
  if (WiFi.status() == WL_CONNECTED) {
    DPRINTLN("WiFi reconnected.");
    lostSinceMs = 0;
    failedReconnectAttempts = 0;
    return;
  }

  failedReconnectAttempts++;
  if (failedReconnectAttempts >= 2) {
    DPRINTLN("WiFi reconnect failed twice. Switching to standalone AP mode for web access.");
    startStandaloneMode();
    wifiRecoveryApMode = true;
    lostSinceMs = 0;
    failedReconnectAttempts = 0;
  }
}

void processPendingWifiSwitch() {
  if (!wifiSwitchPending) return;

  const unsigned long nowMs = millis();
  const int st = WiFi.status();

  if (st == WL_CONNECTED) {
    String current = WiFi.SSID();

    if (current == wifiSwitchTargetSsid) {
      if (wifiSwitchRollbackRunning) {
        wifiSwitchLastResult = "rollback_connected";
      } else {
        wifiSwitchLastResult = "connected";
      }
      if (!wifiSwitchRollbackRunning && wifiSwitchSaveOnSuccess) {
        wifiSwitchTargetSsid.toCharArray(prm.wifiSsid, sizeof(prm.wifiSsid));
        wifiSwitchTargetPsw.toCharArray(prm.wifiPsw, sizeof(prm.wifiPsw));
        settingsDirty = true;
        requestSaveEEPROM();
        DPRINTLN("WiFi switch successful, credentials saved.");
      }
      wifiSwitchPending = false;
      wifiSwitchRollbackRunning = false;
      wifiRecoveryApMode = false;
      return;
    }

    // Connected, but not to target. Keep waiting until timeout, then rollback if needed.
    if ((nowMs - wifiSwitchStartMs) > WIFI_SWITCH_TIMEOUT_MS) {
      if (!wifiSwitchRollbackRunning && wifiSwitchCanRollback && wifiSwitchPrevSsid.length() > 0) {
        DPRINTLN("WiFi switch timed out on wrong SSID, rolling back.");
        wifiSwitchLastResult = "rollback_started";
        wifiSwitchRollbackRunning = true;
        wifiSwitchCanRollback = false;
        wifiSwitchSaveOnSuccess = false;
        wifiSwitchTargetSsid = wifiSwitchPrevSsid;
        wifiSwitchTargetPsw = wifiSwitchPrevPsw;
        wifiSwitchStartMs = nowMs;
        setWifiModeForSwitchKeepAp();
        delay(30);
        if (wifiSwitchTargetPsw.length() > 0) WiFi.begin(wifiSwitchTargetSsid.c_str(), wifiSwitchTargetPsw.c_str());
        else WiFi.begin(wifiSwitchTargetSsid.c_str());
      } else {
        wifiSwitchLastResult = "connect_failed";
        wifiSwitchPending = false;
        wifiSwitchRollbackRunning = false;
        startStandaloneMode();
      }
    }
    return;
  }

  if ((nowMs - wifiSwitchStartMs) > WIFI_SWITCH_TIMEOUT_MS) {
    if (!wifiSwitchRollbackRunning && wifiSwitchCanRollback && wifiSwitchPrevSsid.length() > 0) {
      DPRINTLN("WiFi switch failed, starting rollback to previous SSID.");
      wifiSwitchLastResult = "rollback_started";
      wifiSwitchRollbackRunning = true;
      wifiSwitchCanRollback = false;
      wifiSwitchSaveOnSuccess = false;
      wifiSwitchTargetSsid = wifiSwitchPrevSsid;
      wifiSwitchTargetPsw = wifiSwitchPrevPsw;
      wifiSwitchStartMs = nowMs;
      setWifiModeForSwitchKeepAp();
      delay(30);
      if (wifiSwitchTargetPsw.length() > 0) WiFi.begin(wifiSwitchTargetSsid.c_str(), wifiSwitchTargetPsw.c_str());
      else WiFi.begin(wifiSwitchTargetSsid.c_str());
    } else {
      wifiSwitchLastResult = "connect_failed";
      wifiSwitchPending = false;
      wifiSwitchRollbackRunning = false;
      startStandaloneMode();
    }
  }
}

inline int mod(int a, int b) {
  int r = a % b;
  return r < 0 ? r + b : r;
}


void testTubes(int dely) {
  Fdelay(dely);
  cathodeProtRunning = true;
  DPRINT("Testing tubes: ");
  for (int i = 0; i < 10; i++) {
    DPRINT(i); DPRINT(" ");
    for (int j = 0; j < maxDigits; j++) {
      newDigit[j] = i;
      digit[i] = i;
      digitDP[j] = i % 2;
    }
    changeDigit();
    Fdelay(dely);
  }
  DPRINTLN(" ");
  Fdelay(1000);
  memset(digitDP, 0, sizeof(digitDP));
  cathodeProtRunning = false;
}

void playTubes() {
  static byte tube = 0;
  memset(digitDP, 0, sizeof(digitDP));
  for (int j = 0; j < maxDigits; j++) {
    newDigit[j] = tube++ % maxDigits;
  }
  changeDigit();
}

void printSensors(void) {
  static unsigned long lastRun = millis();
  
  if ((millis() - lastRun) < 30000) return;
  lastRun = millis();  
  int validTempCount = 0;
  for (int i=0; i<useTemp; i++) {
    if (isValidTemperatureReading(temperature[i])) validTempCount++;
  }
  if (validTempCount > 0) {
    DPRINT("Temperature ("); DPRINT(validTempCount); DPRINT("): ");
    bool first = true;
    for (int i=0; i<useTemp; i++) {
      if (!isValidTemperatureReading(temperature[i])) continue;
      if (!first) DPRINT(", ");
      DPRINT(temperature[i]);
      first = false;
    }
    DPRINTLN(" ");
  }

  int validHumidCount = 0;
  for (int i=0; i<useHumid; i++) {
    if (isValidHumidityReading(humid[i])) validHumidCount++;
  }
  if (validHumidCount > 0) {
    DPRINT("Humidity    ("); DPRINT(validHumidCount); DPRINT("): ");
    bool first = true;
    for (int i=0; i<useHumid; i++) {
      if (!isValidHumidityReading(humid[i])) continue;
      if (!first) DPRINT(", ");
      DPRINT(humid[i]);
      first = false;
    }
    DPRINTLN(" ");
  }
}

void printChar(int i) {
  if (i < 10)      {DPRINT(i);}
    else if (i==10)  {DPRINT(" ");}
    else if (i==TEMP_CHARCODE)    {DPRINT("C");}
    else if (i==GRAD_CHARCODE)    {DPRINT("°");}
    else if (i==18)    {DPRINT(".");}
    else if (i==PERCENT_CHARCODE) {DPRINT("%"); }   
    else if (i==19) {DPRINT("I");}    
    else if (i==14) {DPRINT("P");}    
    else    DPRINT("-");
    
    if (digitDP[i]) {DPRINT(".");} else {DPRINT(" ");}
}

void printDigits(unsigned long timeout) {
  static unsigned long lastRun = millis();

  if ((millis() - lastRun) < timeout) return;
  lastRun = millis();
  
  #ifdef DEBUG
  DPRINT("   digit: [");  for (int i = maxDigits - 1; i >= 0; i--) {printChar(digit[i]);} DPRINT("]");
  DPRINT(colonBlinkState ? " * " : "   ");
  //DPRINT("\noldDigit: ");  for (int i = maxDigits - 1; i >= 0; i--) {printChar(oldDigit[i]);}
  //DPRINT("\nnewDigit: ");  for (int i = maxDigits - 1; i >= 0; i--) {printChar(newDigit[i]);}

  /*
    if ((millis()/1000%10) == 1) {  //show free memory for debugging memory leak
    DPRINT("Heap:"); DPRINT(ESP.getFreeHeap()); DPRINT(" byte");
    #if defined(ESP8266)
      DPRINT(" Fragmentation:");  DPRINT(100 - ESP.getMaxFreeBlockSize() * 100.0 / ESP.getFreeHeap()); DPRINT("%");
    #endif
    }
  */
  //DPRINT("INT:"); DPRINT(intCounter); intCounter = 0;  //show multiplex interrupt counter
  //DPRINT(" ESaving:"); DPRINT(EEPROMsaving);
  if (useLux>0) {
    DPRINT("  Lux:"); DPRINT(lx);
  }
  if (RADAR_PIN>=0) {
    //DPRINT("  RadarPin:"); DPRINT(digitalRead(RADAR_PIN)); DPRINT("  lastOn:"); DPRINT(radarLastOn);  DPRINT("  sec:"); DPRINTLN((millis()-radarLastOn)/1000);  
  }
  //DPRINT("  tON:"); DPRINT(timerON); DPRINT("  tOFF:"); DPRINT(timerOFF);   //Multiplex timing values for testing
  if (WiFi.status() != WL_CONNECTED) DPRINT("  no WIFI");
  DPRINTLN(" ");
  printSensors();
  #endif  
}



// Returns current motion state (radar/PIR), from GPIO if available, else from MQTT state.
bool isMotionDetectedNow() {
#ifdef RADAR_PIN
  if (RADAR_PIN >= 0) {
    return digitalRead(RADAR_PIN);
  }
#endif
  return mqttRadarON;
}

void checkTubePowerOnOff(void) {
  #ifdef CLOCK_54
  // NCS312 (D1 R32) uses a simple display power model to avoid motion/radar-induced flapping.
  const bool wantOn54 = prm.enableTimeDisplay && (!prm.manualDisplayOff);
  if (wantOn54 != tubesPowerState) {
    tubesPowerState = wantOn54;
    DPRINT("DISPLAY: ");
    DPRINTLN(tubesPowerState ? "ON" : "OFF");
    if (!tubesPowerState) {
      clearDigits();
      rawWrite();
      #ifdef USE_NEOPIXEL
      darkenNeopixels();
      #endif
    }
  }
#ifdef TUBE_POWER_PIN
  if (GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)TUBE_POWER_PIN)) {
  #if defined(TUBE_POWER_ON)
    digitalWrite((int)TUBE_POWER_PIN, tubesPowerState ? TUBE_POWER_ON : !TUBE_POWER_ON);
  #else
    digitalWrite((int)TUBE_POWER_PIN, tubesPowerState ? HIGH : LOW);
  #endif
  }
#endif
  return;
  #endif

  // Clock64-friendly tubes sleep logic (based on V8 behavior):
  // - If prm.tubesSleep is enabled, tubes are OFF by default and wake ON for N seconds after motion.
  // - N is prm.tubesWakeSeconds (seconds). If not set, fallback to prm.radarTimeout (minutes) for backward compatibility.
  // - Works with or without TUBE_POWER_PIN. If there is no power pin, we soft-blank in writeDisplaySingleGuarded().

  const unsigned long now = millis();

  // Raw motion read (for debug / polarity checks)
  const bool motion = isMotionDetectedNow();

  // Debug on rising edge (no spam)
  static bool lastMotion = false;
  if (motion && !lastMotion) {
    DPRINTF("RADAR: motion detected (GPIO%d)\n", (int)RADAR_PIN);
  }
  lastMotion = motion;

  if (motion) {
    tubesLastMotionMs = now;
  }

  // Determine wake window in milliseconds
  unsigned long wakeMs = 0;
  if (prm.tubesWakeSeconds > 0) {
    wakeMs = (unsigned long)prm.tubesWakeSeconds * 1000UL;
  } else if (prm.radarTimeout > 0) {
    wakeMs = (unsigned long)prm.radarTimeout * 60UL * 1000UL;
  } else {
    wakeMs = 10UL * 1000UL; // safe default
  }

  const bool sleepEnabled = prm.wakeOnMotionEnabled; // Wake toggle controls sleep

  bool radarAllowed = true;
  if (sleepEnabled) {
    if (tubesLastMotionMs == 0) {
      radarAllowed = false;
    } else {
      radarAllowed = ((unsigned long)(now - tubesLastMotionMs) < wakeMs);
    }
  } else {
    // Keep timer fresh so switching into sleep doesn't instantly turn OFF
    if (tubesLastMotionMs == 0) tubesLastMotionMs = now;
    radarAllowed = true;
  }

  // Manual displayON + MQTT radar gating
  // If wake-on-motion is disabled, keep display always ON (unless manual OFF)
  const bool motionMode = prm.wakeOnMotionEnabled;
  const bool effectiveRadarGate = motionMode ? mqttRadarON : true;
  const bool effectiveRadarAllowed = motionMode ? radarAllowed : true;
  const bool wantOn = prm.enableTimeDisplay && (!prm.manualDisplayOff) && effectiveRadarAllowed && effectiveRadarGate;

  if (wantOn != tubesPowerState) {
    tubesPowerState = wantOn;
    DPRINT("DISPLAY: ");
    DPRINTLN(tubesPowerState ? "ON" : "OFF");
    if (!tubesPowerState) {
      // Ensure tubes and LEDs immediately go dark when display is OFF.
      clearDigits();
      rawWrite(); // push "all off" to HV driver
      #ifdef USE_NEOPIXEL
    darkenNeopixels();
#endif
    }
  }


  // ---- Debug snapshot fields (updated every loop, printed once/sec by debugSnapshot()) ----
#if DEBUG_ENABLED
  dbg.motion = motion;
  dbg.tubesSleep = sleepEnabled;
  dbg.wakeOnMotionEnabled = prm.wakeOnMotionEnabled;
  dbg.manualDisplayOff = prm.manualDisplayOff;
  dbg.enableTimeDisplay = prm.enableTimeDisplay;
  dbg.mqttRadarON = mqttRadarON;
  dbg.radarAllowed = effectiveRadarAllowed;
  dbg.tubesPowerState = tubesPowerState;
  dbg.tubesWakeSeconds = (uint16_t)(prm.tubesWakeSeconds ? prm.tubesWakeSeconds : (prm.radarTimeout ? (prm.radarTimeout * 60) : 10));
  dbg.nowMs = now;
  dbg.lastMotionMs = tubesLastMotionMs;
  dbg.wakeMs = wakeMs;

  // wake left seconds (0 if sleeping / always-on mode / no timer)
  uint16_t left = 0;
  if (prm.wakeOnMotionEnabled && tubesLastMotionMs != 0 && wakeMs > 0) {
    unsigned long elapsedMs = (unsigned long)(now - tubesLastMotionMs);
    if (elapsedMs < wakeMs) left = (uint16_t)((wakeMs - elapsedMs + 999UL) / 1000UL);
  }
  dbg.wakeLeftSeconds = left;

  // timeSource is already computed for UI in handleSendCurrentInfos(); here we keep a lightweight best-effort:
  dbg.timeSource = (WiFi.status() == WL_CONNECTED) ? "NTP/WiFi" : "NoWiFi";
#endif
#ifdef TUBE_POWER_PIN
  // If you have a real HV power pin, use it as well.
  if (GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)TUBE_POWER_PIN)) {
  #if defined(TUBE_POWER_ON)
    digitalWrite((int)TUBE_POWER_PIN, tubesPowerState ? TUBE_POWER_ON : !TUBE_POWER_ON);
  #else
    digitalWrite((int)TUBE_POWER_PIN, tubesPowerState ? HIGH : LOW);
  #endif
  }
#endif
}

//Calculation parameters are defined in clocks.h
//https://www.pcboard.ca/ldr-light-dependent-resistor
 
int luxMeter(void) {    
#if LIGHT_SENSOR_PIN >=0   
  static float oldLux = MAXIMUM_LUX;  
  float ADCdata = analogRead(LIGHT_SENSOR_PIN); //DPRINT("ADC:"); DPRINTLN(ADCdata);
  float ldrResistance = (MAX_ADC_READING - ADCdata) / ADCdata * REF_RESISTANCE;
  float ldrLux = LUX_CALC_SCALAR * pow(ldrResistance, LUX_CALC_EXPONENT);
  if ((ldrLux>=MAXIMUM_LUX)||(ldrLux <0)) ldrLux = MAXIMUM_LUX;   //Limit lux value to maximum
  oldLux = oldLux + (ldrLux-oldLux)/10;   //slow down Lux change
  return (int)oldLux;
#else
  return (0);
#endif
}

void getLightSensor(void) {
  static unsigned long lastRun = 0;
  static int oldLx = 0;
  int tmp;
  if ((millis()-lastRun)<500) return;
  lastRun = millis();

  if (BH1750exist) {
    tmp = getBH1750();
    if ((abs(tmp-oldLx)>1) || (tmp >= MAXIMUM_LUX)) {
      lx=tmp; oldLx = lx; 
    }
    if (lx>=MAXIMUM_LUX-2) lx = MAXIMUM_LUX;
    autoBrightness = prm.enableAutoDim;
  }
  else if (LDRexist) {
    tmp = luxMeter();
    if ((abs(tmp-oldLx)>1) || (tmp >= MAXIMUM_LUX))  {
      lx=tmp;  oldLx = lx; 
    }
    if (lx>=MAXIMUM_LUX-2) lx = MAXIMUM_LUX;
    autoBrightness = prm.enableAutoDim;
  }
  else {
    lx = MAXIMUM_LUX;
    autoBrightness = false;
  }
}

void checkWifiMode() {     
static boolean oldMode = prm.wifiMode;  

  if (oldMode != prm.wifiMode) {
    if (prm.wifiMode) {
        startNewWifiMode();
      }
      else  {
        startStandaloneMode();
      }
       oldMode = prm.wifiMode;  
    }  
}

void checkPrm() {
  static byte oldPrm[sizeof(prm)];
  int tmp = memcmp(oldPrm,&prm,sizeof(prm));
  if (tmp!=0){
    DPRINT("Prm mismatch!"); DPRINTLN(tmp);
    memcpy(oldPrm,&prm,sizeof(prm));
  }
}

#if defined(ESP32)
static void calibrateTouchButton() {
  if (TOUCH_BUTTON_PIN < 0) return;
  uint32_t sum = 0;
  const int samples = 16;
  for (int i = 0; i < samples; i++) {
    sum += (uint16_t)touchRead(TOUCH_BUTTON_PIN);
    delay(4);
  }
  touchBaseline = (uint16_t)(sum / samples);
  uint16_t margin = (uint16_t)max(15, (int)(touchBaseline / 4));
  touchThreshold = (touchBaseline > margin) ? (touchBaseline - margin) : 5;
  if (touchThreshold < 5) touchThreshold = 5;
  debugLogf("[Touch33] calibrated baseline=%u threshold=%u", touchBaseline, touchThreshold);
}

void cycleFixedColorPalette(int step) {
  static const uint8_t palette[][3] = {
    {255,   0,   0},
    {255,  48,   0},
    {255,  96,   0},
    {255, 160,   0},
    {255, 220,   0},
    {180, 255,   0},
    {100, 255,   0},
    {  0, 255,   0},
    {  0, 255, 120},
    {  0, 255, 220},
    {  0, 180, 255},
    {  0, 110, 255},
    {  0,  40, 255},
    { 90,   0, 255},
    {160,   0, 255},
    {230,   0, 255},
    {255,   0, 190},
    {255,   0, 120},
    {255, 255, 255},
    {255, 190, 120},
    {255, 120,  20},
    {255,  80, 160}
  };
  const int count = sizeof(palette) / sizeof(palette[0]);
  if (count <= 0) return;
  if (step == 0) step = 1;
  int idx = -1;
  for (int i = 0; i < count; i++) {
    if (settings.rgbFixR == palette[i][0] && settings.rgbFixG == palette[i][1] && settings.rgbFixB == palette[i][2]) {
      idx = i;
      break;
    }
  }
  int next = 0;
  if (idx < 0) {
    next = (step > 0) ? 0 : (count - 1);
  } else {
    int delta = (step > 0) ? 1 : -1;
    next = (idx + delta + count) % count;
  }
  settings.rgbFixR = palette[next][0];
  settings.rgbFixG = palette[next][1];
  settings.rgbFixB = palette[next][2];
  prm.rgbEffect = 1;
  saveRGBSettingsToEEPROM();
}

static void cycleTouchColor() {
  cycleFixedColorPalette(1);
  debugLogf("[Touch33] color change -> R:%u G:%u B:%u", settings.rgbFixR, settings.rgbFixG, settings.rgbFixB);
}

static void executeTouchAction(uint8_t action, const char* src) {
  switch (action) {
    case TOUCH_ACTION_ALARM_OFF:
      alarmON = false;
      writeAlarmPin(!ALARM_ON);
      debugLogf("[Touch33] %s action: Alarm OFF", src);
      break;
    case TOUCH_ACTION_COLOR_CHANGE:
      cycleTouchColor();
      debugLogf("[Touch33] %s action: Color change", src);
      break;
    case TOUCH_ACTION_COLOR_PREV:
      cycleFixedColorPalette(-1);
      debugLogf("[Touch33] %s action: Color previous", src);
      break;
    case TOUCH_ACTION_DISPLAY_OFF:
      prm.manualDisplayOff = true;
      debugLogf("[Touch33] %s action: Display OFF", src);
      break;
    case TOUCH_ACTION_DISPLAY_TOGGLE:
      prm.manualDisplayOff = !prm.manualDisplayOff;
      debugLogf("[Touch33] %s action: Display %s", src, prm.manualDisplayOff ? "OFF" : "ON");
      break;
    default:
      debugLogf("[Touch33] %s action: None", src);
      break;
  }
}

void executeGestureMappedAction(uint8_t direction) {
  switch (direction) {
    case 0:
      executeTouchAction(settings.gestureUpAction, "gesture-up");
      break;
    case 1:
      executeTouchAction(settings.gestureDownAction, "gesture-down");
      break;
    case 2:
      executeTouchAction(settings.gestureLeftAction, "gesture-left");
      break;
    case 3:
      executeTouchAction(settings.gestureRightAction, "gesture-right");
      break;
    default:
      break;
  }
}

static void processTouchButton() {
  if (TOUCH_BUTTON_PIN < 0) return;
  if (touchThreshold == 0) {
    calibrateTouchButton();
  }

  uint32_t nowMs = millis();
  uint16_t raw = (uint16_t)touchRead(TOUCH_BUTTON_PIN);
  bool isTouched = (raw > 0) && (raw < touchThreshold);

  if (isTouched) {
    if (!touchPressed) {
      touchPressed = true;
      touchPressedAtMs = nowMs;
      touchLastLogMs = 0;
      debugLogf("[Touch33] down raw=%u threshold=%u", raw, touchThreshold);
    }
    if ((nowMs - touchLastLogMs) >= TOUCH_LOG_INTERVAL_MS) {
      touchLastLogMs = nowMs;
      debugLogf("[Touch33] raw=%u", raw);
    }
    return;
  }

  if (touchPressed) {
    touchPressed = false;
    uint32_t duration = nowMs - touchPressedAtMs;
    debugLogf("[Touch33] up raw=%u duration=%ums", raw, duration);

    if (duration >= TOUCH_LONG_PRESS_MS) {
      touchShortPending = false;
      executeTouchAction(prm.touchLongAction, "long");
      return;
    }

    if (touchShortPending && ((nowMs - touchLastReleaseMs) <= TOUCH_DOUBLE_PRESS_MS)) {
      touchShortPending = false;
      executeTouchAction(prm.touchDoubleAction, "double");
      return;
    }

    touchShortPending = true;
    touchLastReleaseMs = nowMs;
  }

  if (touchShortPending && ((nowMs - touchLastReleaseMs) > TOUCH_DOUBLE_PRESS_MS)) {
    touchShortPending = false;
    executeTouchAction(prm.touchShortAction, "short");
  }
}
#endif


void loop(void) {
  //checkPrm();
  telnetHousekeeping();
  writeDisplay2();
  if (WiFi.status() != WL_CONNECTED) 
    dnsServer.processNextRequest();
  enableDisplay(3000);
  timeProgram();
  //writeDisplaySingleGuarded();
  writeDisplay2();
#ifdef USE_NEOPIXEL
  static bool neopixelsForcedOff = false;
#endif

#ifdef USE_NEOPIXEL
  if (tubesPowerState || alarmON) {
    neopixelsForcedOff = false;
    doAnimationMakuna();
  } else {
    if (!neopixelsForcedOff) {
      darkenNeopixels();
      neopixelsForcedOff = true;
    }
  }
#else
  doAnimationMakuna();
#endif
  checkTubePowerOnOff();
  doAnimationPWM();
  alarmSound();
#if defined(ESP32)
  processTouchButton();
#endif
  processGestureSensor();
  debugSnapshot();
  //heapGuardTick();
  getLightSensor();
  processPendingWifiSwitch();
  //checkWifiMode();
  if (prm.wifiMode) { //Wifi Clock Mode
    if (WiFi.status() == WL_CONNECTED) {
      if (prm.mqttEnable) mqttSend();
    }
    else {
      WiFiMode_t wm = WiFi.getMode();
      bool apActive = (wm == WIFI_AP) || (wm == WIFI_AP_STA);
      if (!wifiSwitchPending && !wifiRecoveryApMode && !apActive) {
        resetWiFi();
      }
    }
  }
  editor();
  if (makeFirmwareUpdate) {
    makeFirmwareUpdate = false;
    doFirmwareUpdate();
  }
  if (makeCathodeProtect) {
    makeCathodeProtect = false;
    doCathodeProtect();
  }  
  yield();
}

void doReset(void) {
  DPRINTLN("Restart Clock...");
  #if defined(ESP8266)
    WiFi.setOutputPower(0);
  #endif

  WiFi.disconnect();
  delay(1000);
  ESP.restart();
}

#if defined(VQC10) || defined(TUBE1CLOCK) || defined(WORDCLOCK)
#else
  void writeDisplay2(void){}  //not interrupt driven display handler
#endif
