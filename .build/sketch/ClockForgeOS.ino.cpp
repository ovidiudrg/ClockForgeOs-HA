#include <Arduino.h>
#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
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
#ifdef USE_MQTT
void mqttReconfigure();
#endif
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

static const unsigned long WEB_AUTH_TTL_MS = 30UL * 60UL * 1000UL;
static const uint8_t WEB_AUTH_MAX_TOKENS = 4;
static const uint8_t WEB_AUTH_MAX_FAILED_ATTEMPTS = 6;
static const unsigned long WEB_AUTH_LOCKOUT_MS = 10UL * 60UL * 1000UL;
static const unsigned long WEB_AUTH_MIN_LOGIN_INTERVAL_MS = 400UL;

struct WebAuthSession {
  String token;
  unsigned long expireMs;
};

static WebAuthSession webAuthSessions[WEB_AUTH_MAX_TOKENS];
static uint8_t webAuthNextSlot = 0;
static uint8_t webAuthFailedAttempts = 0;
static unsigned long webAuthLockoutUntilMs = 0;
static unsigned long webAuthLastLoginAttemptMs = 0;

#line 301 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static bool secureEquals(const String& provided, const char* expected);
#line 315 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static bool isWebAuthLockedOut();
#line 319 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static uint16_t webAuthLockoutSecondsLeft();
#line 325 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static bool isDefaultWebPasswordInUse();
#line 329 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static String generateWebAuthToken();
#line 344 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static bool isWebAuthExpired(unsigned long expireMs);
#line 349 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void webAuthCleanupExpired();
#line 359 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static int webAuthFindTokenIndex(const String& token);
#line 369 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static String webAuthIssueToken();
#line 390 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static bool requireWebAuth(AsyncWebServerRequest *request);
#line 418 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static bool isValidTemperatureReading(float value);
#line 422 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static bool isValidHumidityReading(float value);
#line 479 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void pushRing(const String& line);
#line 485 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void telnetHousekeeping();
#line 506 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static uint8_t telnetActiveConnections();
#line 514 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void telnetBroadcast(const String& line);
#line 520 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void debugLogLine(const String& line);
#line 557 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void debugLogf(const char* fmt, ...);
#line 649 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void writeDisplaySingleGuarded();
#line 684 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void rawWrite();
#line 743 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static const char * dbgTimeSourceStr(const char* ts);
#line 746 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void debugSnapshot();
#line 828 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void updateExternalDeviceSensor(uint8_t slot, float value);
#line 948 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void applyOnboardLedState();
#line 957 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void forceOnboardLedOffEarly();
#line 1011 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void clearLogRing();
#line 1019 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void closeTelnetClients();
#line 1025 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void heapGuardTick();
#line 1071 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void regPin(byte p,const char * txt);
#line 1098 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void listPins();
#line 1116 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void startTimer();
#line 1135 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void stopTimer();
#line 1150 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void clearDigits();
#line 1157 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void Fdelay(unsigned long d);
#line 1201 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
float round1(float in);
#line 1205 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void sanitizeAsciiCString(char* buf, size_t len);
#line 1231 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void enableDisplay(unsigned long timeout);
#line 1239 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void disableDisplay();
#line 1245 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
boolean findBestWifi();
#line 1285 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void wifiManager();
#line 1326 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void startNewWifiMode();
#line 1386 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void startWifiMode();
#line 1435 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
boolean updateTimefromTimeserver();
#line 1486 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void startMDNS();
#line 1503 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void startStandaloneMode();
#line 1534 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void doFirmwareUpdate();
#line 1604 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void doCathodeProtect();
#line 1646 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void startServer();
#line 3157 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void setWifiModeForSwitchKeepAp();
#line 3166 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void startWifiSwitchAttempt(const String& ssid, const String& psw, bool saveOnSuccess, AsyncWebServerRequest *request);
#line 3404 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void setup();
#line 3698 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void calcTime();
#line 3718 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void timeProgram();
#line 3796 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void loadEEPROM();
#line 3813 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void saveEEPROM();
#line 3828 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void settingsSetDefaults();
#line 3873 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
bool settingsLoadFromEEPROM();
#line 3910 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void settingsSaveToEEPROM();
#line 3930 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void requestSaveEEPROM();
#line 3934 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void processPendingEepromSave();
#line 4041 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void newCathodeProtect(unsigned long t,int dir);
#line 4128 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void cathodeProtect();
#line 4186 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void incMod10(byte &x);
#line 4219 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayTemp(byte ptr);
#line 4285 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayHumid(byte ptr);
#line 4325 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayPressure(byte ptr);
#line 4379 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayDate();
#line 4432 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayTime4();
#line 4454 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayTime6();
#line 4494 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayTime8();
#line 4528 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void changeDigit();
#line 4780 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void writeAlarmPin(boolean newState);
#line 4791 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void alarmSound(void);
#line 4851 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void evalShutoffTime(void);
#line 4882 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void writeIpTag(byte iptag);
#line 4904 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void showMyIp(void);
#line 4916 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void fifteenMinToHM(int &hours, int &minutes, int fifteenMin);
#line 4922 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void resetWiFi(void);
#line 4980 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void processPendingWifiSwitch();
#line 5056 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
int mod(int a, int b);
#line 5062 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void testTubes(int dely);
#line 5082 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void playTubes();
#line 5091 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void printSensors(void);
#line 5129 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void printChar(int i);
#line 5143 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void printDigits(unsigned long timeout);
#line 5181 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
bool isMotionDetectedNow();
#line 5190 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void checkTubePowerOnOff(void);
#line 5300 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
int luxMeter(void);
#line 5314 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void getLightSensor(void);
#line 5343 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void checkWifiMode();
#line 5357 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void checkPrm();
#line 5367 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void calibrateTouchButton();
#line 5431 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void cycleTouchColor();
#line 5436 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void executeTouchAction(uint8_t action, const char* src);
#line 5484 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static void processTouchButton();
#line 5537 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void loop(void);
#line 5602 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void doReset(void);
#line 5615 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void writeDisplay2(void);
#line 91 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\DHT_temp.ino"
void setupDHTemp();
#line 92 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\DHT_temp.ino"
void getDHTemp();
#line 55 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
void setupI2Csensors();
#line 209 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
void getBME280();
#line 230 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
void getBMP280();
#line 245 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
void getAHTX0();
#line 265 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
void getSHT21();
#line 282 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
void getI2Csensors();
#line 295 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
void I2Cscanner();
#line 337 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
int getBH1750();
#line 164 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MAX6921_ESP32.ino"
void generateBitTable();
#line 140 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dallas_temp.ino"
void setupDallasTemp();
#line 141 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dallas_temp.ino"
void requestDallasTemp(boolean force);
#line 142 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dallas_temp.ino"
void getTemp();
#line 143 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dallas_temp.ino"
void resetSensors();
#line 91 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\gps.ino"
void setupGPS();
#line 92 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\gps.ino"
boolean getGPS();
#line 63 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
static float parseNumericPayload(const char* s);
#line 69 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
static bool parseTemperaturePayload(const char* s, float& out);
#line 86 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
static bool parseHumidityPayload(const char* s, float& out);
#line 103 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
static bool parsePressurePayload(const char* s, float& out);
#line 118 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
static bool parseLuxPayload(const char* s, int& out);
#line 136 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
static bool mqttTopicMatches(const char* filter, const char* topic);
#line 171 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
static void syncMasterTopicsFromConfig();
#line 201 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
static void mqttEnsureSubscriptions();
#line 220 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
void onMqttMessage(const char* topic, const uint8_t* payload, uint16_t length);
#line 287 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
void onMqttConnected();
#line 300 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
void setupMqtt();
#line 426 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
void mqttSend();
#line 10 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
static uint8_t rtcBcdToDec(uint8_t v);
#line 14 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
static uint8_t rtcDecToBcd(uint8_t v);
#line 18 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
static bool pcf8563ReadDateTime(int &yy, int &mo, int &dd, int &hh, int &mm, int &ss);
#line 52 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
static bool pcf8563WriteDateTime(int yy, int mo, int dd, int hh, int mm, int ss);
#line 116 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
void setupRTC();
#line 176 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
void editor();
#line 205 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
void showValue();
#line 264 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
void getRTC();
#line 297 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
void saveRTC();
#line 314 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
void scanButFLD(unsigned long mill);
#line 369 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
void scanButSET(unsigned long mill);
#line 431 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
int I2C_ClearBus();
#line 46 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
bool isMappedActivePixel(int idx);
#line 81 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
static void applyCurrentLimitAndShow(uint8_t requestedBrightness);
#line 145 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void setupNeopixel();
#line 159 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
RgbColor Wheel(byte WheelPos);
#line 180 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
byte randomWheelNoWhite();
#line 188 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void alarmLight();
#line 218 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void showSolidNeopixels(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
#line 227 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void rainbow();
#line 249 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void rainbow2();
#line 281 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void effect1();
#line 327 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
RgbColor blendColors(RgbColor color1, RgbColor color2, float ratio);
#line 334 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void effect2();
#line 382 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
int colorDistance(int c1,int c2);
#line 390 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void effect3(boolean enableRandom,boolean eachPixelRandom);
#line 490 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void effect4();
#line 536 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void setPixels(byte tubeNo, RgbColor c);
#line 544 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void fixColor();
#line 558 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void kitt();
#line 578 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void heartbeat();
#line 664 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
RgbColor scaleColor(RgbColor color, float brightness);
#line 301 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static bool secureEquals(const String& provided, const char* expected) {
  const size_t providedLen = provided.length();
  const size_t expectedLen = strlen(expected);
  const size_t maxLen = (providedLen > expectedLen) ? providedLen : expectedLen;
  uint8_t diff = (providedLen == expectedLen) ? 0 : 1;

  for (size_t i = 0; i < maxLen; i++) {
    const uint8_t a = (i < providedLen) ? (uint8_t)provided[i] : 0;
    const uint8_t b = (i < expectedLen) ? (uint8_t)expected[i] : 0;
    diff |= (uint8_t)(a ^ b);
  }
  return diff == 0;
}

static bool isWebAuthLockedOut() {
  return (webAuthLockoutUntilMs != 0) && ((long)(millis() - webAuthLockoutUntilMs) < 0);
}

static uint16_t webAuthLockoutSecondsLeft() {
  if (!isWebAuthLockedOut()) return 0;
  unsigned long remainingMs = webAuthLockoutUntilMs - millis();
  return (uint16_t)((remainingMs + 999UL) / 1000UL);
}

static bool isDefaultWebPasswordInUse() {
  return (strcmp(WEB_ADMIN_PASSWORD, "ChangeMeNow!") == 0);
}

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

static bool isWebAuthExpired(unsigned long expireMs) {
  if (expireMs == 0) return true;
  return ((long)(millis() - expireMs) >= 0);
}

static void webAuthCleanupExpired() {
  for (uint8_t i = 0; i < WEB_AUTH_MAX_TOKENS; i++) {
    if (webAuthSessions[i].token.length() == 0) continue;
    if (isWebAuthExpired(webAuthSessions[i].expireMs)) {
      webAuthSessions[i].token = "";
      webAuthSessions[i].expireMs = 0;
    }
  }
}

static int webAuthFindTokenIndex(const String& token) {
  if (token.length() == 0) return -1;
  webAuthCleanupExpired();
  for (uint8_t i = 0; i < WEB_AUTH_MAX_TOKENS; i++) {
    if (webAuthSessions[i].token.length() == 0) continue;
    if (webAuthSessions[i].token == token) return (int)i;
  }
  return -1;
}

static String webAuthIssueToken() {
  webAuthCleanupExpired();

  int slot = -1;
  for (uint8_t i = 0; i < WEB_AUTH_MAX_TOKENS; i++) {
    if (webAuthSessions[i].token.length() == 0) {
      slot = (int)i;
      break;
    }
  }
  if (slot < 0) {
    slot = (int)(webAuthNextSlot % WEB_AUTH_MAX_TOKENS);
    webAuthNextSlot = (uint8_t)((webAuthNextSlot + 1) % WEB_AUTH_MAX_TOKENS);
  }

  String token = generateWebAuthToken();
  webAuthSessions[slot].token = token;
  webAuthSessions[slot].expireMs = millis() + WEB_AUTH_TTL_MS;
  return token;
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

  int tokenIdx = webAuthFindTokenIndex(token);
  if (tokenIdx < 0) {
    request->send(401, "application/json", "{\"error\":\"unauthorized\"}");
    return false;
  }

  webAuthSessions[tokenIdx].expireMs = millis() + WEB_AUTH_TTL_MS;
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

static void sanitizeAsciiCString(char* buf, size_t len) {
  if (!buf || len == 0) return;
  buf[len - 1] = '\0';
  for (size_t i = 0; i < len; i++) {
    unsigned char c = (unsigned char)buf[i];
    if (c == '\0') return;
    if (c < 32 || c > 126) {
      buf[i] = '\0';
      return;
    }
  }
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
  const unsigned long nowMs = millis();
  if ((nowMs - webAuthLastLoginAttemptMs) < WEB_AUTH_MIN_LOGIN_INTERVAL_MS) {
    request->send(429, "application/json", "{\"error\":\"too_many_requests\"}");
    return;
  }
  webAuthLastLoginAttemptMs = nowMs;

  if (isDefaultWebPasswordInUse()) {
    request->send(503, "application/json", "{\"error\":\"default_password_not_allowed\"}");
    return;
  }

  if (isWebAuthLockedOut()) {
    DynamicJsonDocument lockDoc(96);
    lockDoc["error"] = "login_locked";
    lockDoc["retrySec"] = webAuthLockoutSecondsLeft();
    String lockJson;
    serializeJson(lockDoc, lockJson);
    request->send(429, "application/json", lockJson);
    return;
  }

  if (!request->hasParam("password", true)) {
    request->send(400, "application/json", "{\"error\":\"missing_password\"}");
    return;
  }

  String password = request->getParam("password", true)->value();
  if (!secureEquals(password, WEB_ADMIN_PASSWORD)) {
    if (webAuthFailedAttempts < 255) webAuthFailedAttempts++;
    if (webAuthFailedAttempts >= WEB_AUTH_MAX_FAILED_ATTEMPTS) {
      webAuthLockoutUntilMs = millis() + WEB_AUTH_LOCKOUT_MS;
      webAuthFailedAttempts = 0;
    }
    request->send(401, "application/json", "{\"error\":\"invalid_credentials\"}");
    return;
  }

  webAuthFailedAttempts = 0;
  webAuthLockoutUntilMs = 0;
  String issuedToken = webAuthIssueToken();

  DynamicJsonDocument doc(256);
  doc["ok"] = true;
  doc["token"] = issuedToken;
  doc["ttlSec"] = (WEB_AUTH_TTL_MS / 1000UL);
  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

void handleConfigChanged(AsyncWebServerRequest *request) {
  bool doSave = true; // added: allow skipping EEPROM save for runtime-only settings
  bool saveImmediately = false; // for critical settings that must survive quick reconnect/reboot
  bool mqttNeedsReconfigure = false;

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
        value.trim();
        value.toCharArray(prm.mqttBrokerAddr,sizeof(prm.mqttBrokerAddr));
        saveImmediately = true;
        mqttNeedsReconfigure = true;
      }
      else if (key == "mqttBrokerUser") {
        value.trim();
        value.toCharArray(prm.mqttBrokerUser,sizeof(prm.mqttBrokerUser));
        saveImmediately = true;
        mqttNeedsReconfigure = true;
      }
      else if (key == "mqttBrokerPsw") {
        value.toCharArray(prm.mqttBrokerPsw,sizeof(prm.mqttBrokerPsw));
        saveImmediately = true;
        mqttNeedsReconfigure = true;
      }         
      else if (key == "mqttBrokerRefresh") {
        prm.mqttBrokerRefresh = value.toInt();
      }     
      else if (key == "mqttEnable") {
        prm.mqttEnable = (value == "true");
        mqttNeedsReconfigure = true;
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
      if (saveImmediately) {
        // MQTT auth/host is safety-critical; persist now to avoid stale reconnect values.
        saveEEPROM();
        eepromSavePending = false;
      }
      else if (doSave) {
        requestSaveEEPROM();
      }
#ifdef USE_MQTT
      if (mqttNeedsReconfigure) {
        mqttReconfigure();
      }
#endif
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
  sanitizeAsciiCString(prm.mqttBrokerAddr, sizeof(prm.mqttBrokerAddr));
  sanitizeAsciiCString(prm.mqttBrokerUser, sizeof(prm.mqttBrokerUser));
  sanitizeAsciiCString(prm.mqttBrokerPsw, sizeof(prm.mqttBrokerPsw));
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
  if (tubesPowerState || alarmON) {
    doAnimationPWM();
  }
  alarmSound();
  checkTubePowerOnOff();
#if defined(ESP32)
  processTouchButton();
#endif
  processGestureSensor();
  debugSnapshot();
  heapGuardTick();
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

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\DHT_temp.ino"
#ifdef USE_DHT_TEMP

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
// TEMP_DHT_PIN is used to connect the sensor
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#ifndef DHTTYPE
  #define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321  //default DHT type
#endif  
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like ESP8266 or ESP32 connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor


DHT dht(TEMP_DHT_PIN, DHTTYPE);
int DHTtempPtr = -1;
int DHThumidPtr = -1;

void setupDHTemp() {
  float tempTMP, humidTMP;
  regPin(TEMP_DHT_PIN,"TEMP_DHT_PIN");
  dht.begin();
  int i= 0;
  while (i<5) {
    humidTMP = dht.readHumidity();
    tempTMP = dht.readTemperature();
    DPRINT("DHT Test:"); DPRINT(tempTMP); DPRINT("/"); DPRINTLN(humidTMP);
    i++;
    if (!isnan(tempTMP)) 
      break;
    delay(1000);
  }  
  if (isnan(tempTMP)) {
    DPRINT("No DHTxx sensor found on GPIO"); DPRINTLN(TEMP_DHT_PIN);
    return;
  }
  DHTtempPtr = useTemp;  //remember my ID-s
  DHThumidPtr = useHumid;
  useTemp++;   //increase the number of sensors
  useHumid++;
}

void getDHTemp() {
static unsigned long lastRun = 0;
float tempTMP, humidTMP;

  if (EEPROMsaving) return;
  if (DHTtempPtr<0) return;  //no sensor
  //if (((millis()-lastRun)<2500) || (second()!=prm.tempStart)) return;
  if ((((millis()-lastRun)<2500) || (second()!=prm.tempStart)) && (lastRun !=0)) return;
  lastRun = millis();  

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  disableDisplay();
  humidTMP = round1(dht.readHumidity());
  if ((humidTMP>0) && (humidTMP<=100))  humid[DHThumidPtr] = humidTMP;
  tempTMP = round1(dht.readTemperature());      // Read temperature as Celsius (the default)
  if (tempTMP<99) temperature[DHTtempPtr] = tempTMP;
  enableDisplay(0);

  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float f = dht.readTemperature(true);   Compute heat index in Fahrenheit (the default)
  //float hif = dht.computeHeatIndex(f, h);
  
  //float hic = dht.computeHeatIndex(t, h, false);  // Compute heat index in Celsius (isFahreheit = false)
  
  if (isnan(humidTMP) || isnan(tempTMP)) {  // Check if any reads failed and exit early (to try again).
    DPRINTLN("DHT sensor reading error!");
    //temperature[DHTtempPtr] = 99.9f;
    //humid[DHThumidPtr] = 0;
  }
  else {
      DPRINT("DHTxx Temperature:"); DPRINT(temperature[DHTtempPtr]); DPRINT("C  Humidity:"); DPRINT(humid[DHThumidPtr]); DPRINTLN(" %");
  }
}

#else
void setupDHTemp() {}
void getDHTemp() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\HV5122.ino"
#ifdef HV5122  //Nixie driver
//P.S. 6x tubes PCB version

char tubeDriver[] = "HV5122";
//HV5122 control pins
//#define PIN_DIN  17   // DataIn  - chip0 DOUT pin is connected to chip1 DIN pin!
//#define PIN_CLK  22   // Clock
//#define PIN_OE   21   // OutputEnable

//int maxDigits = 6;

//Data pin numbers 100+ means: chip1 pins are used. Chip0-s DOUT is connected to chip1's DIN
/* Example:
  byte digitPins[maxDigits+1][10] = {
    {2,10,9,8,7,6,5,4,3,1},                      //sec  1 , chip0  (tube#0)
    {11,32,20,19,18,17,16,15,14,13},             //sec  10 , chip0 (tube#1)
    {22,31,29,30,27,28,25,26,23,24},             //min   1 , chip0 (tube#2)
    {101,131,110,109,108,107,106,105,104,103},   //min  10 , chip1 (tube#3)
    {111,132,120,119,118,117,116,115,114,113},   //hour  1 , chip1 (tube#4)
    {122,129,130,127,128,125,126,123,124,121},   //hour 10 , chip1 (tube#5)
    {0,12,21,102,112,0,    0,0,0,0}              //extra decimal point (tube0...tube6)
    };   
*/

#define SHIFT_LSB_FIRST false  //true= LSB first, false= MSB first
int PWMrefresh = 10000; //Brightness PWM period. Greater value => slower brightness PWM frequency
//int PWMtiming[11] = {0,500,800,1200,2000,2500,3000,4500,6000,8000,10000};
int PWM_min = 500;
int PWM_max = 10000;

void setup_pins() {
  DPRINTLN("Setup pins -  HV5122 Nixie driver...");
  DPRINT("- CLK   : GPIO"); DPRINTLN(PIN_CLK);
  DPRINT("- DATAIN: GPIO"); DPRINTLN(PIN_DIN);
  DPRINT("- OUTPUT_ENABLE: GPIO"); DPRINTLN(PIN_OE);
  pinMode(PIN_CLK, OUTPUT);  regPin(PIN_CLK, "PIN_CLK");
  pinMode(PIN_DIN, OUTPUT);  regPin(PIN_DIN, "PIN_DIN");
  pinMode(PIN_OE, OUTPUT);   regPin(PIN_OE, "PIN_OE");
  digitalWrite(PIN_CLK, HIGH);
  digitalWrite(PIN_OE, LOW);
  driverSetupStr = "<br>HV5122 pin settings:<br>";
  for (int i=0;i<maxDigits+1;i++) {
    driverSetupStr += String("{");
    for (int j=0;j<10;j++) {
      driverSetupStr += String(digitPins[i][j]); 
      driverSetupStr += (j==9) ? "}," : ",";
    }
    driverSetupStr += String("<br>");
  }
  clearTubes();
  startTimer();
}


#if defined(ESP32)
void IRAM_ATTR writeDisplay() { //void IRAM_ATTR  writeDisplay(){
#else
void ICACHE_RAM_ATTR writeDisplay() {       //https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
#endif
  static volatile int timer = PWMrefresh;
  static volatile boolean state=true;
  static volatile int brightness;
  static int PWMtimeBrightness=PWM_min;

  intCounter++;
  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    #if defined(ESP8266)
      timer1_write(PWMrefresh);
    #endif  
    return;
  }
  
  #if defined(ESP32)
    portENTER_CRITICAL_ISR(&timerMux);
    noInterrupts();
  #endif

  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  if (autoBrightness && displayON) {
    PWMtimeBrightness = max(PWM_min, (PWM_max * lx * brightness) / (MAXIMUM_LUX * MAXBRIGHTNESS));
  }
  else
    PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
  
  if ((!autoBrightness) && (brightness==MAXBRIGHTNESS))  
    state = true;
  
  if (state) {  //ON state
    timer = PWMtimeBrightness;
    timerON = timer;
  }
  else {  //OFF state
    timer = PWMrefresh-PWMtimeBrightness;
    timerOFF = timer;  
  }
  if (timer<500) timer = 500;  //safety only...
  
  if ( (brightness == 0) || (!state) || (!radarON)) {  //OFF state, blank digit
    digitalWrite(PIN_OE, LOW); //OFF
    }
  else {  //ON state
    digitalWrite(PIN_OE, HIGH);   //ON
  }
  
  state = !state; 
   
#if defined(ESP8266)
  timer1_write(timer);
#elif defined(ESP32)
  portEXIT_CRITICAL_ISR(&timerMux);
  //ESP32timer = timerBegin(0, PRESCALER, true);  //set prescaler, true = edge generated signal
  //timerAttachInterrupt(ESP32timer, &writeDisplay, true);   
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
#endif
}

#if defined(ESP32)
void IRAM_ATTR shift(uint32_t Data) {
#else
void ICACHE_RAM_ATTR shift(uint32_t Data) {
#endif
  static boolean b;

  for (uint32_t i = 0; i < 32; i++) {
    digitalWrite(PIN_CLK, HIGH);
    for (int t=0;t<20;t++) asm volatile ("nop");
    if (SHIFT_LSB_FIRST)
      b = ((Data & (uint32_t(1) << i))) > 0;  //LSB first
    else
      b = (Data & (uint32_t(1) << (31 - i))) > 0; //MSB first
    digitalWrite(PIN_DIN, b);
    for (int t=0;t<20;t++) asm volatile ("nop");
    digitalWrite(PIN_CLK, LOW);  //falling CLK  to store DIN
    for (int t=0;t<20;t++) asm volatile ("nop");
  }
  digitalWrite(PIN_CLK, HIGH);
  for (int t=0;t<20;t++) asm volatile ("nop");
}

void clearTubes() {
  shift(0);
  shift(0);
}

void writeDisplaySingle() {
  static unsigned long lastRun = 0;
  uint32_t bitBuffer0 = 0;
  uint32_t bitBuffer1 = 0;
  byte num;

  if ((millis() - lastRun) < 50) return; //slow down!!!
  lastRun = millis();
 
  bitBuffer0 = 0; bitBuffer1 = 0;
  for (int i = 0; i < maxDigits; i++) { //Set the clock digits
    num = digit[i];
    if (num < 10) {
      if (digitPins[i][num] < 100) {
        bitBuffer0 |= (uint32_t)(1 << (digitPins[i][num]-1)); 
        //DPRINT(digitPins[i][num]); DPRINT(":");
        } //chip0
      else {
        bitBuffer1 |= (uint32_t)(1 << (digitPins[i][num] - 101)); 
        //DPRINT(digitPins[i][num]);DPRINT(":");} //chip1
      }
    }
  }  //end for i

  #ifdef  MAKE_BLINKING_DOTS //it means, the extra datapins are used as 4 blinking dot instead of decimal points!
    if (showClock) {
      digitDP[1] = digitDP[2]; //left two dots
      digitDP[3] = digitDP[4]; //right two dots
    }
  #endif
  
  for (int i = 0; i < maxDigits-1; i++) { //Set the extra decimal point dots
    if (digitDP[i] && digitPins[maxDigits][i]>0) {
      if (digitPins[maxDigits][i] < 100) 
        bitBuffer0 |= (uint32_t)(1 << (digitPins[maxDigits][i]-1)); //chip0
      else
        bitBuffer1 |= (uint32_t)(1 << (digitPins[maxDigits][i] - 101)); //chip1
    }
  }  //end for i

  //bitBuffer0 = 1<<cnt;  bitBuffer1 = 1<<cnt;  cnt++;  if (cnt>32) cnt = 0;  DPRINTLN(cnt);
  shift(bitBuffer1);
  //delayMicroseconds(1);   //for testing on oscilloscope
  shift(bitBuffer0);
  //showBits(bitBuffer1);  showBits(bitBuffer0);   DPRINTLN(" ");
}

//For testing -------------------
/*
void test() {
  digitalWrite(PIN_OE, HIGH);
  clearTubes();
  for (int j=0;j<2;j++)
    for (uint32_t i=0;i<32;i++) {
      DPRINT("D"); DPRINT(i+1); DPRINT(": ");
      shift((uint32_t)(1 << (i))); 
      DPRINTLN((uint32_t)(1 << (i)),HEX);
      delay(3000);
    }
}

void testTubes2() {
while (true) {
   
  DPRINTLN("Testing tubes: ");
  for (int i = 0; i < 10; i++) {
    DPRINT(i); DPRINT(" ");
    for (int j = 0; j < maxDigits; j++) {
      digit[j] = i;
      digitDP[j] = i % 2;
    }
    //changeDigit();
 
    writeDisplaySingle();
    delay(3000);
    yield();
  }
  }  //end while
}

void showBits(uint32_t bits) {
  boolean b;

  for (uint32_t i = 0; i < 32; i++) {
     if (SHIFT_LSB_FIRST)
      b = ((bits & (uint32_t(1) << i))) > 0;  //LSB first
    else
      b = (bits & (uint32_t(1) << (31 - i))) > 0; //MSB first
    if (b) {
      DPRINT("1");
    }
    else {
      DPRINT("0");
    }
  }  //end for
  DPRINT("-");
}

*/
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\HV5122new.ino"
#ifdef NEWHV5122  //Nixie driver
//P.S. 6x tubes PCB version

char tubeDriver[] = "HV5122NEW";
//HV5122 control pins
//#define PIN_DIN  17   // DataIn  - chip0 DOUT pin is connected to chip1 DIN pin!
//#define PIN_CLK  22   // Clock
//#define PIN_OE   21   // OutputEnable

//int maxDigits = 6;

//Data pin numbers 100+ means: chip1 pins are used. Chip0-s DOUT is connected to chip1's DIN
/* Example:
  byte digitPins[maxDigits+1][10] = {
    {2,10,9,8,7,6,5,4,3,1},                      //sec  1 , chip0  (tube#0)
    {11,32,20,19,18,17,16,15,14,13},             //sec  10 , chip0 (tube#1)
    {22,31,29,30,27,28,25,26,23,24},             //min   1 , chip0 (tube#2)
    {101,131,110,109,108,107,106,105,104,103},   //min  10 , chip1 (tube#3)
    {111,132,120,119,118,117,116,115,114,113},   //hour  1 , chip1 (tube#4)
    {122,129,130,127,128,125,126,123,124,121},   //hour 10 , chip1 (tube#5)
    {0,12,21,102,112,0,    0,0,0,0}              //extra decimal point (tube0...tube6)
    };   
*/

uint32_t  DRAM_ATTR bitBuffer0 = 0;      //new digits
uint32_t  DRAM_ATTR bitBuffer1 = 0;
uint32_t  DRAM_ATTR oldBitBuffer0 = 0;   //old digits
uint32_t  DRAM_ATTR oldBitBuffer1 = 0;
int DRAM_ATTR animPtr = 0;

#define SHIFT_LSB_FIRST false  //true= LSB first, false= MSB first

//Brightness PWM timing. Greater value => slower brightness PWM frequency  
// Suggested values: 100000 / 20000, for faster PWM: 50000 / 10000
uint32_t DRAM_ATTR PWMrefresh = 100000; 
uint32_t DRAM_ATTR PWM_min = 6000;

uint32_t DRAM_ATTR time1 = 2000;
uint32_t DRAM_ATTR time2 = 2000; 
uint32_t DRAM_ATTR offTime = 2000;
uint32_t DRAM_ATTR brightness = 0;
uint32_t DRAM_ATTR PWMtimeBrightness;
  
#if defined(ESP32)
void IRAM_ATTR shift(uint32_t Data) {
#else
void ICACHE_RAM_ATTR shift(uint32_t Data) {
#endif
  static boolean b;

  for (uint32_t i = 0; i < 32; i++) {
    digitalWrite(PIN_CLK, HIGH);
    for (int t=0;t<20;t++) asm volatile ("nop");
    if (SHIFT_LSB_FIRST)
      b = ((Data & (uint32_t(1) << i))) > 0;  //LSB first
    else
      b = (Data & (uint32_t(1) << (31 - i))) > 0; //MSB first
    digitalWrite(PIN_DIN, b);
    for (int t=0;t<20;t++) asm volatile ("nop");
    digitalWrite(PIN_CLK, LOW);  //falling CLK  to store DIN
    for (int t=0;t<20;t++) asm volatile ("nop");
  }
  digitalWrite(PIN_CLK, HIGH);
  for (int t=0;t<20;t++) asm volatile ("nop");
}

void clearTubes() {
  shift(0);
  shift(0);
}

void setup_pins() {
  DPRINTLN("Setup pins -  HV5122 Nixie driver...");
  DPRINT("- CLK   : GPIO"); DPRINTLN(PIN_CLK);
  DPRINT("- DATAIN: GPIO"); DPRINTLN(PIN_DIN);
  DPRINT("- OUTPUT_ENABLE: GPIO"); DPRINTLN(PIN_OE);
  pinMode(PIN_CLK, OUTPUT);  regPin(PIN_CLK, "PIN_CLK");
  pinMode(PIN_DIN, OUTPUT);  regPin(PIN_DIN, "PIN_DIN");
  pinMode(PIN_OE, OUTPUT);   regPin(PIN_OE, "PIN_OE");
  digitalWrite(PIN_CLK, HIGH);
  digitalWrite(PIN_OE, LOW);
  driverSetupStr = "<br>HV5122 pin settings:<br>";
  for (int i=0;i<maxDigits+1;i++) {
    driverSetupStr += String("{");
    for (int j=0;j<10;j++) {
      driverSetupStr += String(digitPins[i][j]); 
      driverSetupStr += (j==9) ? "}," : ",";
    }
    driverSetupStr += String("<br>");
  }
  clearTubes();
  startTimer();
}


#if defined(ESP32)
void IRAM_ATTR writeDisplay() { //void IRAM_ATTR  writeDisplay(){
#else
void ICACHE_RAM_ATTR writeDisplay() {       //https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
#endif
  
  static DRAM_ATTR byte state=0;
  int timer = PWMrefresh;

  intCounter++;
  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    #if defined(ESP8266)
      timer1_write(PWMrefresh);
    #endif  
    return;
  }
  
  #if defined(ESP32)
    portENTER_CRITICAL_ISR(&timerMux);
    noInterrupts();
  #endif
  digitalWrite(PIN_OE, LOW); //OFF
  switch (state) {   //state machine...
    case 0:   //show old character
      if (time1>=1000) {
        timer = time1;
        shift(oldBitBuffer1);
        shift(oldBitBuffer0);
        state = 1;
        break;
      }
    case 1:  //show new character
      if (time2>=1000) {
        timer = time2;
        shift(bitBuffer1);
        shift(bitBuffer0);
        if (offTime<500) state = 0;      
        else state = 2;
        break;
      }
    case 2:  //blank display
      timer = offTime;
      state = 3;
      break;
   }  //end switch
 
  if (timer<500) timer = 500;  //safety only...
  
  if ( (state==3) || (!radarON) || (brightness == 0) ) {  //OFF state, blank digit
    digitalWrite(PIN_OE, LOW); //OFF
    state = 0;
    }
  else {  //ON state
    digitalWrite(PIN_OE, HIGH);   //ON
  }
   
#if defined(ESP8266)
  timer1_write(timer);
#elif defined(ESP32)
  portEXIT_CRITICAL_ISR(&timerMux);
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
#endif
}

void writeDisplaySingle() {
  static byte lastAnimMask[BUFSIZE];
  static byte lastDigit[BUFSIZE];
  static unsigned long lastRun = 0;  
  uint32_t b0,b1,ob0,ob1;  //temp buffers
  byte num;

  if ( (memcmp(digit,lastDigit,maxDigits) == 0) && (memcmp(lastDigit,newDigit,maxDigits) == 0) && (memcmp(animMask,lastAnimMask,maxDigits)==0) ) return;
  //if ((millis()-lastRun)<5) return;
  lastRun = millis();

  memcpy(lastAnimMask,animMask,maxDigits);
  memcpy(lastDigit,digit,maxDigits);

  animPtr = 0;
  b0 = 0; b1 = 0; 
  for (int i = 0; i < maxDigits; i++) { //Set the clock digits new values
    num = digit[i];
    if (animMask[i]>0) {
      animPtr = animMask[i];
      num = newDigit[i];
    }
    if (num < 10) {
      if (digitPins[i][num] < 100) {
        b0 |= (uint32_t)(1 << (digitPins[i][num]-1)); 
        //DPRINT(digitPins[i][num]); DPRINT(":");
        } //chip0
      else {
        b1 |= (uint32_t)(1 << (digitPins[i][num] - 101)); 
        //DPRINT(digitPins[i][num]);DPRINT(":");} //chip1
      }
    }
  }  //end for i

if (animPtr>0) {  
  ob0 = 0; ob1 = 0;
  for (int i = 0; i < maxDigits; i++) { //Set the clock digits old values
    num = oldDigit[i];
    if (num < 10) {
      if (digitPins[i][num] < 100) {
        ob0 |= (uint32_t)(1 << (digitPins[i][num]-1)); 
        //DPRINT(digitPins[i][num]); DPRINT(":");
        } //chip0
      else {
        ob1 |= (uint32_t)(1 << (digitPins[i][num] - 101)); 
        //DPRINT(digitPins[i][num]);DPRINT(":");} //chip1
      }
    }
  }  //end for i
}

  #ifdef  MAKE_BLINKING_DOTS //it means, the extra datapins are used as 4 blinking dot instead of decimal points!
    if (showClock) {
      digitDP[1] = digitDP[2]; //left two dots
      digitDP[3] = digitDP[4]; //right two dots
    }
  #endif
  
  for (int i = 0; i < maxDigits-1; i++) { //Set the extra decimal point dots
    if (digitDP[i] && digitPins[maxDigits][i]>0) {
      if (digitPins[maxDigits][i] < 100) {
        b0  |= (uint32_t)(1 << (digitPins[maxDigits][i]-1)); //chip0
        ob0 |= (uint32_t)(1 << (digitPins[maxDigits][i]-1)); //chip0
      }
      else {
        b1  |= (uint32_t)(1 << (digitPins[maxDigits][i] - 101)); //chip1
        ob1 |= (uint32_t)(1 << (digitPins[maxDigits][i] - 101)); //chip1
      }
    }
  }  //end for i



  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  if (autoBrightness && displayON) 
    PWMtimeBrightness = max(PWM_min, (PWMrefresh * lx * brightness) / (MAXIMUM_LUX * MAXBRIGHTNESS));
  else
    PWMtimeBrightness = max(PWM_min, (PWMrefresh*brightness)/MAXBRIGHTNESS);
   
  if ((animPtr==0) || (PWMtimeBrightness<15000)) {  //no animation #5
    ob0 = b0;
    ob1 = b1;
    animPtr = 10;  //to make equal timing
  }

  time1 = PWMtimeBrightness * (20-animPtr)/20;
  if (time1<1000) time1=0;
  if (time1 > (PWMtimeBrightness-1000)) time1 = PWMtimeBrightness;
  time2 = PWMtimeBrightness - time1;
  offTime = PWMrefresh - time1 - time2;
  
  bitBuffer0 = b0;
  bitBuffer1 = b1;
  oldBitBuffer0 = ob0;
  oldBitBuffer1 = ob1;

//  DPRINT("Br:"); DPRINT(brightness);  DPRINT(" A:"); DPRINT(animPtr);  
//  DPRINT(" PTB"); DPRINT(PWMtimeBrightness);
//  DPRINT(" t1:"); DPRINT(time1); DPRINT(" t2:"); DPRINT(time2); DPRINT(" off:"); DPRINT(offTime);
//  DPRINT(" "); DPRINT(b0,HEX); DPRINT(" ");   DPRINT(b1,HEX); DPRINT(" "); 
//  DPRINT(ob0,HEX); DPRINT(" ");   DPRINTLN(ob1,HEX);   

} 


//For testing -------------------
/*
void test() {
  digitalWrite(PIN_OE, HIGH);
  clearTubes();
  for (int j=0;j<2;j++)
    for (uint32_t i=0;i<32;i++) {
      DPRINT("D"); DPRINT(i+1); DPRINT(": ");
      shift((uint32_t)(1 << (i))); 
      DPRINTLN((uint32_t)(1 << (i)),HEX);
      delay(3000);
    }
}

void testTubes2() {
while (true) {
   
  DPRINTLN("Testing tubes: ");
  for (int i = 0; i < 10; i++) {
    DPRINT(i); DPRINT(" ");
    for (int j = 0; j < maxDigits; j++) {
      digit[j] = i;
      digitDP[j] = i % 2;
    }
    //changeDigit();
 
    writeDisplaySingle();
    delay(3000);
    yield();
  }
  }  //end while
}

void showBits(uint32_t bits) {
  boolean b;

  for (uint32_t i = 0; i < 32; i++) {
     if (SHIFT_LSB_FIRST)
      b = ((bits & (uint32_t(1) << i))) > 0;  //LSB first
    else
      b = (bits & (uint32_t(1) << (31 - i))) > 0; //MSB first
    if (b) {
      DPRINT("1");
    }
    else {
      DPRINT("0");
    }
  }  //end for
  DPRINT("-");
}

*/
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
//BME280, and AHT20+BMP280 sensor drivers
#ifdef USE_I2CSENSORS

//#define USE_BME280
//#define USE_BMP280
//#define USE_AHTX0
//#define USE_SHT21
//#define USE_BH1750

#include <Wire.h>
#include <SPI.h>

#ifdef USE_BME280
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme; // use I2C interface   address = 0x76 or 0x77
  Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
  Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
  Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
  byte BME280tempPtr;
  byte BME280humidPtr;
  byte BME280pressPtr;
#endif

#ifdef USE_BMP280
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp; // use I2C interface   address = 0x76 or 0x77
  Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
  Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
  byte BMP280tempPtr;
  byte BMP280humidPtr;
  byte BMP280pressPtr;
#endif

#ifdef USE_AHTX0
  #include <Adafruit_AHTX0.h>     //I2C address: 0x38
  Adafruit_AHTX0 aht;
  Adafruit_Sensor *aht_humidity;   //, *aht_temp;
  //byte AHTX0tempPtr;
  byte AHTX0humidPtr;
#endif

#ifdef USE_SHT21
  #include <SHT21.h>     //I2C address: 0x80 and 0x81
  SHT21 sht; 
  float SHT21temp,SHT21humid =0;
  byte SHT21tempPtr;
  byte SHT21humidPtr;
#endif

#ifdef USE_BH1750
  #include <BH1750.h>
  BH1750 lightMeter(0x23);
#endif

void setupI2Csensors() {
  DPRINTLN("Starting I2Csensors sensors...");    
  
  pinMode(PIN_SDA,INPUT_PULLUP);  regPin(PIN_SDA,"PIN_SDA");
  pinMode(PIN_SCL,INPUT_PULLUP);  regPin(PIN_SCL,"PIN_SCL");
  Wire.begin(PIN_SDA,PIN_SCL); 
  delay(100);
  I2Cscanner();

#ifdef USE_RTC
  DPRINTLN("Starting RTC Clock...");    
  while(millis()<2000) {yield();}  //waiting for 2 sec from startup
  //I2C_ClearBus();
  delay(100);
  #if defined(PIN_SDA) && defined(PIN_SCL)
    auto probeI2C = [](uint8_t addr) -> byte {
      Wire.beginTransmission(addr);
      return Wire.endTransmission();
    };

    byte error68 = 4;
    for (byte attempt = 0; attempt < 4; attempt++) {
      error68 = probeI2C(0x68);
      if (error68 == 0) break;
      delay(40);
    }
    byte error51 = probeI2C(0x51);

    if (error68 == 0) {   //DS3231 at 0x68
      RTCexist = true;
      RTCisPCF8563 = false;
      DPRINTLN("RTC found on 0x68.");
    }
    else if (error51 == 0) {
      RTCexist = true;
      RTCisPCF8563 = true;
      DPRINTLN("RTC found on 0x51 (PCF8563).");
    }
    else { 
      RTCexist = false;
      RTCisPCF8563 = false;
      DPRINTLN("!!!No RTC found on 0x68!!!");
      byte error57 = probeI2C(0x57);
      if (error57 == 0) {
        DPRINTLN("I2C 0x57 is present (AT24C32 EEPROM on many DS3231 modules). Check RTC wiring/power and SDA/SCL pin mapping.");
      }
      else if (error51 == 0) {
        DPRINTLN("I2C 0x51 is present (likely PCF8563 RTC). This firmware RTC driver expects DS3231 at 0x68.");
      }
      else {
        DPRINTLN("No response on 0x68 or 0x57. Check VCC/GND and configured PIN_SDA/PIN_SCL for this clock profile.");
      }
    }
  #else
    RTCexist = false;
    RTCisPCF8563 = false;
    DPRINTLN("PIN_SDA or PIN_SCL not defined!");
  #endif  
#endif

  
#ifdef USE_BME280  
  if (!bme.begin(0x76)) {
    DPRINTLN("Could not find a valid BME280 sensor on address 0x76, check wiring!");
  }
  else {
    BME280exist = true;
    DPRINTLN("BME280 Found!");
    bme_temp->printSensorDetails();
    bme_pressure->printSensorDetails();
    bme_humidity->printSensorDetails();
    BME280tempPtr = useTemp;  //remember my ID-s
    BME280humidPtr = useHumid;
    BME280pressPtr = usePress;
    useTemp++;   //increase the number of sensors
    useHumid++;
    usePress++;
  }
 #endif

#ifdef USE_BMP280  
  if (!bmp.begin(0x77)) {
    DPRINTLN("Could not find a valid BMP280 sensor on address 0x77, check wiring!");
  }
  else {
    BMP280exist = true;
    /* BMP280 default settings from datasheet. */
    DPRINTLN("BMP280 Found!");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    bmp_temp->printSensorDetails();
    BMP280tempPtr = useTemp;  //remember my ID-s
    BMP280pressPtr = usePress;  //remember my ID-s
    useTemp++;   //increase the number of sensors
    usePress++;   //increase the number of sensors
  }
#endif

#ifdef USE_AHTX0
  if (!aht.begin()) {
    DPRINTLN("Could not find a valid AHT10/AHT20 sensor, check wiring!");
  }
  else {
    AHTX0exist = true;
    DPRINTLN("AHT10/AHT20 Found!");
    //aht_temp = aht.getTemperatureSensor();
    //aht_temp->printSensorDetails();
    aht_humidity = aht.getHumiditySensor();
    aht_humidity->printSensorDetails();
    //AHTX0tempPtr = useTemp;  //remember my ID-s
    AHTX0humidPtr = useHumid;
    //useTemp++;   //increase the number of sensors
    useHumid++;    
  }
#endif  
  DPRINTLN(" ");

#ifdef USE_SHT21
  int error1,error2;
  Wire.beginTransmission(0x40);
  error1 = Wire.endTransmission();
  Wire.beginTransmission(0x80);
  error2 = Wire.endTransmission();
  if ((error1 !=0) && (error2!=0)) {
    DPRINTLN("Could not find a valid SHT21 sensor on 0x40 / 0x80, check wiring!");
  }
  else {
    SHT21exist = true;
    DPRINTLN("SHT21 Found!");
    SHT21tempPtr = useTemp;  //remember my ID-s
    SHT21humidPtr = useHumid;
    useTemp++;   //increase the number of sensors
    useHumid++;    
  }
#endif  

#ifdef USE_BH1750
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    DPRINTLN("BH1750 luxmeter sensor found");
    BH1750exist = true;
    useLux++;
  }
  else {
    DPRINTLN("Error initialising BH1750 on address 0x23");
  }
#endif

  DPRINTLN(" ");
} //end of SetupI2Csensors()


void getBME280() {
#ifdef USE_BME280  
  temperature[BME280tempPtr] = round1(bme.readTemperature());
  humid[BME280humidPtr] = round1(bme.readHumidity());
  pressur[BME280pressPtr] = round1(bme.readPressure()/100);
  DPRINT("BME280: ");
  DPRINT("Temperature = ");
  DPRINT(temperature[BME280tempPtr]);
  DPRINT(" *C");

  DPRINT("  Humidity = ");
  DPRINT(humid[BME280humidPtr]);
  DPRINT(" %");

  DPRINT("  Pressure = ");
  DPRINT(pressur[BME280pressPtr]);
  DPRINTLN(" hPa");
#endif
}


void getBMP280() {
#ifdef USE_BMP280  
  temperature[BMP280tempPtr] = round1(bmp.readTemperature());
  pressur[BMP280pressPtr] = round1(bmp.readPressure()/100);
  DPRINT("BMP280: ");
  DPRINT(F("Temperature = "));
  DPRINT(temperature[BMP280tempPtr]);
  DPRINT(" *C");
  
  DPRINT(F("  Pressure = "));
  DPRINT(pressur[BMP280pressPtr]);
  DPRINTLN(" hPa");
#endif
}

void getAHTX0() {
#ifdef USE_AHTX0
  //  /* Get a new normalized sensor event */
  sensors_event_t humidity;
  //sensors_event_t temp;
  aht_humidity->getEvent(&humidity);
  //aht_temp->getEvent(&temp);
  //temperature[AHTX0tempPtr] = round1(temp.temperature);
  DPRINT("AHT sensor:  ");
  //DPRINT("  Temperature ");
  //DPRINT(temperature[AHTX0tempPtr]);
  //DPRINT(" deg C");
  
  DPRINT("  Humidity: ");  /* Display the results (humidity is measured in % relative humidity (% rH) */
  humid[AHTX0humidPtr] = round1(humidity.relative_humidity);
  DPRINT(humid[AHTX0humidPtr]);
  DPRINTLN(" % rH");
#endif
}

void getSHT21() {
#ifdef USE_SHT21
  temperature[SHT21tempPtr] = round1(sht.getTemperature());
  DPRINT("SHT21 sensor:  ");
  DPRINT("  Temperature ");
  DPRINT(temperature[SHT21tempPtr]);
  DPRINT(" deg C");
  
  DPRINT("  Humidity: ");  /* Display the results (humidity is measured in % relative humidity (% rH) */
  humid[SHT21humidPtr] = round1(sht.getHumidity());
  DPRINT(humid[SHT21humidPtr]);
  DPRINTLN(" % rH");
#endif
}



void getI2Csensors() {
  static unsigned long lastRun = 0;

  if (((millis()-lastRun)<30000) && (lastRun !=0)) return;  // || (second()!=TEMP_START)
  lastRun = millis();  
  if (BME280exist) getBME280();
  if (BMP280exist) getBMP280();
  if (AHTX0exist) getAHTX0();
  if (SHT21exist) getSHT21();
  if (BH1750exist) getBH1750();
}


void I2Cscanner() {
  byte error, address;
  int nDevices=0;
  DPRINTLN(F("________________________________"));
  DPRINTLN("Scanning I2C devices...");

  for(address = 1; address < 127; address++ )   {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)  {
      DPRINT("I2C device found at address 0x");
      if (address<16) 
        DPRINT("0");
      DPRINT(address,HEX);
      DPRINTLN("  !");

      nDevices++;
    }
    else if (error==4) {
      DPRINT("Unknown error at address 0x");
      if (address<16) 
        DPRINT("0");
      DPRINTLN(address,HEX);
    }    
  }  //end for
  
  if (nDevices == 0)
    DPRINTLN("No I2C devices found\n");
  DPRINTLN(F("_________________________________"));
}


#else
void setupI2Csensors() {}
void getI2Csensors() {}
#endif

#ifdef USE_BH1750
int getBH1750() {
  static float oldLux = MAXIMUM_LUX;
  
  if (lightMeter.measurementReady()) {
    float lux = lightMeter.readLightLevel();
    //DPRINT("BH1750 Light: "); DPRINT(lux); DPRINTLN(" lx");
    
    if (lux>MAXIMUM_LUX) lux = MAXIMUM_LUX;   //Limited //Limit lux value to maximum
    if (lux<0) lux = oldLux;
    oldLux = oldLux + (lux-oldLux)/10;   //slow down Lux change
  }
  return (int)oldLux;
}
#else
int getBH1750() {
  return(0);
}  
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MAX6921.ino"
#ifdef MAX6921   
//VFD driver driver for ESP8266
char tubeDriver[] = "MAX6921";
//------------------abcdefgDP----------------   definition of different characters
byte charDefinition[] = {
                   B11111100,   //0: abcdef
                   B01100000,   //1: bc 
                   B11011010,   //2: abdeg
                   B11110010,   //3: abcdg
                   B01100110,   //4: bcfg
                   B10110110,   //5: acdfg
                   B10111110,   //6: acdefg
                   B11100000,   //7: abc
                   B11111110,   //8: abcdefg
                   B11110110,   //9: abcdfg
                   B00000000,   // : BLANK (10)
                   B00000010,   //-: minus (11)
                   B00000001,   // decimal point (12)
                   B11101110,   // A  abcefg (13)
                   B11001110,   // P  abefg (14)
                   B10011100,   // C  adef (15)
                   B11000110,   //grad (upper circle) abfg (16)
                   B10110100,   //%  acdf  (17)
                   B00111010,   //lower circle cdeg  (18)
                   B01100000,   //I  bc    (19)
                   B10001110    //F  aefg  (20)

};

uint32_t animationMaskBits[5];

#define MAXCHARS sizeof(charDefinition)
#define MAXSEGMENTS sizeof(segmentEnablePins)
int maxDigits =  sizeof(digitEnablePins);

uint32_t charTable[MAXCHARS];              //generated pin table from segmentDefinitions
uint32_t segmentEnableBits[MAXSEGMENTS];   //bitmaps, generated from EnablePins tables
uint32_t digitEnableBits[10];

int PWMrefresh=5500;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
int PWM_min = 500;
int PWM_max = 5000;

//int PWMtiming[11] = {0,500,800,1200,2000,2500,3000,3500,4000,4500,5000};
//-----------------------------------------------------------------------------------------

//https://sub.nanona.fi/esp8266/timing-and-ticks.html
//One tick is 1us / 80 = 0.0125us = 12.5ns on 80MHz
//One tick is 1us / 80 = 0.0125us = 6.25ns on 160MHz
// 1 NOP is 1 tick


void inline delayMS(int d) {
    //for (int i=0;i<d;i++) 
      asm volatile ("nop"); 
}


void setup_pins() {
  
  #if defined(ESP8266)
  #else
    #error "Board is not supported! For ESP32 use MAX6921_ESP32 !"  
  #endif
  
  DPRINTLN("VFD Clock - setup MAX6921 pins");
  pinMode(PIN_LE,  OUTPUT);  regPin(PIN_LE,"PIN_LE");
  pinMode(PIN_BL,  OUTPUT);  regPin(PIN_BL,"PIN_BL");
  digitalWrite(PIN_BL,LOW);  //brightness
  pinMode(PIN_DATA,OUTPUT);  regPin(PIN_DATA,"PIN_DATA");
  pinMode(PIN_CLK, OUTPUT);  regPin(PIN_CLK,"PIN_CLK");
  
  driverSetupStr = "MAX6921 segmentEnablePins:";
  for (int j=0;j<sizeof(segmentEnablePins);j++) {
    driverSetupStr += String(segmentEnablePins[j]) + ",";
  }
  driverSetupStr += "<br>MAX6921 digitEnablePins:";
  for (int j=0;j<sizeof(digitEnablePins);j++) {
    driverSetupStr += String(digitEnablePins[j]) + ",";
  }
  driverSetupStr += String("<br>");
  
  generateBitTable();
  digitsOnly = false;
  startTimer();
}  

void ICACHE_RAM_ATTR writeDisplay(){        //https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
  static volatile int timer = PWMrefresh;
  static volatile uint32_t val;
  static volatile byte pos = 0;
  static volatile boolean state=true;
  static volatile int brightness;
  static int PWMtimeBrightness=PWM_min;

  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
      digitalWrite(PIN_BL,HIGH);    //OFF
      timer1_write(PWMrefresh);
    return;
  }
  
  intCounter++;
  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  if ((!autoBrightness) && (brightness==MAXBRIGHTNESS)) state = true;

  if (state) {  //ON state
    pos++;  if (pos>maxDigits-1)  {   //go to the tube#0
      pos = 0;  //go to the first tube
      if (autoBrightness && displayON) {   //change brightness only on the tube#0
        PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
        }
      else
        PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
    }
    val = (digitEnableBits[pos] | charTable[digit[pos]]);  //the full bitmap to send to MAX chip
    if (digitDP[pos]) val = val | charTable[12];    //Decimal Point
    
    timer = PWMtimeBrightness;
    #ifdef CLOCK_4
      if (pos==2) timer = 3*timer;  //Weak IV11 tube#2 brightness compensation, some hacking
    #endif  
    timerON = timer;
    timerOFF = PWMrefresh-PWMtimeBrightness;  
  }
  else {  //OFF state
    timer = PWMrefresh-PWMtimeBrightness;
  }

  if (timer<500) timer = 500;  //safety only...

  if ( (brightness == 0) || (!state) || (!radarON)) {  //OFF state, blank digit
    digitalWrite(PIN_BL,HIGH);    //OFF
    }
  else {  //ON state

    if (animMask[pos]>0) val &= animationMaskBits[animMask[pos]-1];  //animationMode 6, mask characters from up to down and back
    for (int i=0; i<20; i++)  {
      if (val & uint32_t(1 << (19 - i))) {
        digitalWrite(PIN_DATA, HIGH);   
        //asm volatile ("nop");
        }
      else {
        digitalWrite(PIN_DATA, LOW);    
        //asm volatile ("nop");
        }
      
      digitalWrite(PIN_CLK,HIGH);  
      //asm volatile ("nop");  //delayMS(1);
      digitalWrite(PIN_CLK,LOW);   
      //asm volatile ("nop"); //delayMS(1);
      } //end for      
 
    digitalWrite(PIN_LE,HIGH );  asm volatile ("nop");
    digitalWrite(PIN_LE,LOW);
    digitalWrite(PIN_BL,LOW );   //ON
  }  //end else

  state = !state;  
  timer1_write(timer);
}

void generateBitTable() {
uint32_t out;

  DPRINTLN("--- Generating segment pins bitmap ---");
  for (int i=0;i<MAXSEGMENTS;i++) {
    segmentEnableBits[i] = uint32_t(1<<segmentEnablePins[i]);
    //DPRINT(i); DPRINT(": "); DPRINTLN(segmentEnableBits[i],BIN);
  }
  animationMaskBits[0] = uint32_t(1<<segmentEnablePins[0]);  //a
  animationMaskBits[1] = animationMaskBits[0] | uint32_t(1<<segmentEnablePins[1]) | uint32_t(1<<segmentEnablePins[5]);  //bf
  animationMaskBits[2] = animationMaskBits[1] | uint32_t(1<<segmentEnablePins[6]);  //g
  animationMaskBits[3] = animationMaskBits[2] | uint32_t(1<<segmentEnablePins[4]) | uint32_t(1<<segmentEnablePins[2]);  //ec
  animationMaskBits[4] = animationMaskBits[3] | uint32_t(1<<segmentEnablePins[3]);  //d
  for (int i=0;i<5;i++) {
    animationMaskBits[i] = ~animationMaskBits[i]; //invert bits
    //DPRINTLN(animationMaskBits[i],HEX);
  }
  DPRINTLN("--- Generating digit pins bitmap ---");
  for (int i=0;i<maxDigits;i++) {
    digitEnableBits[i] = uint32_t(1 << digitEnablePins[i]);
    //DPRINT(i); DPRINT(": "); DPRINTLN( digitEnableBits[i],BIN);
  }

DPRINTLN("---- Generated Character / Pins table -----");
  for (int i=0;i<MAXCHARS;i++) {
    out = 0;
    //DPRINT(i); DPRINT(":  ");
    //DPRINT(charDefinition[i],BIN);  //DPRINT(" = ");
    for (int j=0;j<=7;j++)   //from a to g
      if ((charDefinition[i] & 1<<(7-j)) != 0) {
        out = out | segmentEnableBits[j]; //DPRINT("1"); 
        }
    //else        DPRINT("0");
    //DPRINT("  >> ");  
    
    charTable[i] = out;
    //DPRINTLN(charTable[i],BIN);
  }  //end for
}


void clearTubes() {
  digitalWrite(PIN_BL,HIGH);
}

void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MAX6921_ESP32.ino"
#ifdef MAX6921_ESP32   
//VFD driver driver for ESP32
  char tubeDriver[] = "MAX6921_ESP32";
  #if defined(ESP32) 
  #else
    #error "Only ESP32 board is supported by this driver!"  
  #endif
  
int DRAM_ATTR maxDig;   //memory variable version of maxDigits
//------------------abcdefgDP----------------   definition of different characters
byte DRAM_ATTR charDefinition[] = {
                   B11111100,   //0: abcdef
                   B01100000,   //1: bc 
                   B11011010,   //2: abdeg
                   B11110010,   //3: abcdg
                   B01100110,   //4: bcfg
                   B10110110,   //5: acdfg
                   B10111110,   //6: acdefg
                   B11100000,   //7: abc
                   B11111110,   //8: abcdefg
                   B11110110,   //9: abcdfg
                   B00000000,   // : BLANK (10)
                   B00000010,   //-: minus (11)
                   B00000001,   // decimal point (12)
                   B11101110,   // A  abcefg (13)
                   B11001110,   // P  abefg (14)
                   B10011100,   // C  adef (15)
                   B11000110,   //grad  (upper circle) abfg (16)
                   B10110100,   //%  acdf  (17)
                   B00111010,   //lower circle cdeg  (18)                  
                   B01100000,   //I  bc    (19)
                   B10001110    //F  aefg  (20)
};

uint32_t DRAM_ATTR animationMaskBits[5];

#define MAXCHARS sizeof(charDefinition)
#define MAXSEGMENTS sizeof(segmentEnablePins)
int maxDigits =  sizeof(digitEnablePins);

uint32_t DRAM_ATTR charTable[MAXCHARS];              //generated pin table from segmentDefinitions
uint32_t DRAM_ATTR segmentEnableBits[MAXSEGMENTS];   //bitmaps, generated from EnablePins tables
uint32_t DRAM_ATTR digitEnableBits[10];

int DRAM_ATTR PWMrefresh=5500;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
int DRAM_ATTR PWM_min = 500;
int DRAM_ATTR PWM_max = 5000;
//int DRAM_ATTR PWMtiming[11] = {0,500,800,1200,2000,2500,3000,3500,4000,4500,5000};

//-----------------------------------------------------------------------------------------

//https://sub.nanona.fi/esp8266/timing-and-ticks.html
//One tick is 1us / 80 = 0.0125us = 12.5ns on 80MHz
//One tick is 1us / 80 = 0.0125us = 6.25ns on 160MHz
// 1 NOP is 1 tick

void inline delayMS(int d) {
  asm volatile ("nop"); 
  asm volatile ("nop");
  asm volatile ("nop");
}


void setup_pins() {
  DPRINTLN("VFD Clock - setup MAX6921 pins");
  pinMode(PIN_LE,  OUTPUT);  regPin(PIN_LE,"PIN_LE");
  pinMode(PIN_BL,  OUTPUT);  regPin(PIN_BL,"PIN_BL");
  digitalWrite(PIN_BL,LOW);  //brightness
  pinMode(PIN_DATA,OUTPUT);  regPin(PIN_DATA,"PIN_DATA");
  pinMode(PIN_CLK, OUTPUT);  regPin(PIN_CLK,"PIN_CLK");
  
  driverSetupStr = "MAX6921 segmentEnablePins:";
  for (int j=0;j<sizeof(segmentEnablePins);j++) {
    driverSetupStr += String(segmentEnablePins[j]) + ",";
  }
  driverSetupStr += "<br>MAX6921 digitEnablePins:";
  for (int j=0;j<sizeof(digitEnablePins);j++) {
    driverSetupStr += String(digitEnablePins[j]) + ",";
  }
  driverSetupStr += String("<br>");
  
  generateBitTable();
  digitsOnly = false;
  startTimer();
}  

void IRAM_ATTR writeDisplay(){  //void IRAM_ATTR  writeDisplay(){
  static DRAM_ATTR int timer = PWMrefresh;
  static DRAM_ATTR uint32_t val;
  static DRAM_ATTR byte pos = 0;
  static DRAM_ATTR boolean state=true;
  static DRAM_ATTR int brightness;
  static DRAM_ATTR int PWMtimeBrightness=PWM_min;

  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
      //digitalWrite(PIN_BL,HIGH);    //OFF
    return;
  }

  portENTER_CRITICAL_ISR(&timerMux);
  noInterrupts();
  intCounter++;

  brightness = displayON ? prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety
  
  if ((!autoBrightness) && (brightness==MAXBRIGHTNESS)) state = true;
  
  if (state) {  //ON state
    pos++;  if (pos>maxDigits-1)  {   //go to the tube#0
      pos = 0; 
      if (autoBrightness && displayON) {   //change brightness only on the tube#0
        PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
        }
      else
        PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
    }  
    
    val = (digitEnableBits[pos] | charTable[digit[pos]]);  //the full bitmap to send to MAX chip
    if (digitDP[pos]) val = val | charTable[12];    //Decimal Point
        
    timer = PWMtimeBrightness;
    //if (pos==2) timer = 3*timer;  //Weak IV11 tube#2 brightness compensation
    timerON = timer;
    timerOFF = PWMrefresh-PWMtimeBrightness;
  }
  else {  //OFF state
    timer = PWMrefresh-PWMtimeBrightness;
  }
  
  if (timer<500) timer = 500;  //safety only...

  if ( (brightness == 0) || (!state) || (!radarON)) {  //OFF state, blank digit
    digitalWrite(PIN_BL,HIGH);    //OFF
  }
  else {  //ON state
    if (animMask[pos]>0) val &= animationMaskBits[animMask[pos]-1];  //animationMode 6, mask characters from up to down and back
    for (int i=0; i<20; i++)  {
      if (val & uint32_t(1 << (19 - i)))
        {digitalWrite(PIN_DATA, HIGH);   //asm volatile ("nop");
        }
      else
        {digitalWrite(PIN_DATA, LOW);    //asm volatile ("nop");
        }
      
      digitalWrite(PIN_CLK,HIGH);  //asm volatile ("nop");  //delayMS(1);
      digitalWrite(PIN_CLK,LOW);   //asm volatile ("nop"); //delayMS(1);
      } //end for      
 
    digitalWrite(PIN_LE,HIGH );  asm volatile ("nop");
    digitalWrite(PIN_LE,LOW);
    digitalWrite(PIN_BL,LOW );   //ON
  }  //end else
  
  portEXIT_CRITICAL_ISR(&timerMux);   
  state = !state;  
  //ESP32timer = timerBegin(0, PRESCALER, true);  //set prescaler, true = edge generated signal
  //timerAttachInterrupt(ESP32timer, &writeDisplay, true);   
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
}

void generateBitTable() {
uint32_t out;

  DPRINTLN("--- Generating segment pins bitmap ---");
  for (int i=0;i<MAXSEGMENTS;i++) {
    segmentEnableBits[i] = uint32_t(1<<segmentEnablePins[i]);
    //DPRINT(i); DPRINT(": "); DPRINTLN(segmentEnableBits[i],BIN);
  }
  animationMaskBits[0] = uint32_t(1<<segmentEnablePins[0]);  //a
  animationMaskBits[1] = animationMaskBits[0] | uint32_t(1<<segmentEnablePins[1]) | uint32_t(1<<segmentEnablePins[5]);  //bf
  animationMaskBits[2] = animationMaskBits[1] | uint32_t(1<<segmentEnablePins[6]);  //g
  animationMaskBits[3] = animationMaskBits[2] | uint32_t(1<<segmentEnablePins[4]) | uint32_t(1<<segmentEnablePins[2]);  //ec
  animationMaskBits[4] = animationMaskBits[3] | uint32_t(1<<segmentEnablePins[3]);  //d
  for (int i=0;i<5;i++) {
    animationMaskBits[i] = ~animationMaskBits[i]; //invert bits
    //DPRINTLN(animationMaskBits[i],HEX);
  }
  DPRINTLN("--- Generating digit pins bitmap ---");
  for (int i=0;i<maxDigits;i++) {
    digitEnableBits[i] = uint32_t(1 << digitEnablePins[i]);
    //DPRINT(i); DPRINT(": "); DPRINTLN( digitEnableBits[i],BIN);
  }

DPRINTLN("---- Generated Character / Pins table -----");
  for (int i=0;i<MAXCHARS;i++) {
    out = 0;
    //DPRINT(i); DPRINT(":  ");
    //DPRINT(charDefinition[i],BIN);  //DPRINT(" = ");
    for (int j=0;j<=7;j++)   //from a to g
      if ((charDefinition[i] & 1<<(7-j)) != 0) {
        out = out | segmentEnableBits[j]; //DPRINT("1"); 
        }
    //else        DPRINT("0");
    //DPRINT("  >> ");  
    
    charTable[i] = out;
    //DPRINTLN(charTable[i],BIN);
  }  //end for
}


void clearTubes() {
  digitalWrite(PIN_BL,HIGH);
}

void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MAX7219CNG.ino"
#ifdef MAX7219CNG
char tubeDriver[] = "MAX7219CNG";
int maxDigits = 6;
byte tubes[] = {0,1,2,3,4,5};    //change it, if needed for the correct tube sequence
byte brightConvert[] = {0,1,2,3,4,6,8,10,12,14,15};   //convert brightness from 0..10 to 0..15

//MAX7219CNG control pins
#define PIN_LOAD 15  // D8 LOAD/CS_
#define PIN_CLK  14  // D5 Clock
#define PIN_DIN  13  // D7 DataIN

//MAX7219 Registers
#define REG_SHUTDOWN   0x0C
#define REG_SCANLIMIT  0x0B  
#define REG_DECODE     0x09 
#define REG_INTENSITY  0x0A 
#define REG_TEST       0x0F 

//------------------abcdefgDP----------------   definition of different characters
byte charDefinition[] = {
                   B11111100,   //0: abcdef
                   B01100000,   //1: bc 
                   B11011010,   //2: abdeg
                   B11110010,   //3: abcdg
                   B01100110,   //4: bcfg
                   B10110110,   //5: acdfg
                   B10111110,   //6: acdefg
                   B11100000,   //7: abc
                   B11111110,   //8: abcdefg
                   B11110110,   //9: abcdfg
                   B00000000,   // : BLANK   (10)
                   B00000010,   //-: minus (11)
                   B00000001,   // decimal point (12)
                   B11101110,   // A  abcefg  (13)
                   B11001110,   // P  abefg (14)
                   B10011100,   // C  adef (15)
                   B11000110,   //grad  (upper circle) abfg  (16)         
                   B10110100,   //%  acdf  (17)    
                   B00111010,   //lower circle cdeg  (18)                   
                   B01100000,   //I  bc    (19)
                   B10001110    //F  aefg  (20)                         
};

//Fill this table with the OUT bits numbers of MAX7219 chip!   I use only No-Decode mode for flexible hardware
byte segmentEnablePins[] =  {6,5,4,3,2,1,0,7};   //segment enable bits of MAX7219 (a,b,c,d,e,f,g,DP)   (You MUST define always 8 bits!!!)

#define MAXCHARS sizeof(charDefinition)
#define MAXSEGMENTS 8

void ICACHE_RAM_ATTR sendBits(byte address,byte val){ 
  digitalWrite(PIN_LOAD, LOW);
  shiftOut(PIN_DIN, PIN_CLK, MSBFIRST, address);
  shiftOut(PIN_DIN, PIN_CLK, MSBFIRST, val);
  digitalWrite(PIN_LOAD, HIGH);
  delay(3);
  //DPRINT(address,HEX); DPRINT(","); DPRINTLN(val,HEX);
}

void setup_pins() {
#if defined(ESP8266) 
#else
  #error "Only 8266 Board is supported!"
#endif
  
  DPRINTLN("MAX7219 Clock - setup pins...");
  pinMode(PIN_LOAD,OUTPUT);   regPin(PIN_LOAD,"PIN_LOAD");
  pinMode(PIN_DIN, OUTPUT);   regPin(PIN_DIN,"PIN_DIN");
  pinMode(PIN_CLK, OUTPUT);   regPin(PIN_CLK,"PIN_CLK");
  digitsOnly = false; 
  
  sendBits(REG_SHUTDOWN,1);              //Set to Normal (not Shutdown) mode
  sendBits(REG_TEST,0);                  //Set to Normal, (Non-Test) mode
  sendBits(REG_SCANLIMIT,maxDigits-1);   //Set scanlimit 
  sendBits(REG_DECODE,0 );               //Set Non-Decode Mode
  sendBits(REG_INTENSITY,0x0F);          //Set maximum brightness
}

void ICACHE_RAM_ATTR writeDisplaySingle() {
static byte oldBright = 0;
static byte newBright = 0;
int bitBuffer;  

//https://sub.nanona.fi/esp8266/timing-and-ticks.html
//One tick is 1us / 80 = 0.0125us = 12.5ns on 80MHz
//One tick is 1us / 80 = 0.0125us = 6.25ns on 160MHz
// 1 NOP is 1 tick


  newBright = brightConvert[displayON ?  prm.dayBright : prm.nightBright];  //convert 0..10 brightness levels to 0..15
  if (!radarON) newBright = 0;
  if (newBright != oldBright) {
    sendBits(REG_INTENSITY, newBright);    //Set Brightness
    oldBright = newBright;
  }
  for (int i=0;i<maxDigits;i++) {
    bitBuffer = 0;
    for (int j=0;j<=7;j++)   //from a to g
      if ((charDefinition[digit[tubes[i]]] & 1<<(7-j)) != 0) {
        bitBuffer |= 1<<segmentEnablePins[j]; 
        }
    if (digitDP[tubes[i]]) bitBuffer |= 1<<segmentEnablePins[7];   //Decimal Point  
    sendBits(i+1, bitBuffer);
  }  //end for i
}

void clearTubes() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MM5450.ino"
#ifdef MM5450
char tubeDriver[] = "MM5450";
int maxDigits = 6;

//Fill this table with the pin numbers of MM5450 chip!
byte segmentEnablePins[4][8] =  {
  {16, 15, 14, 13, 12, 11, 10, 9}, //segment enable pins of MM5450 (a,b,c,d,e,f,g,DP)
  {8, 7, 6, 5, 4, 3, 2, 1},
  {17, 18, 19, 20, 21, 22, 23, 24},
  {25, 26, 27, 28, 29, 30, 31, 32}
};


/* Old version
  byte segmentEnablePins[4][8] =  {
                 {25,26,27,28,29,30,31,32},   //segment enable pins of MM5450 (a,b,c,d,e,f,g,DP)
                 {17,18,19,20,21,22,23,24},
                 {9,10,11,12,13,14,15,16},
                 {1,2,3,4,5,6,7,8}
                 };
*/


byte sideEnablePins[] = {34, 33}; //segment enable pins of 2x4 LEDS

//MM5450 control pins
#define PIN_LE    14  // D6 Shift Register Latch Enable
#define PIN_CLK   12  // D7 Shift Register Clock
#define PIN_DATA  13  // D5 Shift Register Data
#define PIN_BR    15  // D8 Shift Register Brightness 

#define PIN_LE_BIT   1<<14  // D6 Shift Register Latch Enable
#define PIN_CLK_BIT  1<<12  // D7 Shift Register Clock
#define PIN_DATA_BIT 1<<13  // D5 Shift Register Data
#define PIN_BR_BIT   1<<15  // D8 Shift Register Brightness 

//------------------abcdefgDP----------------   definition of different characters
byte charDefinition[] = {
  B11111100,   //0: abcdef
  B01100000,   //1: bc
  B11011010,   //2: abdeg
  B11110010,   //3: abcdg
  B01100110,   //4: bcfg
  B10110110,   //5: acdfg
  B10111110,   //6: acdefg
  B11100000,   //7: abc
  B11111110,   //8: abcdefg
  B11110110,   //9: abcdfg
  B00000000,   // : BLANK   (10)
  B00000010,   //-: minus (11)
  B00000001,   // decimal point (12)
  B11101110,   // A  abcefg  (13)
  B11001110,   // P  abefg (14)
  B10011100,   // C  adef (15)
  B11000110,   //grad (upper circle) abfg  (16)
  B10110100,   //%  acdf  (17)
  B00111010,   //lower circle cdeg  (18)
  B01100000,   //I  bc    (19)
  B10001110    //F  aefg  (20)
};

#define MAXCHARS sizeof(charDefinition)

int PWMrefresh = 10000; ////msec, Multiplex time period. Greater value => slower multiplex frequency
int PWM_min = 500;
int PWM_max = 10000;
//int PWMtiming[11] = {0,500,800,1200,2000,2500,3000,4500,6000,8000,10000};

byte bitBuffer[36];

void setup_pins() {
#if defined(ESP8266)
#else
#error "Only 8266 Board is supported!"
#endif

  DPRINTLN("MM5450 - Setup pins...");
  pinMode(PIN_LE,  OUTPUT);  regPin(PIN_LE, "PIN_LE");
  pinMode(PIN_BR,  OUTPUT);  regPin(PIN_BR, "PIN_BR");
  pinMode(PIN_DATA, OUTPUT);  regPin(PIN_DATA, "PIN_DATA");
  pinMode(PIN_CLK, OUTPUT);  regPin(PIN_CLK, "PIN_CLK");
  digitalWrite(PIN_BR, HIGH); //brightness

  digitsOnly = false;
  startTimer();

}

void ICACHE_RAM_ATTR writeBits() {
  //https://sub.nanona.fi/esp8266/timing-and-ticks.html
  //One tick is 1us / 80 = 0.0125us = 12.5ns on 80MHz
  //One tick is 1us / 80 = 0.0125us = 6.25ns on 160MHz
  // 1 NOP is 1 tick

  //noInterrupts();
  WRITE_PERI_REG( PIN_OUT_CLEAR, PIN_LE_BIT );   //DATA_ENABLE
  for (int t = 0; t < 5; t++) asm volatile ("nop"); //100nsec

  for (int i = 0; i < 36; i++)  {
    if (bitBuffer[i]) {
      WRITE_PERI_REG( PIN_OUT_SET, PIN_DATA_BIT );  //digitalWrite(PIN_DATA,HIGH);
    }
    else {
      WRITE_PERI_REG( PIN_OUT_CLEAR, PIN_DATA_BIT ); //digitalWrite(PIN_DATA,LOW);
    }

    for (int t = 0; t < 12; t++) asm volatile ("nop"); //300nsec

    WRITE_PERI_REG( PIN_OUT_SET, PIN_CLK_BIT );
    for (int t = 0; t < 12; t++) asm volatile ("nop"); //300nsec
    WRITE_PERI_REG( PIN_OUT_CLEAR, PIN_CLK_BIT );
    for (int t = 0; t < 12; t++) asm volatile ("nop"); //300nsec
  } //end for

  WRITE_PERI_REG( PIN_OUT_SET, PIN_LE_BIT );
  //interrupts();
}

void ICACHE_RAM_ATTR writeDisplay() {       // Writes to the MM5450 driver for LEDS
  static volatile int timer = PWMrefresh;
  static volatile boolean state = true;
  static volatile int brightness;
  static int PWMtimeBrightness;
  static volatile uint32_t val;
  static volatile byte pos = 0;
  static byte side = 0;
  byte num = 0;

  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    timer1_write(PWMrefresh);
    return;
  }

  intCounter++;
  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness > MAXBRIGHTNESS) brightness = MAXBRIGHTNESS; //only for safety

  if (autoBrightness && displayON) {
    PWMtimeBrightness = max(PWM_min, PWM_max * lx / MAXIMUM_LUX);
  }
  else
    PWMtimeBrightness = max(PWM_min, PWM_max * brightness / MAXBRIGHTNESS);

  if ((!autoBrightness) && (brightness == MAXBRIGHTNESS))
    state = true;

  if (state) {  //ON state
    timer = PWMtimeBrightness;
    timerON = timer;
    timerOFF = PWMrefresh - PWMtimeBrightness;
  }
  else {  //OFF state
    timer = PWMrefresh - PWMtimeBrightness;
  }
  if (timer < 500) timer = 500; //safety only...

  if ( (brightness == 0) || (!state) || (!radarON)) {  //OFF state, blank digit
    WRITE_PERI_REG( PIN_OUT_CLEAR, PIN_BR_BIT );    //OFF
  }
  else {  //ON state
    WRITE_PERI_REG( PIN_OUT_SET, PIN_BR_BIT );   //ON

    memset(bitBuffer, 0, sizeof(bitBuffer)); //clear array
    bitBuffer[0] = 1;   //starting bit

    bitBuffer[sideEnablePins[side]] = 0;
    writeBits();
    for (int t = 0; t < 100; t++) asm volatile ("nop"); //clean display and wait between changing te two sides
    bitBuffer[sideEnablePins[side]] = 0;   //disable old side..

    if (side == 0) side = 1; else side = 0;   //change side...

    for (int i = 0; i < 4; i++) {
      pos = 7 - (4 * side + i);
      num = digit[pos];
      //if (animMask[pos]>0) val &= animationMaskBits[animMask[pos]-1];  //animationMode 6, mask characters from up to down and back
      for (int j = 0; j <= 7; j++) //from a to g
        if ((charDefinition[num] & 1 << (7 - j)) != 0) {
          bitBuffer[segmentEnablePins[i][j]] = 1;
        }
      if (digitDP[pos]) bitBuffer[segmentEnablePins[i][7]] = 1;   //Decimal Point
    }  //end for

    bitBuffer[sideEnablePins[side]] = 1;   //enable new side
    writeBits();
  }  //end else ON

  state = !state;
  timer1_write(timer);
}


void clearTubes() {}
void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\Numitron_4511N.ino"
#ifdef Numitron_4511N
char tubeDriver[] = "Numitron_4511N";
//used by GP Numitron v3 panel

//#define LTBIpin 5
//byte digitEnablePins[] = {13,12,14,16};    //define here the digit enable pins from 4 to 8
//byte ABCDPins[4] = {4,0,2,15};

int maxDigits = sizeof(digitEnablePins);

#if defined(ESP32)
  int DRAM_ATTR PWMrefresh=5500;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
  //int DRAM_ATTR PWMtiming[11] = {0,500,800,1200,2000,2500,3000,3500,4000,4500,5000};
  int DRAM_ATTR PWM_min = 500;
  int DRAM_ATTR PWM_max = 5000;  
  int DRAM_ATTR maxDig=maxDigits;
#else
  int PWMrefresh=5500;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
  int PWM_min = 500;
  int PWM_max = 5000;
  int maxDig=maxDigits;
  //int PWMtiming[11] = {0,500,800,1200,2000,2500,3000,3500,4000,4500,5000};
#endif

void setup_pins() {
  char tmp[30];
    
  DPRINTLN("Numitron Clock - setup pins");
  delay(1000);
  pinMode(LTBIpin, OUTPUT);  regPin(LTBIpin,"LTBIpin");
  digitalWrite(LTBIpin,HIGH);
  for (int i=0;i<maxDig;i++) {
    pinMode(digitEnablePins[i], OUTPUT);
    sprintf(tmp,"digitEnablePins[%d]",i);
    regPin(digitEnablePins[i],tmp); 
  }
  for (int i=0;i<4;i++) {
    pinMode(ABCDPins[i], OUTPUT);
    sprintf(tmp,"Pin[%c]",char('A'+i));
    regPin(ABCDPins[i],tmp); 
  }
  startTimer();
}

#if defined(ESP32)
void IRAM_ATTR writeDisplay() { //void IRAM_ATTR  writeDisplay(){
  static DRAM_ATTR volatile int timer = PWMrefresh;
  static DRAM_ATTR volatile int dpCounter = 0; //decimal point stepper
  static DRAM_ATTR boolean state=true;
  int PWMtimeBrightness;
  int brightness;
  int num;

  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    return;
  }
  
  portENTER_CRITICAL_ISR(&timerMux);
  noInterrupts();
  intCounter++;
  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  if (autoBrightness && (displayON )) {   
    PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
    }
  else
    PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
  
  if ((!autoBrightness) && (brightness==MAXBRIGHTNESS))  
    state = true;
  
  if (state) {  //ON state
    timer = PWMtimeBrightness;
    timerON = timer;
    timerOFF = PWMrefresh-PWMtimeBrightness; 
  }
  else {  //OFF state
    timer = PWMrefresh-PWMtimeBrightness;
  }
  if (timer<500) timer = 500;  //safety only...
  if ( (brightness == 0) || (!state) || (!radarON)) {  //OFF state, blank digit
        digitalWrite(LTBIpin,LOW); //disable display
        #if COLON_PIN >=0  
          digitalWrite(COLON_PIN,LOW);  // Blink colon pin
        #endif
    }
  else {  //ON state
        digitalWrite(LTBIpin,HIGH); //enable display
        #if COLON_PIN >=0  
          digitalWrite(COLON_PIN,colonBlinkState);
        #endif
  }
  state = !state;  
  portEXIT_CRITICAL_ISR(&timerMux);
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
}

void IRAM_ATTR writeDisplaySingle() {
  int num;

  for (int pos=0; pos<=maxDig; pos++) {
    num = digit[pos]; 
    digitalWrite(digitEnablePins[pos],LOW);   //latch enable 
    for (int j=0;j<4;j++) {digitalWrite(ABCDPins[j],num  & (1<<j)); }
    digitalWrite(digitEnablePins[pos],HIGH);    
  } //end for}
}
//__________________ ESP8266 driver __________________________
#else   
#ifdef PCB_VERSION
  byte panelVersion = PCB_VERSION;
#else  
  byte panelVersion = 1;
#endif

void ICACHE_RAM_ATTR writeDisplay() {       //https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
  static int timer = PWMrefresh;
  static int brightness;
  static int PWMtimeBrightness = PWM_min;
  static boolean state = true;
  int num;

  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
      timer1_write(PWMrefresh);
  return;
  }
  
  intCounter++;
/*  
  if (((panelVersion>=3) && ((displayON ?  prm.dayBright : prm.nightBright)<brightCounter)) || (!radarON)) {
    digitalWrite(LTBIpin,LOW);   //Blank display
  }
  else { 
    digitalWrite(LTBIpin,HIGH); //enable display
    for (int pos=0; pos<=maxDig; pos++) {
      num = digit[pos]; 
      if ((displayON ?  prm.dayBright : prm.nightBright)<brightCounter)  num = 10;   //clear digit
      digitalWrite(digitEnablePins[pos],LOW);   //latch enable 
      for (int j=0;j<4;j++) {digitalWrite(ABCDPins[j],num  & (1<<j)); }
      digitalWrite(digitEnablePins[pos],HIGH);    
    } //end for
  } //end else
  brightCounter++;   if (brightCounter>MAXBRIGHTNESS) brightCounter = 1;
  if (brightCounter == 1) {
    dpCounter++;   if (dpCounter>maxDig) dpCounter = 0;
    #if defined(DP_PIN) && (DP_PIN>=0)
      digitalWrite(DP_PIN,digitDP[dpCounter]); 
    #endif
  } 
  #if COLON_PIN >=0  
    digitalWrite(COLON_PIN,!colonBlinkState);  // Blink colon pin
  #endif  
*/

  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  if (autoBrightness && (displayON )) {   
    PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
    }
  else
    PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
  
  if ((!autoBrightness) && (brightness==MAXBRIGHTNESS))  
    state = true;
  
  if (state) {  //ON state
    timer = PWMtimeBrightness;
    timerON = timer;
    timerOFF = PWMrefresh-PWMtimeBrightness; 
  }
  else {  //OFF state
    timer = PWMrefresh-PWMtimeBrightness;
  }
  if (timer<500) timer = 500;  //safety only...
  if ( (brightness == 0) || (!state) || (!radarON)) {  //OFF state, blank digit
        digitalWrite(LTBIpin,LOW); //disable display
        #if COLON_PIN >=0  
          digitalWrite(COLON_PIN,LOW);  // Blink colon pin
        #endif
  }
  else {  //ON state
        digitalWrite(LTBIpin,HIGH); //enable display
        #if COLON_PIN >=0  
          digitalWrite(COLON_PIN,colonBlinkState);
        #endif
  }
  state = !state;  
  
  timer1_write(timer);
}

void writeDisplaySingle() {
  int num;
  for (int pos=0; pos<=maxDig; pos++) {
    num = digit[pos]; 
    digitalWrite(digitEnablePins[pos],LOW);   //latch enable 
    for (int j=0;j<4;j++) {digitalWrite(ABCDPins[j],num  & (1<<j)); }
    digitalWrite(digitEnablePins[pos],HIGH);    
  } //end for}}
}
#endif  //esp8266 end

void clearTubes() {
/*  //not necessary to use
  if (panelVersion==3) {
    digitalWrite(LTBIpin,LOW);
  }
  else {
  for (int pos=0; pos<=maxDigits; pos++) {
    digitalWrite(digitEnablePins[pos],LOW);
    for (int i=0;i<4;i++) digitalWrite(ABCDPins[i],HIGH); 
    digitalWrite(digitEnablePins[pos],HIGH); 
  }
  }
*/  
}


#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\PT6311.ino"
#ifdef PT6311
//CLOCK_17 
char tubeDriver[] = "PT6311";
int maxDigits = sizeof(digitEnablePins);
int wt = 1;   //Serial timing

//#define PIN_CLK  14   //  Clock
//#define PIN_DIN  13   //  DataIN
//#define PIN_LOAD 15   //  STB pin 

#define SEGMENT8   //if not used, 17 segments or more
//#define TESTMODE   //for testing the segments of a new type VFD modul

//PT6311 Commands

byte DISPLAY_MODE[11] = {
	0B00000000,	//OFF
	0B10001000,	//1
	0B10001001,	//2
	0B10001010,	//3
	0B10001011,	//4
	0B10001100,	//5
	0B10001100,	//6
	0B10001101,	//7
	0B10001110,	//8
	0B10001110,	//9 
	0B10001111,	//10
};

#define C1_DISPLAY_MODE_SETTING     B00000000  //8 digit, 20 segments
#define C2_DATA_SETTING             B01000000  //write data to display, addr increment
#define C2_DATA_SETTING_TESTMODE    B01001000  //write data to display, addr increment
#define C3_ADDRESS_SETTING          B11000000  //Address setting to 0
#define C4_DISPLAY_CONTROL          B10001111  //max brightness, ON

#ifdef SEGMENT8
//------------------abcdefgDP----------------   definition of different characters

#define MAXSEGMENTS 8
boolean ASCIImode = false;
byte charDefinition[] = {
                   B11111100,   //0: abcdef
                   B01100000,   //1: bc 
                   B11011010,   //2: abdeg
                   B11110010,   //3: abcdg
                   B01100110,   //4: bcfg
                   B10110110,   //5: acdfg
                   B10111110,   //6: acdefg
                   B11100000,   //7: abc
                   B11111110,   //8: abcdefg
                   B11110110,   //9: abcdfg
                   B00000000,   // : BLANK   (10)
                   B00000010,   //-: minus (11)
                   B00000001,   // decimal point (12)
                   B11101110,   // A  abcefg  (13)
                   B11001110,   // P  abefg (14)
                   B10011100,   // C  adef (15)
                   B11000110,   //grad (upper circle) abfg  (16)         
                   B10110100,   //%  acdf  (17) 
                   B00111010,   //lower circle cdeg  (18)                   
                   B01100000,   //I  bc    (19)
                   B10001110    //F  aefg  (20)                            
};

//byte segmentEnablePins[] =  {9,8,5,3,4,7,6,2};   //segment enable bits (a,b,c,d,e,f,g,DP)   (You MUST define always 8 bits!!!)

#else //16 segment + DP  ---------------------------------------------------------------------------------------------
#define C1_DISPLAY_MODE_SETTING    B00000000  //8 digit, 20 segments
#define MAXSEGMENTS 17
boolean ASCIImode = true;
uint32_t charDefinition[96] = {
  0b00000000000000000, /* (space) */ 
  0b10000000000001100, /* ! */   
  0b00000001000000100, /* " */   //   100000001
  0b01010101000111100, /* # */
  0b01010101010111011, /* $ */
  0b01110111010011001, /* % */
  0b01001001101110001, /* & */
  0b00000001000000000, /* ' */
  0b00001010000000000, /* ( */
  0b00100000100000000, /* ) */
  0b01111111100000000, /* * */
  0b01010101000000000, /* + */
  0b00100000000000000, /* , */
  0b01000100000000000, /* - */
  0b10000000000000000, /* . */
  0b00100010000000000, /* / */
  0b00100010011111111, /* 0 */
  0b00000010000001100, /* 1 */
  0b01000100001110111, /* 2 */
  0b00000100000111111, /* 3 */
  0b01000100010001100, /* 4 */
  0b01001000010110011, /* 5 */
  0b01000100011111011, /* 6 */
  0b00000000000001111, /* 7 */
  0b01000100011111111, /* 8 */
  0b01000100010111111, /* 9 */
  0b00010001000000000, /* : */
  0b00100001000000000, /* ; */
  0b01001010000000000, /* < */
  0b01000100000110000, /* = */
  0b00100100100000000, /* > */
  0b10010100000000111, /* ? */
  0b00000101011110111, /* @ */
  0b01000100011001111, /* A */
  0b00010101000111111, /* B */
  0b00000000011110011, /* C */
  0b00010001000111111, /* D */
  0b01000000011110011, /* E */
  0b01000000011000011, /* F */
  0b00000100011111011, /* G */
  0b01000100011001100, /* H */
  0b00010001000110011, /* I */
  0b00000000001111100, /* J */
  0b01001010011000000, /* K */
  0b00000000011110000, /* L */
  0b00000010111001100, /* M */
  0b00001000111001100, /* N */
  0b00000000011111111, /* O */
  0b01000100011000111, /* P */
  0b00001000011111111, /* Q */
  0b01001100011000111, /* R */
  0b01000100010111011, /* S */
  0b00010001000000011, /* T */
  0b00000000011111100, /* U */
  0b00100010011000000, /* V */
  0b00101000011001100, /* W */
  0b00101010100000000, /* X */
  0b01000100010111100, /* Y */
  0b00100010000110011, /* Z */
  0b00010001000010010, /* [ */
  0b00001000100000000, /* \ */
  0b00010001000100001, /* ] */
  0b00101000000000000, /* ^ */
  0b00000000000110000, /* _ */
  0b00000000100000000, /* ` */
  0b01010000001110000, /* a */
  0b01010000011100000, /* b */
  0b01000000001100000, /* c */
  0b00010100000011100, /* d */
  0b01100000001100000, /* e */
  0b01010101000000010, /* f */
  0b01010001010100001, /* g */
  0b01010000011000000, /* h */
  0b00010000000000000, /* i */
  0b00010001001100000, /* j */
  0b00011011000000000, /* k */
  0b00000000011000000, /* l */
  0b01010100001001000, /* m */
  0b01010000001000000, /* n */
  0b01010000001100000, /* o */
  0b01000001011000001, /* p */
  0b01010001010000001, /* q */
  0b01000000001000000, /* r */
  0b01010000010100001, /* s */
  0b01000000011100000, /* t */
  0b00010000001100000, /* u */
  0b00100000001000000, /* v */
  0b00101000001001000, /* w */
  0b00101010100000000, /* x */
  0b00000101000011100, /* y */
  0b01100000000100000, /* z */
  0b01010001000010010, /* { */
  0b00010001000000000, /* | */
  0b00010101000100001, /* } */
  0b01100110000000000, /* ~ */
  0b00000000000000000, /* (del) */
};

  //byte segmentEnablePins[] =  {16,7,0,1,2,3,4,5,6,8,9,11,10,12,13,14,15};    //segment enable bits 
#endif


char asciiConvert[] = "0123456789 -.APC~%oIF";

#define MAXCHARS sizeof(charDefinition)/sizeof(charDefinition[0])

void inline shift(byte Data) {
  for (int i=0;i<8;i++) {
    digitalWrite(PIN_CLK,LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_DIN, Data & (1<<i));
    delayMicroseconds(5);
    digitalWrite(PIN_CLK,HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_CLK,LOW);
    delayMicroseconds(2);
  }
}

void ICACHE_RAM_ATTR sendCommand(byte val){    //for commands C1..C3
  digitalWrite(PIN_LOAD,LOW);
  delayMicroseconds(2);
  shift(val);
  digitalWrite(PIN_LOAD,HIGH);
  delayMicroseconds(2);

  //DPRINT(address,HEX); DPRINT(","); DPRINTLN(val,HEX);
}

void setup_pins() {
  DPRINTLN("PT6311 Clock - Setup pins...");
  pinMode(PIN_LOAD,OUTPUT);   regPin(PIN_LOAD,"PIN_LOAD"); 
  pinMode(PIN_DIN, OUTPUT);   regPin(PIN_DIN,"PIN_DIN"); 
  pinMode(PIN_CLK, OUTPUT);   regPin(PIN_CLK,"PIN_CLK"); 
  digitalWrite(PIN_CLK, HIGH);
  digitalWrite(PIN_LOAD, HIGH);
  digitsOnly = true;
  delay(10);
  sendCommand(C1_DISPLAY_MODE_SETTING); 
}

#if defined(ESP32) 
void IRAM_ATTR writeDisplaySingle(){ 
#else 
void ICACHE_RAM_ATTR writeDisplaySingle(){     
#endif
byte newBright = 0;
uint32_t bitBuffer;  
int dispChar;
static int count = 15;

  newBright = displayON ?  prm.dayBright : prm.nightBright;
  if (newBright>10) newBright = 10;

  digitalWrite(PIN_LOAD, LOW);  //safety only
  delayMicroseconds(2);

  sendCommand(C2_DATA_SETTING); 
  sendCommand(DISPLAY_MODE[newBright]);
  
for (int i=0;i<maxDigits;i++) {
    dispChar = digit[i];
    if (ASCIImode) {
      if (dispChar<sizeof(asciiConvert)) dispChar = asciiConvert[dispChar];       
      dispChar -= 32;                  
    }
    bitBuffer = 0;
    digitalWrite(PIN_LOAD,LOW);
    delayMicroseconds(2);
    shift(C3_ADDRESS_SETTING + i*3);  //set display address
    for (int j=0;j<MAXSEGMENTS;j++)   
      if ((charDefinition[dispChar] & (uint32_t)1<<(MAXSEGMENTS-1-j)) != 0) {
        bitBuffer |= (uint32_t)1<<segmentEnablePins[j]; 
        }
    if (digitDP[i]) bitBuffer |= (uint32_t)1<<segmentEnablePins[7];   //Decimal Point  
    //DPRINT(charDefinition[dispChar],BIN); DPRINT(" / "); DPRINTLN(bitBuffer,BIN);
    if (newBright==0) bitBuffer = 0;  //switch OFF display
    shift(bitBuffer & 0xFF);
    shift(bitBuffer >>8);
    if (MAXSEGMENTS>16) shift(bitBuffer >>16);
    delayMicroseconds(2);
    digitalWrite(PIN_LOAD,HIGH);
    delayMicroseconds(2);
  }  //end for i

/*  //TEST  mode
  digitalWrite(PIN_LOAD, LOW);  //upload data
  delayMicroseconds(2);
  shift(C3_ADDRESS_SETTING); 
  for (int i=0;i<48;i++) {
    shift(0xff); // Data to fill table 5*16 = 80 bits
    //shift(0x00); // Data to fill table 5*16 = 80 bits
    //shift(0xff); // Data to fill table 5*16 = 80 bits
  }
  digitalWrite(PIN_LOAD, HIGH);
  delay(2);
*/  
}

/*********************** Send command without strobe/chip select******************************************/

void writeDisplay() {}
void clearTubes() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\PT6355.ino"
#ifdef PT6355
char tubeDriver[] = "PT6355";
//int maxDigits = 6;
int wt = 5;   //Serial timing

//#define PIN_CS 22 //12 or 15     // D6 CS_
//#define PIN_CLK  16   // D7 Clock
//#define PIN_SIN  21   // D5 DataIN

//#define SEGMENT8
//#define TESTMODE   //for testing the segments of a new type VFD modul

//PT6355 Commands
#define C1_DISPLAY_STATE_SETTING   B11000000  // Display duty setting=15 + Display ON
#define C3_PORT_DATA_SETTINGS      B10000000  //Port selection + output data

#ifdef SEGMENT8
//------------------abcdefgDP----------------   definition of different characters
#define C0_DISPLAY_DATA_SETTING    B11101100  //grid=10, segments<=16  
#define C2_GRID_SELECTION          B10101000  //Grid start pin setting D17..D7
#define MAXSEGMENTS 8
boolean ASCIImode = false;
byte charDefinition[] = {
                   B11111100,   //0: abcdef
                   B01100000,   //1: bc 
                   B11011010,   //2: abdeg
                   B11110010,   //3: abcdg
                   B01100110,   //4: bcfg
                   B10110110,   //5: acdfg
                   B10111110,   //6: acdefg
                   B11100000,   //7: abc
                   B11111110,   //8: abcdefg
                   B11110110,   //9: abcdfg
                   B00000000,   // : BLANK   (10)
                   B00000010,   //-: minus (11)
                   B00000001,   // decimal point (12)
                   B11101110,   // A  abcefg  (13)
                   B11001110,   // P  abefg (14)
                   B10011100,   // C  adef (15)
                   B11000110,   //grad (upper circle) abfg  (16)         
                   B10110100,   //%  acdf  (17) 
                   B00111010,   //lower circle cdeg  (18)                   
                   B01100000,   //I  bc    (19)
                   B10001110    //F  aefg  (20)                            
};

//byte segmentEnablePins[] =  {9,8,5,3,4,7,6,2};   //segment enable bits (a,b,c,d,e,f,g,DP)   (You MUST define always 8 bits!!!)

#else //16 segment + DP  ---------------------------------------------------------------------------------------------
#define C0_DISPLAY_DATA_SETTING    B11101111  //grid=10, segments>17  
#define C2_GRID_SELECTION          B10101000  //Grid start pin setting D17..D7
#define MAXSEGMENTS 17
boolean ASCIImode = true;
uint32_t charDefinition[96] = {
  0b00000000000000000, /* (space) */ 
  0b10000000000001100, /* ! */   
  0b00000001000000100, /* " */   //   100000001
  0b01010101000111100, /* # */
  0b01010101010111011, /* $ */
  0b01110111010011001, /* % */
  0b01001001101110001, /* & */
  0b00000001000000000, /* ' */
  0b00001010000000000, /* ( */
  0b00100000100000000, /* ) */
  0b01111111100000000, /* * */
  0b01010101000000000, /* + */
  0b00100000000000000, /* , */
  0b01000100000000000, /* - */
  0b10000000000000000, /* . */
  0b00100010000000000, /* / */
  0b00100010011111111, /* 0 */
  0b00000010000001100, /* 1 */
  0b01000100001110111, /* 2 */
  0b00000100000111111, /* 3 */
  0b01000100010001100, /* 4 */
  0b01001000010110011, /* 5 */
  0b01000100011111011, /* 6 */
  0b00000000000001111, /* 7 */
  0b01000100011111111, /* 8 */
  0b01000100010111111, /* 9 */
  0b00010001000000000, /* : */
  0b00100001000000000, /* ; */
  0b01001010000000000, /* < */
  0b01000100000110000, /* = */
  0b00100100100000000, /* > */
  0b10010100000000111, /* ? */
  0b00000101011110111, /* @ */
  0b01000100011001111, /* A */
  0b00010101000111111, /* B */
  0b00000000011110011, /* C */
  0b00010001000111111, /* D */
  0b01000000011110011, /* E */
  0b01000000011000011, /* F */
  0b00000100011111011, /* G */
  0b01000100011001100, /* H */
  0b00010001000110011, /* I */
  0b00000000001111100, /* J */
  0b01001010011000000, /* K */
  0b00000000011110000, /* L */
  0b00000010111001100, /* M */
  0b00001000111001100, /* N */
  0b00000000011111111, /* O */
  0b01000100011000111, /* P */
  0b00001000011111111, /* Q */
  0b01001100011000111, /* R */
  0b01000100010111011, /* S */
  0b00010001000000011, /* T */
  0b00000000011111100, /* U */
  0b00100010011000000, /* V */
  0b00101000011001100, /* W */
  0b00101010100000000, /* X */
  0b01000100010111100, /* Y */
  0b00100010000110011, /* Z */
  0b00010001000010010, /* [ */
  0b00001000100000000, /* \ */
  0b00010001000100001, /* ] */
  0b00101000000000000, /* ^ */
  0b00000000000110000, /* _ */
  0b00000000100000000, /* ` */
  0b01010000001110000, /* a */
  0b01010000011100000, /* b */
  0b01000000001100000, /* c */
  0b00010100000011100, /* d */
  0b01100000001100000, /* e */
  0b01010101000000010, /* f */
  0b01010001010100001, /* g */
  0b01010000011000000, /* h */
  0b00010000000000000, /* i */
  0b00010001001100000, /* j */
  0b00011011000000000, /* k */
  0b00000000011000000, /* l */
  0b01010100001001000, /* m */
  0b01010000001000000, /* n */
  0b01010000001100000, /* o */
  0b01000001011000001, /* p */
  0b01010001010000001, /* q */
  0b01000000001000000, /* r */
  0b01010000010100001, /* s */
  0b01000000011100000, /* t */
  0b00010000001100000, /* u */
  0b00100000001000000, /* v */
  0b00101000001001000, /* w */
  0b00101010100000000, /* x */
  0b00000101000011100, /* y */
  0b01100000000100000, /* z */
  0b01010001000010010, /* { */
  0b00010001000000000, /* | */
  0b00010101000100001, /* } */
  0b01100110000000000, /* ~ */
  0b00000000000000000, /* (del) */
};

  //byte segmentEnablePins[] =  {16,7,0,1,2,3,4,5,6,8,9,11,10,12,13,14,15};    //segment enable bits 
#endif


char asciiConvert[] = "0123456789 -.APC~%oIF";

#define MAXCHARS sizeof(charDefinition)/sizeof(charDefinition[0])


void inline startBits(byte CS) {
  digitalWrite(CS, LOW);
  delayMicroseconds(wt);
}

void inline stopBits(byte CS) {
  delayMicroseconds(wt);
  digitalWrite(CS, HIGH);
  delayMicroseconds(wt);
}

void inline shift(byte SIN, byte CLK, byte Data) {
  for (int i=0;i<8;i++) {
    digitalWrite(CLK,LOW);
    delayMicroseconds(wt);
    digitalWrite(SIN, Data & (1<<i));
    delayMicroseconds(wt);
    digitalWrite(CLK,HIGH);
    delayMicroseconds(wt);
  }
  delayMicroseconds(wt);
}

void ICACHE_RAM_ATTR sendCommand(byte val){    //for commands C1..C3
  startBits(PIN_CS);
  shift(PIN_SIN, PIN_CLK, val);
  stopBits(PIN_CS);

  //DPRINT(address,HEX); DPRINT(","); DPRINTLN(val,HEX);
}

void setup_pins() {
  DPRINTLN("PT6355 Clock - Setup pins...");
  pinMode(PIN_CS,OUTPUT);     regPin(PIN_CS,"PIN_CS"); 
  pinMode(PIN_SIN, OUTPUT);   regPin(PIN_SIN,"PIN_SIN"); 
  pinMode(PIN_CLK, OUTPUT);   regPin(PIN_CLK,"PIN_CLK"); 
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_CLK, HIGH);
  digitsOnly = false;
  delay(1);

  sendCommand(C1_DISPLAY_STATE_SETTING);
  sendCommand(C2_GRID_SELECTION);
  #ifdef TESTMODE
    while (true) {writeDisplaySingle(); delay(10000);}
    for (int i=0;i<10;i++) digit[i] = 1;   // " char
    while (true) {writeDisplaySingle(); delay(10000);}
  #endif
   
}

#if defined(ESP32) 
void IRAM_ATTR writeDisplaySingle(){ 
#else 
void ICACHE_RAM_ATTR writeDisplaySingle(){     
#endif
static byte oldBright = 0;
static byte newBright = 0;
byte dispState;
uint32_t bitBuffer;  
int dispChar;
static int count = 15;

  #ifdef TESTMODE
    DPRINT("count:"); DPRINTLN(count);
  #endif
  newBright = displayON ?  prm.dayBright : prm.nightBright;
  if (newBright != oldBright) {
    dispState = C1_DISPLAY_STATE_SETTING;
    if (newBright>0) dispState |= 1;  
    if (newBright>5) dispState |= 6;
    sendCommand(dispState);    //Set Brightness
    oldBright = newBright;
  }
  startBits(PIN_CS);
  shift(PIN_SIN, PIN_CLK, C0_DISPLAY_DATA_SETTING);
  for (int i=0;i<maxDigits;i++) {
    dispChar = digit[i];
    if (ASCIImode) {
      if (dispChar<sizeof(asciiConvert)) dispChar = asciiConvert[dispChar];       
      dispChar -= 32;                  
    }
    bitBuffer = 0;
    for (int j=0;j<MAXSEGMENTS;j++)   
      if ((charDefinition[dispChar] & (uint32_t)1<<(MAXSEGMENTS-1-j)) != 0) {
        bitBuffer |= (uint32_t)1<<segmentEnablePins[j]; 
        }
    //if (digitDP[i]) bitBuffer |= (uint32_t)1<<segmentEnablePins[7];   //Decimal Point  
    #ifdef TESTMODE
      bitBuffer = (uint32_t)1<<count;
    #endif   
    //DPRINT(charDefinition[dispChar],BIN); DPRINT(" / "); DPRINTLN(bitBuffer,BIN);
    //bitBuffer = 0x03FFFF;
    shift(PIN_SIN, PIN_CLK, bitBuffer & 0xFF);
    shift(PIN_SIN, PIN_CLK, bitBuffer >>8);
    if (MAXSEGMENTS>16) shift(PIN_SIN, PIN_CLK, bitBuffer >>16);
  }  //end for i
  stopBits(PIN_CS);
  count++;
}

void writeDisplay() {
}

void clearTubes() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\SN75512.ino"
#ifdef SN75512   
// 2 x SN75512
char tubeDriver[] = "SN75512";
#define VFDrefresh 1200    //msec, Multiplex time period. Greater value => slower multiplex frequency

#define DATABITS 24        //total length of the shift registers

//Fill this table with the bit positions of the shift registers!   
byte segmentEnablePins[] =  {16,17,18,19,20,21,22,23};  //segment enable bits (a,b,c,d,e,f,g,DP)  (You MUST define always 8 bits!!!)
byte digitEnablePins[] = {0,1,2,3,4,5,6,7,8};           //digit enable bits   (You may define any number of digits between 1 and 10 )

//SN75512 control pins
#define PIN_LE      13  // D7 Shift Register Latch Enable
#define PIN_CLK     12  // D6 Shift Register Clock
#define PIN_DATA    14  // D5 Shift Register Data
#define PIN_STROBE  5   // D1 Shift Register Strobe (1=display off     0=display on)

#define PIN_LE_BIT     1<<PIN_LE    
#define PIN_CLK_BIT    1<<PIN_CLK    
#define PIN_DATA_BIT   1<<PIN_DATA  
#define PIN_STROBE_BIT 1<<PIN_STROBE    



//------------------abcdefgDP----------------   definition of different characters
byte charDefinition[] = {
                   B11111100,   //0: abcdef
                   B01100000,   //1: bc 
                   B11011010,   //2: abdeg
                   B11110010,   //3: abcdg
                   B01100110,   //4: bcfg
                   B10110110,   //5: acdfg
                   B10111110,   //6: acdefg
                   B11100000,   //7: abc
                   B11111110,   //8: abcdefg
                   B11110110,   //9: abcdfg
                   B00000000,   // : BLANK   (10)
                   B00000010,   //-: minus (11)
                   B00000001,   // decimal point (12)
                   B11101110,   // A  abcefg  (13)
                   B11001110,   // P  abefg (14)
                   B10011100,   // C  adef (15)
                   B11000110,   //grad (upper circle) abfg  (16)
                   B10110100,   //%  acdf  (17)
                   B00111010,   //lower circle cdeg  (18)                   
                   B01100000,   //I  bc    (19)
                   B10001110    //F  aefg  (20)                   
};

#define MAXCHARS sizeof(charDefinition)
#define MAXSEGMENTS sizeof(segmentEnablePins)
int maxDigits =  sizeof(digitEnablePins);

uint32_t charTable[MAXCHARS];              //generated pin table from segmentDefinitions
uint32_t segmentEnableBits[MAXSEGMENTS];   //bitmaps, generated from EnablePins tables
uint32_t digitEnableBits[10];
//-----------------------------------------------------------------------------------------

void ICACHE_RAM_ATTR writeDisplay(){        
static volatile uint32_t val = 0;
static volatile byte pos = 0;
static volatile int brightCounter[] = {0,9,2,8,4,7,6,5,3,1};

//https://sub.nanona.fi/esp8266/timing-and-ticks.html
//One tick is 1us / 80 = 0.0125us = 12.5ns on 80MHz
//One tick is 1us / 80 = 0.0125us = 6.25ns on 160MHz
// 1 NOP is 1 tick

  //noInterrupts();
  
 
  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    timer1_write(VFDrefresh);
    return;  
  }

  if (brightCounter[pos] % MAXBRIGHTNESS < (displayON ?  prm.dayBright : prm.nightBright))
    WRITE_PERI_REG( PIN_OUT_CLEAR, PIN_STROBE_BIT );   //ON
  else 
    WRITE_PERI_REG( PIN_OUT_SET, PIN_STROBE_BIT );    //OFF
    
  for (int t=0; t<4;t++) asm volatile ("nop");
  
  brightCounter[pos]++;  
  if (brightCounter[pos]>MAXBRIGHTNESS) brightCounter[pos] = 1;
  
  val = (digitEnableBits[pos] | charTable[digit[pos]]);  //the full bitmap to send to MAX chip
  if (digitDP[pos]) val = val | charTable[12];    //Decimal Point
  for (int i=0; i<DATABITS; i++)  {
    if ((val & (uint32_t)1 << (DATABITS -1 - i)) ) {
      WRITE_PERI_REG( PIN_OUT_SET, PIN_DATA_BIT );
          for (int t=0; t<4;t++) asm volatile ("nop");   
    }
    else {
      WRITE_PERI_REG( PIN_OUT_CLEAR, PIN_DATA_BIT );
          for (int t=0; t<8;t++) asm volatile ("nop");   
    }
    WRITE_PERI_REG( PIN_OUT_SET, PIN_CLK_BIT );
    for (int t=0; t<8;t++) asm volatile ("nop");  
    WRITE_PERI_REG( PIN_OUT_CLEAR, PIN_CLK_BIT );
    for (int t=0; t<4;t++) asm volatile ("nop");   
    } //end for      
 
  WRITE_PERI_REG( PIN_OUT_SET, PIN_LE_BIT );
  for (int t=0; t<4;t++) asm volatile ("nop");   
  WRITE_PERI_REG( PIN_OUT_CLEAR, PIN_LE_BIT );

  pos++; if (pos >= maxDigits) pos = 0; 

  timer1_write(VFDrefresh);
  //interrupts();
}

void generateBitTable() {
uint32_t out;

  DPRINTLN("--- Generating segment pins bitmap ---");
  for (int i=0;i<MAXSEGMENTS;i++) {
    segmentEnableBits[i] = (uint32_t)1<<segmentEnablePins[i];
    //DPRINT(i); DPRINT(": "); DPRINTLN(segmentEnableBits[i],BIN);
  }
  
  DPRINTLN("--- Generating digit pins bitmap ---");
  for (int i=0;i<maxDigits;i++) {
    digitEnableBits[i] = (uint32_t)1 << digitEnablePins[i];
    //DPRINT(i); DPRINT(": "); DPRINTLN( digitEnableBits[i],BIN);
  }

DPRINTLN("---- Generated Character / Pins table -----");
  for (int i=0;i<MAXCHARS;i++) {
    out = 0;
    //DPRINT(i); DPRINT(":  ");
    //DPRINT(charDefinition[i],BIN);  //DPRINT(" = ");
    for (int j=0;j<=7;j++)   //from a to g
      if ((charDefinition[i] & (uint32_t)1<<(7-j)) != 0) {
        out = out | segmentEnableBits[j]; //DPRINT("1"); 
        }
    //else        DPRINT("0");
    //DPRINT("  >> ");  
    
    charTable[i] = out;
    //DPRINTLN(charTable[i],BIN);
  }  //end for
}


void setup_pins() {
#if defined(ESP8266) 
#else
  #error "Only 8266 Board is supported!"  
#endif
  
  DPRINTLN("Clock with SN75512 - Setup pins...");
  pinMode(PIN_LE,  OUTPUT);   regPin(PIN_LE,"PIN_LE"); 
  pinMode(PIN_STROBE,OUTPUT); regPin(PIN_STROBE,"PIN_STROBE"); 
  pinMode(PIN_DATA,OUTPUT);   regPin(PIN_DATA,"PIN_DATA"); 
  pinMode(PIN_CLK, OUTPUT);   regPin(PIN_CLK,"PIN_CLK"); 
  digitalWrite(PIN_STROBE,LOW);  //brightness
  
  generateBitTable();
  digitsOnly = false;
    
  startTimer();
}  

void clearTubes() {}
void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\SN75518.ino"
#ifdef SN75518_ESP32
char tubeDriver[] = "SN75518_ESP32";
//int maxDigits = 6;
int wt = 5;   //Serial timing

#define PIN_LE      13  // D7 Shift Register Latch Enable
#define PIN_CLK     12  // D6 Shift Register Clock
#define PIN_DATA    27  // D5 Shift Register Data
#define PIN_STROBE  14  // D1 Shift Register Strobe (1=display off     0=display on)

//#define SEGMENT8   //please set in clocks.h
//#define TESTMODE   //for testing the segments of a new type VFD modul
int count = 15;  //output# to test
unsigned long lastTest = 0;

#ifdef SEGMENT8
  boolean ASCIImode = false;
  #define DP_CHAR 12
  byte charDefinition[] = {
//------------------abcdefgDP----------------   definition of different characters
                   B11111100,   //0: abcdef
                   B01100000,   //1: bc 
                   B11011010,   //2: abdeg
                   B11110010,   //3: abcdg
                   B01100110,   //4: bcfg
                   B10110110,   //5: acdfg
                   B10111110,   //6: acdefg
                   B11100000,   //7: abc
                   B11111110,   //8: abcdefg
                   B11110110,   //9: abcdfg
                   B00000000,   // : BLANK   (10)
                   B00000010,   //-: minus (11)
                   B00000001,   // decimal point (12)
                   B11101110,   // A  abcefg  (13)
                   B11001110,   // P  abefg (14)
                   B10011100,   // C  adef (15)
                   B11000110,   //grad (upper circle) abfg  (16)         
                   B10110100,   //%  acdf  (17) 
                   B00111010,   //lower circle cdeg  (18)                   
                   B01100000,   //I  bc    (19)
                   B10001110    //F  aefg  (20)                            
};

//byte segmentEnablePins[] =  {9,8,5,3,4,7,6,2};   //segment enable bits (a,b,c,d,e,f,g,DP)   (You MUST define always 8 bits!!!)

#else
  //16 segment + DP  ---------------------------------------------------------------------------------------------
  boolean ASCIImode = true;
  #define DP_CHAR 14
  uint32_t charDefinition[96] = {
//  Dutsrpnmkhgfedcba   
  0b00000000000000000, /* (space) */ 
  0b10000000000001100, /* ! */   
  0b00000001000000100, /* " */   //   100000001
  0b01010101000111100, /* # */
  0b01010101010111011, /* $ */
  0b01110111010011001, /* % */
  0b01001001101110001, /* & */
  0b00000001000000000, /* ' */
  0b00001010000000000, /* ( */
  0b00100000100000000, /* ) */
  0b01111111100000000, /* * */
  0b01010101000000000, /* + */
  0b00100000000000000, /* , */
  0b01000100000000000, /* - */
  0b10000000000000000, /* . */
  0b00100010000000000, /* / */
  0b00100010011111111, /* 0 */
  0b00000010000001100, /* 1 */
  0b01000100001110111, /* 2 */
  0b00000100000111111, /* 3 */
  0b01000100010001100, /* 4 */
  0b01001000010110011, /* 5 */
  0b01000100011111011, /* 6 */
  0b00000000000001111, /* 7 */
  0b01000100011111111, /* 8 */
  0b01000100010111111, /* 9 */
  0b00010001000000000, /* : */
  0b00100001000000000, /* ; */
  0b01001010000000000, /* < */
  0b01000100000110000, /* = */
  0b00100100100000000, /* > */
  0b10010100000000111, /* ? */
  0b00000101011110111, /* @ */
  0b01000100011001111, /* A */
  0b00010101000111111, /* B */
  0b00000000011110011, /* C */
  0b00010001000111111, /* D */
  0b01000000011110011, /* E */
  0b01000000011000011, /* F */
  0b00000100011111011, /* G */
  0b01000100011001100, /* H */
  0b00010001000110011, /* I */
  0b00000000001111100, /* J */
  0b01001010011000000, /* K */
  0b00000000011110000, /* L */
  0b00000010111001100, /* M */
  0b00001000111001100, /* N */
  0b00000000011111111, /* O */
  0b01000100011000111, /* P */
  0b00001000011111111, /* Q */
  0b01001100011000111, /* R */
  0b01000100010111011, /* S */
  0b00010001000000011, /* T */
  0b00000000011111100, /* U */
  0b00100010011000000, /* V */
  0b00101000011001100, /* W */
  0b00101010100000000, /* X */
  0b01000100010111100, /* Y */
  0b00100010000110011, /* Z */
  0b00010001000010010, /* [ */
  0b00001000100000000, /* \ */
  0b00010001000100001, /* ] */
  0b00101000000000000, /* ^ */
  0b00000000000110000, /* _ */
  0b00000000100000000, /* ` */
  0b01010000001110000, /* a */
  0b01010000011100000, /* b */
  0b01000000001100000, /* c */
  0b00010100000011100, /* d */
  0b01100000001100000, /* e */
  0b01010101000000010, /* f */
  0b01010001010100001, /* g */
  0b01010000011000000, /* h */
  0b00010000000000000, /* i */
  0b00010001001100000, /* j */
  0b00011011000000000, /* k */
  0b00000000011000000, /* l */
  0b01010100001001000, /* m */
  0b01010000001000000, /* n */
  0b01010000001100000, /* o */
  0b01000001011000001, /* p */
  0b01010001010000001, /* q */
  0b01000000001000000, /* r */
  0b01010000010100001, /* s */
  0b01000000011100000, /* t */
  0b00010000001100000, /* u */
  0b00100000001000000, /* v */
  0b00101000001001000, /* w */
  0b00101010100000000, /* x */
  0b00000101000011100, /* y */
  0b01100000000100000, /* z */
  0b01010001000010010, /* { */
  0b00010001000000000, /* | */
  0b00010101000100001, /* } */
  0b01000001010000001, /* ~ */
  0b00000000000000000, /* (del) */
  //Dutsrpnmkhgfedcba  
};

  //byte segmentEnablePins[] =  {16,7,0,1,2,3,4,5,6,8,9,11,10,12,13,14,15};    //segment enable bits defined in clocks.h
#endif

#define MAXCHARS sizeof(charDefinition)/sizeof(charDefinition[0])
#define MAXSEGMENTS sizeof(segmentEnablePins)
char asciiConvert[] = "0123456789 -.APC~% IF";
int maxDigits =  sizeof(digitEnablePins);
int dispChar;

uint32_t DRAM_ATTR animationMaskBits[5];
uint32_t DRAM_ATTR charTable[MAXCHARS];              //generated pin table from segmentDefinitions
uint32_t DRAM_ATTR segmentEnableBits[MAXSEGMENTS];   //bitmaps, generated from EnablePins tables
uint32_t DRAM_ATTR digitEnableBits[10];

int DRAM_ATTR PWMrefresh=5500;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
int DRAM_ATTR PWM_min = 500;
int DRAM_ATTR PWM_max = 5000;
//int DRAM_ATTR PWMtiming[11] = {0,500,800,1200,2000,2500,3000,3500,4000,4500,5000};

//-----------------------------------------------------------------------------------------

//https://sub.nanona.fi/esp8266/timing-and-ticks.html
//One tick is 1us / 80 = 0.0125us = 12.5ns on 80MHz
//One tick is 1us / 80 = 0.0125us = 6.25ns on 160MHz
// 1 NOP is 1 tick

void inline delayMS(int d) {
  asm volatile ("nop"); 
  asm volatile ("nop");
  asm volatile ("nop");
}

void setup_pins() {
  DPRINTLN("VFD Clock - setup SN75518 pins");
  pinMode(PIN_LE,  OUTPUT);  regPin(PIN_LE,"PIN_LE");
  pinMode(PIN_STROBE,  OUTPUT);  regPin(PIN_STROBE,"PIN_STROBE");
  digitalWrite(PIN_STROBE,LOW);  //brightness
  pinMode(PIN_DATA,OUTPUT);  regPin(PIN_DATA,"PIN_DATA");
  pinMode(PIN_CLK, OUTPUT);  regPin(PIN_CLK,"PIN_CLK");
  #ifdef TESTMODE
    DPRINT("Testing output Q#:"); DPRINTLN(count);
  #endif
  driverSetupStr = "SN75518 segmentEnablePins:";
  for (int j=0;j<sizeof(segmentEnablePins);j++) {
    driverSetupStr += String(segmentEnablePins[j]) + ",";
  }
  driverSetupStr += "<br>SN75518 digitEnablePins:";
  for (int j=0;j<sizeof(digitEnablePins);j++) {
    driverSetupStr += String(digitEnablePins[j]) + ",";
  }
  driverSetupStr += String("<br>");
  
  generateBitTable();
  digitsOnly = false;
  startTimer();
}  

void IRAM_ATTR writeDisplay(){  //void IRAM_ATTR  writeDisplay(){
  static DRAM_ATTR int timer = PWMrefresh;
  static DRAM_ATTR uint32_t val;
  static DRAM_ATTR byte pos = 0;
  static DRAM_ATTR boolean state=true;
  static DRAM_ATTR int brightness;
  static DRAM_ATTR int PWMtimeBrightness=PWM_min;

  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
      //digitalWrite(PIN_STROBE,HIGH);    //OFF
    return;
  }
  //digit[5] = 15;   digit[4] = 16;     digit[3] = 17;     digit[2] = 18;      digit[1] = 19;       digit[0] = 20;
  portENTER_CRITICAL_ISR(&timerMux);
  noInterrupts();
  intCounter++;

  brightness = displayON ? prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety
  
  if ((!autoBrightness) && (brightness==MAXBRIGHTNESS)) state = true;
  
  if (state) {  //ON state
    pos++;  if (pos>maxDigits-1)  {   //go to the tube#0
      pos = 0; 
      if (autoBrightness && displayON) {   //change brightness only on the tube#0
        PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
        }
      else
        PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
    }  
    dispChar = digit[pos];
    if (ASCIImode) {
      if (dispChar<sizeof(asciiConvert)) dispChar = asciiConvert[dispChar];       
      dispChar -= 32;                  
    }
    val = (digitEnableBits[pos] | charTable[dispChar]);  //the full bitmap to send to MAX chip
    if (digitDP[pos]) val = val | charTable[DP_CHAR];    //Decimal Point
    #ifdef TESTMODE
      val = digitEnableBits[pos] | ((uint32_t)1<<(count-1));
      animMask[pos] = 0;
      if ((millis()-lastTest)>5000) {
        lastTest=millis();
        count++; if (count>32) count = 0;
      }
    #endif       
    timer = PWMtimeBrightness;
    #ifdef CLOCK_xx  //example only
      if (pos==2) timer = 2*timer;  //Weak  tube#2 brightness compensation, some hacking
      if (pos==4) timer = 3*timer;  //Weak  tube#4 brightness compensation, some hacking
    #endif 
    timerON = timer;
    timerOFF = PWMrefresh-PWMtimeBrightness;
  }
  else {  //OFF state
    timer = PWMrefresh-PWMtimeBrightness;
  }
  
  if (timer<500) timer = 500;  //safety only...

  if ( (brightness == 0) || (!state) || (!radarON)) {  //OFF state, blank digit
    digitalWrite(PIN_STROBE,HIGH);    //OFF
  }
  else {  //ON state
    if (animMask[pos]>0) val &= animationMaskBits[animMask[pos]-1];  //animationMode 6, mask characters from up to down and back
    for (int i=0; i<32; i++)  {
      if (val & uint32_t(1 << (31 - i)))
        {digitalWrite(PIN_DATA, HIGH);   //asm volatile ("nop");
        }
      else
        {digitalWrite(PIN_DATA, LOW);    //asm volatile ("nop");
        }
      
      digitalWrite(PIN_CLK,HIGH);  //asm volatile ("nop");  //delayMS(1);
      digitalWrite(PIN_CLK,LOW);   //asm volatile ("nop"); //delayMS(1);
      } //end for      
 
    digitalWrite(PIN_LE,HIGH );  asm volatile ("nop");
    digitalWrite(PIN_LE,LOW);
    digitalWrite(PIN_STROBE,LOW );   //ON
  }  //end else
  
  portEXIT_CRITICAL_ISR(&timerMux);   
  state = !state;  
  //ESP32timer = timerBegin(0, PRESCALER, true);  //set prescaler, true = edge generated signal
  //timerAttachInterrupt(ESP32timer, &writeDisplay, true);   
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
}

void generateBitTable() {
uint32_t out;

  DPRINT("--- Generating segment pins bitmap:"); DPRINTLN(MAXSEGMENTS);
  for (int i=0;i<MAXSEGMENTS;i++) {
    segmentEnableBits[i] = uint32_t(1)<<(segmentEnablePins[i]-1); 
    //DPRINT(i); DPRINT(": "); DPRINTLN(segmentEnableBits[i],BIN);
  }
#ifdef SEGMENT8
  animationMaskBits[0] = uint32_t(1<<segmentEnablePins[0]);  //a
  animationMaskBits[1] = animationMaskBits[0] | uint32_t(1)<<segmentEnablePins[1] | uint32_t(1<<segmentEnablePins[5]);  //bf
  animationMaskBits[2] = animationMaskBits[1] | uint32_t(1)<<segmentEnablePins[6];  //g
  animationMaskBits[3] = animationMaskBits[2] | uint32_t(1)<<segmentEnablePins[4] | uint32_t(1<<segmentEnablePins[2]);  //ec
  animationMaskBits[4] = animationMaskBits[3] | uint32_t(1)<<segmentEnablePins[3];  //d
#else  
  //Dutsrpnmkhgfedcba  
  //DP=16,u=15,t=14,s=13,r=12,p=11,n=10,m=9,k=8,h=7,g=6,f=5,e=4,d=3,c=2,b=1,a=0}
  animationMaskBits[0] = uint32_t(1<<segmentEnablePins[0]) | uint32_t(1<<segmentEnablePins[1]);  //ab
  animationMaskBits[1] = animationMaskBits[0] | uint32_t(1<<segmentEnablePins[2]) | uint32_t(1<<segmentEnablePins[8]) |
      uint32_t(1<<segmentEnablePins[9]) | uint32_t(1<<segmentEnablePins[10]) | uint32_t(1<<segmentEnablePins[7]);  //hkmnc
  animationMaskBits[2] = animationMaskBits[1] | uint32_t(1<<segmentEnablePins[15]) | uint32_t(1<<segmentEnablePins[11]);  //up
  animationMaskBits[3] = animationMaskBits[2] | uint32_t(1<<segmentEnablePins[6]) | uint32_t(1<<segmentEnablePins[14]) |
      uint32_t(1<<segmentEnablePins[13]) |uint32_t(1<<segmentEnablePins[12]) |uint32_t(1<<segmentEnablePins[3]);  //gtsrd
  animationMaskBits[4] = animationMaskBits[3] | uint32_t(1<<segmentEnablePins[5]) | uint32_t(1<<segmentEnablePins[4]);  //fe
#endif  
  DPRINTLN("--- Generating animation mask bitmap:");
  for (int i=0;i<5;i++) {
    animationMaskBits[i] = ~animationMaskBits[i]; //invert bits
    //DPRINTLN(animationMaskBits[i],BIN);
  }
  DPRINT("--- Generating digit pins bitmap:"); DPRINTLN(maxDigits);
  for (int i=0;i<maxDigits;i++) {
    digitEnableBits[i] = uint32_t(1 << (digitEnablePins[i]-1));
    //DPRINT(i); DPRINT(": "); DPRINTLN( digitEnableBits[i],BIN);
  }

  DPRINT("--- Generated Character / Pins table:"); DPRINTLN(MAXCHARS);
  for (int i=0;i<MAXCHARS;i++) {
    out = 0;
    //DPRINT(i); DPRINT(":  ");
    //DPRINT(charDefinition[i],BIN);  //DPRINT(" = ");
    for (int j=0;j<MAXSEGMENTS;j++)   //from a to g
      if ((uint32_t)(charDefinition[i] & (uint32_t)1<<j) != 0) {
        out = out | segmentEnableBits[MAXSEGMENTS-j-1]; //DPRINT("1"); 
        }
    //else        {DPRINT("0");}
    //DPRINT("  >> ");  
    
    charTable[i] = out;
    //DPRINTLN(charTable[i],BIN);
  }  //end for
}


void clearTubes() {
  digitalWrite(PIN_STROBE,HIGH);
}

void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\VQC10.ino"
#ifdef VQC10
char tubeDriver[] = "VQC10";
#define NUMDIGITS 8
int maxDigits = NUMDIGITS;

//Please, set the pin numbers in clock.h!
//#define latchPin  5 //D2 RCK
//#define clockPin  4 //D1 SCK
//#define dataPin   2 //D4 DIN
//#define ledOffpin 0 //D5
//#define D5pin  14


#define NUMCOLS 5
#define NUMROWS 7

boolean _upsidedown = false;
byte row[NUMROWS];

char asciiConvert[] = "0123456789 -.APC~%oIF";
char DPchar = 0xFF;  //decimal point

char cDigit[NUMDIGITS+1] = "76543210";
byte  dat[NUMDIGITS][NUMROWS];
byte  oldDat[NUMDIGITS][NUMROWS];
byte  newDat[NUMDIGITS][NUMROWS];

// standard ascii 5x7 font incl. the IPM-PC extension:
// already defined in DotMatrx5x7.h
//#define aUML "\x84" // ä = 0x84
//#define oUML "\x94" // ö = 0x94
//#define uUML "\x81" // ü = 0x81
//#define sZET "\xE1" // ß = 0xE1
//#define AUML "\x8E" // Ä = 0x8E
//#define OUML "\x99" // Ö = 0x99
//#define UUML "\x9A" // Ü = 0x9A

byte  _font[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, // 0x00 empty
  0x3E, 0x5B, 0x4F, 0x5B, 0x3E, // 0x01 sad
  0x3E, 0x6B, 0x4F, 0x6B, 0x3E, // 0x02 happy
  0x1C, 0x3E, 0x7C, 0x3E, 0x1C, // 0x03 heart
  0x18, 0x3C, 0x7E, 0x3C, 0x18, // 0x04 diamond
  0x1C, 0x57, 0x7D, 0x57, 0x1C, // 0x05 clover
  0x1C, 0x5E, 0x7F, 0x5E, 0x1C, // 0x06 spade
  0x00, 0x18, 0x3C, 0x18, 0x00, // 0x07 small full circle
  0xFF, 0xE7, 0xC3, 0xE7, 0xFF, // 0x08 inverted small circle 
  0x06, 0x09, 0x06, 0x00, 0x00, // 0x09 small empty circle     //old: 0x00, 0x18, 0x24, 0x18, 0x00, // 0x09 small empty circle 
  0xFF, 0xE7, 0xDB, 0xE7, 0xFF, // 0x0A block (linefeed)
  0x30, 0x48, 0x3A, 0x06, 0x0E, // 0x0B male
  0x26, 0x29, 0x79, 0x29, 0x26, // 0x0C female
  0x40, 0x7F, 0x05, 0x05, 0x07, // 0x0D note (carriage return)
  0x40, 0x7F, 0x05, 0x25, 0x3F, // 0x0E two notes
  0x5A, 0x3C, 0xE7, 0x3C, 0x5A, // 0x0F sun
  0x7F, 0x3E, 0x1C, 0x1C, 0x08, // 0x10 right triangle
  0x08, 0x1C, 0x1C, 0x3E, 0x7F, // 0x11 left triangle
  0x14, 0x22, 0x7F, 0x22, 0x14, // 0x12 up/down arrow
  0x5F, 0x5F, 0x00, 0x5F, 0x5F, // 0x13 double exclamation mark
  0x06, 0x09, 0x7F, 0x01, 0x7F, // 0x14 paragraph end sign (pilcrow)
  // 0x00, 0x66, 0x89, 0x95, 0x6A, // 0x15 paragraph sign original
  0x00, 0x26, 0x49, 0x55, 0x28, // 0x15 paragraph sign
  0x60, 0x60, 0x60, 0x60, 0x60, // 0x16 double underline
  0x54, 0x62, 0x7F, 0x62, 0x54, // 0x17 up/down arrow with underline
  0x08, 0x04, 0x7E, 0x04, 0x08, // 0x18 up arrow
  0x10, 0x20, 0x7E, 0x20, 0x10, // 0x19 down arrow
  0x08, 0x08, 0x2A, 0x1C, 0x08, // 0x1A right arrow
  0x08, 0x1C, 0x2A, 0x08, 0x08, // 0x1B left arrow
  0x14, 0x3E, 0x55, 0x41, 0x22, // 0x1C euro
  0x63, 0x75, 0x69, 0x75, 0x63, // 0x1D timer
  0x30, 0x38, 0x3E, 0x38, 0x30, // 0x1E up triangle
  0x06, 0x0E, 0x3E, 0x0E, 0x06, // 0x1F down triangle
  0x00, 0x00, 0x00, 0x00, 0x00, // 0x20 space
  0x00, 0x00, 0x5F, 0x00, 0x00, // 0x21 !
  0x00, 0x07, 0x00, 0x07, 0x00, // 0x22 "
  0x14, 0x7F, 0x14, 0x7F, 0x14, // 0x23 #
  0x24, 0x2A, 0x7F, 0x2A, 0x12, // 0x24 $
  0x23, 0x13, 0x08, 0x64, 0x62, // 0x25 %
  0x36, 0x49, 0x56, 0x20, 0x50, // 0x26 &
  0x00, 0x08, 0x07, 0x03, 0x00, // 0x27 ´
  0x00, 0x1C, 0x22, 0x41, 0x00, // 0x28 (
  0x00, 0x41, 0x22, 0x1C, 0x00, // 0x29 )
  0x2A, 0x1C, 0x7F, 0x1C, 0x2A, // 0x2A *
  0x08, 0x08, 0x3E, 0x08, 0x08, // 0x2B +
  0x00, 0x80, 0x70, 0x30, 0x00, // 0x2C ,
  0x08, 0x08, 0x08, 0x08, 0x08, // 0x2D -
  0x00, 0x00, 0x60, 0x60, 0x00, // 0x2E .
  0x20, 0x10, 0x08, 0x04, 0x02, // 0x2F /
  0x3E, 0x51, 0x49, 0x45, 0x3E, // 0x30 0
  0x00, 0x42, 0x7F, 0x40, 0x00, // 0x31 1
  0x72, 0x49, 0x49, 0x49, 0x46, // 0x32 2
  0x21, 0x41, 0x49, 0x4D, 0x33, // 0x33 3
  0x18, 0x14, 0x12, 0x7F, 0x10, // 0x34 4
  0x27, 0x45, 0x45, 0x45, 0x39, // 0x35 5
  0x3C, 0x4A, 0x49, 0x49, 0x31, // 0x36 6
  0x41, 0x21, 0x11, 0x09, 0x07, // 0x37 7
  0x36, 0x49, 0x49, 0x49, 0x36, // 0x38 8
  0x46, 0x49, 0x49, 0x29, 0x1E, // 0x39 9
  0x00, 0x00, 0x14, 0x00, 0x00, // 0x3A :
  0x00, 0x40, 0x34, 0x00, 0x00, // 0x3B ;
  0x00, 0x08, 0x14, 0x22, 0x41, // 0x3C <
  0x14, 0x14, 0x14, 0x14, 0x14, // 0x3D =
  0x00, 0x41, 0x22, 0x14, 0x08, // 0x3E >
  0x02, 0x01, 0x59, 0x09, 0x06, // 0x3F ?
  0x3E, 0x41, 0x5D, 0x59, 0x4E, // 0x40 @
  0x7C, 0x12, 0x11, 0x12, 0x7C, // 0x41 A
  0x7F, 0x49, 0x49, 0x49, 0x36, // 0x42 B 
  0x3E, 0x41, 0x41, 0x41, 0x22, // 0x43 C
  0x7F, 0x41, 0x41, 0x41, 0x3E, // 0x44 D
  0x7F, 0x49, 0x49, 0x49, 0x41, // 0x45 E
  0x7F, 0x09, 0x09, 0x09, 0x01, // 0x46 F
  0x3E, 0x41, 0x41, 0x51, 0x73, // 0x47 G
  0x7F, 0x08, 0x08, 0x08, 0x7F, // 0x48 H
  0x00, 0x41, 0x7F, 0x41, 0x00, // 0x49 I
  0x20, 0x40, 0x41, 0x3F, 0x01, // 0x4A J
  0x7F, 0x08, 0x14, 0x22, 0x41, // 0x4B K
  0x7F, 0x40, 0x40, 0x40, 0x40, // 0x4C L
  0x7F, 0x02, 0x1C, 0x02, 0x7F, // 0x4D M
  0x7F, 0x04, 0x08, 0x10, 0x7F, // 0x4E N
  0x3E, 0x41, 0x41, 0x41, 0x3E, // 0x4F O
  0x7F, 0x09, 0x09, 0x09, 0x06, // 0x50 P
  0x3E, 0x41, 0x51, 0x21, 0x5E, // 0x51 Q
  0x7F, 0x09, 0x19, 0x29, 0x46, // 0x52 R
  0x26, 0x49, 0x49, 0x49, 0x32, // 0x53 S
  0x03, 0x01, 0x7F, 0x01, 0x03, // 0x54 T
  0x3F, 0x40, 0x40, 0x40, 0x3F, // 0x55 U
  0x1F, 0x20, 0x40, 0x20, 0x1F, // 0x56 V
  0x3F, 0x40, 0x38, 0x40, 0x3F, // 0x57 W
  0x63, 0x14, 0x08, 0x14, 0x63, // 0x58 X
  0x03, 0x04, 0x78, 0x04, 0x03, // 0x59 Y
  0x61, 0x59, 0x49, 0x4D, 0x43, // 0x5A Z
  0x00, 0x7F, 0x41, 0x41, 0x41, // 0x5B [
  0x02, 0x04, 0x08, 0x10, 0x20, // 0x5C '\'
  0x00, 0x41, 0x41, 0x41, 0x7F, // 0x5D ]
  0x04, 0x02, 0x01, 0x02, 0x04, // 0x5E ^
  0x40, 0x40, 0x40, 0x40, 0x40, // 0x5F _
  0x00, 0x03, 0x07, 0x08, 0x00, // 0x60 `
  0x20, 0x54, 0x54, 0x78, 0x40, // 0x61 a
  0x7F, 0x28, 0x44, 0x44, 0x38, // 0x62 b
  0x38, 0x44, 0x44, 0x44, 0x00, // 0x63 c
  0x38, 0x44, 0x44, 0x28, 0x7F, // 0x64 d
  0x38, 0x54, 0x54, 0x54, 0x18, // 0x65 e
  0x00, 0x08, 0x7E, 0x09, 0x02, // 0x66 f
  // 0x18, 0xA4, 0xA4, 0x9C, 0x78, // 0x67 g (old)
  0x0C, 0x52, 0x52, 0x4E, 0x3C, // 0x67 g 
  0x7F, 0x08, 0x04, 0x04, 0x78, // 0x68 h
  0x00, 0x44, 0x7D, 0x40, 0x00, // 0x69 i
  0x20, 0x40, 0x40, 0x3D, 0x00, // 0x6A j
  0x7F, 0x10, 0x28, 0x44, 0x00, // 0x6B k
  0x00, 0x41, 0x7F, 0x40, 0x00, // 0x6C l
  0x7C, 0x04, 0x78, 0x04, 0x78, // 0x6D m
  0x7C, 0x08, 0x04, 0x04, 0x78, // 0x6E n
  0x38, 0x44, 0x44, 0x44, 0x38, // 0x6F o
  0xFC, 0x18, 0x24, 0x24, 0x18, // 0x70 p
  // 0x18, 0x24, 0x24, 0x18, 0xFC, // 0x71 q (old)
  0x0C, 0x12, 0x12, 0x0C, 0x7E, // 0x71 q
  0x7C, 0x08, 0x04, 0x04, 0x08, // 0x72 r
  0x48, 0x54, 0x54, 0x54, 0x24, // 0x73 s
  0x04, 0x04, 0x3F, 0x44, 0x24, // 0x74 t
  0x3C, 0x40, 0x40, 0x20, 0x7C, // 0x75 u
  0x1C, 0x20, 0x40, 0x20, 0x1C, // 0x76 v
  0x3C, 0x40, 0x30, 0x40, 0x3C, // 0x77 w
  0x44, 0x28, 0x10, 0x28, 0x44, // 0x78 x
  0x0C, 0x10, 0x10, 0x50, 0x3C, // 0x79 y
  0x44, 0x64, 0x54, 0x4C, 0x44, // 0x7A z
  0x00, 0x08, 0x36, 0x41, 0x00, // 0x7B {
  0x00, 0x00, 0x7F, 0x00, 0x00, // 0x7C |
  0x00, 0x41, 0x36, 0x08, 0x00, // 0x7D }
  0x02, 0x01, 0x02, 0x04, 0x02, // 0x7E ~
  0x3C, 0x26, 0x23, 0x26, 0x3C, // 0x7F house
  0x1E, 0xA1, 0xA1, 0x61, 0x12, // 0x81
  0x3A, 0x40, 0x40, 0x20, 0x7A, // 0x81 ü
  0x38, 0x54, 0x54, 0x55, 0x59, // 0x82
  0x21, 0x55, 0x55, 0x79, 0x41, // 0x83
  0x21, 0x54, 0x54, 0x78, 0x41, // 0x84 ä
  0x21, 0x55, 0x54, 0x78, 0x40, // 0x85
  0x20, 0x54, 0x55, 0x79, 0x40, // 0x86
  0x0C, 0x1E, 0x52, 0x72, 0x12, // 0x87
  0x39, 0x55, 0x55, 0x55, 0x59, // 0x88
  0x39, 0x54, 0x54, 0x54, 0x59, // 0x89
  0x39, 0x55, 0x54, 0x54, 0x58, // 0x8A
  0x00, 0x00, 0x45, 0x7C, 0x41, // 0x8B
  0x00, 0x02, 0x45, 0x7D, 0x42, // 0x8C
  0x00, 0x01, 0x45, 0x7C, 0x40, // 0x8D
  0xF0, 0x29, 0x24, 0x29, 0xF0, // 0x8E Ä
  0xF0, 0x28, 0x25, 0x28, 0xF0, // 0x8F
  0x7C, 0x54, 0x55, 0x45, 0x00, // 0x90
  0x20, 0x54, 0x54, 0x7C, 0x54, // 0x91
  0x7C, 0x0A, 0x09, 0x7F, 0x49, // 0x92
  0x32, 0x49, 0x49, 0x49, 0x32, // 0x93
  0x32, 0x48, 0x48, 0x48, 0x32, // 0x94 ö
  0x32, 0x4A, 0x48, 0x48, 0x30, // 0x95
  0x3A, 0x41, 0x41, 0x21, 0x7A, // 0x96
  0x3A, 0x42, 0x40, 0x20, 0x78, // 0x97
  0x00, 0x9D, 0xA0, 0xA0, 0x7D, // 0x98
  //0x39, 0x44, 0x44, 0x44, 0x39, // 0x99 Ö (old)
  0x3D, 0x42, 0x42, 0x42, 0x3D, // 0x99 Ö
  0x3D, 0x40, 0x40, 0x40, 0x3D, // 0x9A Ü
  0x3C, 0x24, 0xFF, 0x24, 0x24, // 0x9B cent
  0x48, 0x7E, 0x49, 0x43, 0x66, // 0x9C pound
  0x2B, 0x2F, 0xFC, 0x2F, 0x2B, // 0x9D
  0xFF, 0x09, 0x29, 0xF6, 0x20, // 0x9E
  0xC0, 0x88, 0x7E, 0x09, 0x03, // 0x9F
  0x20, 0x54, 0x54, 0x79, 0x41, // 0xA0
  0x00, 0x00, 0x44, 0x7D, 0x41, // 0xA1
  0x30, 0x48, 0x48, 0x4A, 0x32, // 0xA2
  0x38, 0x40, 0x40, 0x22, 0x7A, // 0xA3
  0x00, 0x7A, 0x0A, 0x0A, 0x72, // 0xA4
  0x7D, 0x0D, 0x19, 0x31, 0x7D, // 0xA5
  0x26, 0x29, 0x29, 0x2F, 0x28, // 0xA6
  0x26, 0x29, 0x29, 0x29, 0x26, // 0xA7
  0x30, 0x48, 0x4D, 0x40, 0x20, // 0xA8
  0x38, 0x08, 0x08, 0x08, 0x08, // 0xA9
  0x08, 0x08, 0x08, 0x08, 0x38, // 0xAA
  0x2F, 0x10, 0xC8, 0xAC, 0xBA, // 0xAB
  0x2F, 0x10, 0x28, 0x34, 0xFA, // 0xAC
  0x00, 0x00, 0x7B, 0x00, 0x00, // 0xAD exclamation mark upside down
  0x08, 0x14, 0x2A, 0x14, 0x22, // 0xAE less less
  0x22, 0x14, 0x2A, 0x14, 0x08, // 0xAF greater greater
  0xAA, 0x00, 0x55, 0x00, 0xAA, // 0xB0  
  0xAA, 0x55, 0xAA, 0x55, 0xAA, // 0xB1
  0xAA, 0x55, 0xAA, 0x55, 0xAA, // 0xB2
  0x00, 0x00, 0x00, 0xFF, 0x00, // 0xB3
  0x10, 0x10, 0x10, 0xFF, 0x00, // 0xB4
  0x14, 0x14, 0x14, 0xFF, 0x00, // 0xB5
  0x10, 0x10, 0xFF, 0x00, 0xFF, // 0xB6
  0x10, 0x10, 0xF0, 0x10, 0xF0, // 0xB7
  0x14, 0x14, 0x14, 0xFC, 0x00, // 0xB8
  0x14, 0x14, 0xF7, 0x00, 0xFF, // 0xB9
  0x00, 0x00, 0xFF, 0x00, 0xFF, // 0xBA
  0x14, 0x14, 0xF4, 0x04, 0xFC, // 0xBB
  0x14, 0x14, 0x17, 0x10, 0x1F, // 0xBC
  0x10, 0x10, 0x1F, 0x10, 0x1F, // 0xBD
  0x14, 0x14, 0x14, 0x1F, 0x00, // 0xBE
  0x10, 0x10, 0x10, 0xF0, 0x00, // 0xBF
  0x00, 0x00, 0x00, 0x1F, 0x10, // 0xC0
  0x10, 0x10, 0x10, 0x1F, 0x10, // 0xC1
  0x10, 0x10, 0x10, 0xF0, 0x10, // 0xC2
  0x00, 0x00, 0x00, 0xFF, 0x10, // 0xC3
  0x10, 0x10, 0x10, 0x10, 0x10, // 0xC4
  0x10, 0x10, 0x10, 0xFF, 0x10, // 0xC5
  0x00, 0x00, 0x00, 0xFF, 0x14, // 0xC6
  0x00, 0x00, 0xFF, 0x00, 0xFF, // 0xC7
  0x00, 0x00, 0x1F, 0x10, 0x17, // 0xC8
  0x00, 0x00, 0xFC, 0x04, 0xF4, // 0xC9
  0x14, 0x14, 0x17, 0x10, 0x17, // 0xCA
  0x14, 0x14, 0xF4, 0x04, 0xF4, // 0xCB
  0x00, 0x00, 0xFF, 0x00, 0xF7, // 0xCC
  0x14, 0x14, 0x14, 0x14, 0x14, // 0xCD
  0x14, 0x14, 0xF7, 0x00, 0xF7, // 0xCE
  0x14, 0x14, 0x14, 0x17, 0x14, // 0xCF
  0x10, 0x10, 0x1F, 0x10, 0x1F, // 0xD0
  0x14, 0x14, 0x14, 0xF4, 0x14, // 0xD1
  0x10, 0x10, 0xF0, 0x10, 0xF0, // 0xD2
  0x00, 0x00, 0x1F, 0x10, 0x1F, // 0xD3
  0x00, 0x00, 0x00, 0x1F, 0x14, // 0xD4
  0x00, 0x00, 0x00, 0xFC, 0x14, // 0xD5
  0x00, 0x00, 0xF0, 0x10, 0xF0, // 0xD6
  0x10, 0x10, 0xFF, 0x10, 0xFF, // 0xD7
  0x14, 0x14, 0x14, 0xFF, 0x14, // 0xD8
  0x10, 0x10, 0x10, 0x1F, 0x00, // 0xD9
  0x00, 0x00, 0x00, 0xF0, 0x10, // 0xDA
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0xDB
  0xF0, 0xF0, 0xF0, 0xF0, 0xF0, // 0xDC
  0xFF, 0xFF, 0xFF, 0x00, 0x00, // 0xDD
  0x00, 0x00, 0x00, 0xFF, 0xFF, // 0xDE
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F, // 0xDF
  0x38, 0x44, 0x44, 0x38, 0x44, // 0xE0 alpha
  0x7C, 0x2A, 0x2A, 0x3E, 0x14, // 0xE1 beta / ß
  0x7E, 0x02, 0x02, 0x06, 0x06, // 0xE2 Gamma
  0x02, 0x7E, 0x02, 0x7E, 0x02, // 0xE3 Pi
  0x63, 0x55, 0x49, 0x41, 0x63, // 0xE4 Sigma
  0x38, 0x44, 0x44, 0x3C, 0x04, // 0xE5 sigma
  0x40, 0x7E, 0x20, 0x1E, 0x20, // 0xE6 mu
  0x06, 0x02, 0x7E, 0x02, 0x02, // 0xE7 tau
  0x99, 0xA5, 0xE7, 0xA5, 0x99, // 0xE8 Phi
  0x1C, 0x2A, 0x49, 0x2A, 0x1C, // 0xE9 theta
  0x4C, 0x72, 0x01, 0x72, 0x4C, // 0xEA Omega
  0x30, 0x4A, 0x4D, 0x4D, 0x30, // 0xEB delta
  0x30, 0x48, 0x78, 0x48, 0x30, // 0xEC phi?
  0xBC, 0x62, 0x5A, 0x46, 0x3D, // 0xED empty set
  0x3E, 0x49, 0x49, 0x49, 0x00, // 0xEE epsilon
  0x7E, 0x01, 0x01, 0x01, 0x7E, // 0xEF
  0x2A, 0x2A, 0x2A, 0x2A, 0x2A, // 0xF0 eqivalence
  0x44, 0x44, 0x5F, 0x44, 0x44, // 0xF1 plus minus
  0x40, 0x51, 0x4A, 0x44, 0x40, // 0xF2 greater or equal
  0x40, 0x44, 0x4A, 0x51, 0x40, // 0xF3 less or equal
  0x00, 0x00, 0xFF, 0x01, 0x03, // 0xF4
  0xE0, 0x80, 0xFF, 0x00, 0x00, // 0xF5
  0x08, 0x08, 0x6B, 0x6B, 0x08, // 0xF6 divided by
  0x36, 0x12, 0x36, 0x24, 0x36, // 0xF7 approx
  0x06, 0x0F, 0x09, 0x0F, 0x06, // 0xF8 degree
  0x00, 0x00, 0x18, 0x18, 0x00, // 0xF9 big dot
  0x00, 0x00, 0x10, 0x10, 0x00, // 0xFA small dot
  0x30, 0x40, 0xFF, 0x01, 0x01, // 0xFB square root
  0x00, 0x1F, 0x01, 0x01, 0x1E, // 0xFC
  0x00, 0x19, 0x1D, 0x17, 0x12, // 0xFD power of two
  0x00, 0x3C, 0x3C, 0x3C, 0x3C, // 0xFE blob
  //0x00, 0x00, 0x00, 0x60, 0x60, // // 0xFF: Decimal point: 4 points
  0x00, 0x00, 0x00, 0x00, 0x40, // // 0xFF: Decimal point, only 1 point
};

void setup_pins() {
  DPRINTLN("VQC10 Clock - setup 74HC595 pins");
  pinMode(latchPin, OUTPUT);  regPin(latchPin,"latchPin");
  pinMode(dataPin,OUTPUT);    regPin(dataPin,"dataPin");
  pinMode(clockPin,OUTPUT);   regPin(clockPin,"clockPin");
  pinMode(D5pin,INPUT);       regPin(D5pin,"D5pin");
  digitsOnly = false;
  asciiConvert[16] = 9;
}

void inline shiftOutForward(byte myDataOut) {
  // This shifts 8 bits out MSB first, on the rising edge of the clock, clock idles low
  for (int i=7; i>=0; i--)  {
    digitalWrite(clockPin, 0);
    digitalWrite(dataPin, myDataOut & (1<<i)); //register shifts bits on upstroke of clock pin
    digitalWrite(clockPin, 1);
    digitalWrite(dataPin, 0);
  }
}

void inline shiftOutReverse(byte myDataOut) {
  // This shifts 8 bits out LSB first, on the rising edge of the clock, clock idles low
  for (int i=0; i<8; i++)  {
    digitalWrite(clockPin, 0);
    digitalWrite(dataPin, myDataOut & (1<<i)); //register shifts bits on upstroke of clock pin
    digitalWrite(clockPin, 1);
    digitalWrite(dataPin, 0);
  }
}



#define PWMrefresh 8000l

void ICACHE_RAM_ATTR writeDisplay() {}


int PWM_min = 0;
int PWM_max = 800;

void writeDisplay2() {
static  int PWMtimeBrightness;
//static uint32_t refresh = 0;
//static uint32_t runtim = 0;
//static uint32_t laststart = 0;
boolean offState;
int brightness;
int iRow,loopMax;
  
  
  //refresh = millis()-laststart;
  //laststart = millis();
  //runtim = laststart;

  brightness = displayON ? prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  if (autoBrightness && displayON) {  
    PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
    }
  else
    PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
  
  iRow = 0;
  loopMax = maxDigits;
  do {
  offState = ( (brightness == 0) || (!radarON) || (iRow == maxDigits));  
 
  for (int i=0;i<NUMDIGITS;i++) { //fill one line with dots
      shiftOutForward(0xFF);     //deselect Z
      shiftOutForward(~(1<<i));  //select digit
      shiftOutReverse(offState ? 0 : dat[i][iRow]);  // set data line
      digitalWrite(latchPin, HIGH); //Write shift register to latch
      digitalWrite(latchPin, LOW);   
      asm volatile ("nop");
      
      //Finish latching
      shiftOutForward(0xFF);   //deselect Z
      shiftOutForward(0xFF);  //All digit select lines are HIGH
      shiftOutReverse(offState ? 0 : dat[i][iRow]);  //keep data line
      digitalWrite(latchPin, HIGH); //Write shift register to latch
      digitalWrite(latchPin, LOW); 
      asm volatile ("nop");
  }

  shiftOutForward(~(1<<iRow)); //enable row
  shiftOutForward(0xFF);     //All digit select lines are HIGH
  shiftOutForward(0);        //anything
  digitalWrite(latchPin, HIGH); //Write shift register to latch
  digitalWrite(latchPin, LOW);
  if (!offState) 
    delayMicroseconds(PWMtimeBrightness);
  else
    delayMicroseconds(PWM_max-PWMtimeBrightness);  
  
  iRow++;
  } while(iRow<loopMax);
    
  //runtim = millis()-runtim;
  //DPRINT(refresh); DPRINT("/"); DPRINTLN(runtim); 
  yield();
}


void show(byte c){
  byte loccol[NUMCOLS];
  for (byte i=0; i < NUMCOLS; i++)
    loccol[i] = _font[c*NUMCOLS+i];
  transposePattern(loccol,row);
}

void transposePattern(byte cols[], volatile byte rows[]){
  byte cmask;
  cmask = (_upsidedown ? 0x40 : 1);
  for (byte r=0; r < NUMROWS; r++) {
    rows[r] = 0;
    for (byte c=0; c < NUMCOLS; c++) {
      if (cmask&cols[(_upsidedown ? (NUMCOLS-c-1) : c)]) rows[r] |= 1;
      rows[r] = rows[r] << 1;
    }
    if (_upsidedown) cmask = cmask >> 1;
    else cmask = cmask << 1;
    rows[r] = rows[r] << (7-NUMCOLS);
  }
}

void bPrint(byte in) {
  for (int i=7;i>=0;i--) {
    boolean b = in & (1<<i);
    DPRINT(b ? '1' : '0');
  }
}

void printBitmap() {  //print bitmap for testing
  for (int j=0;j<NUMROWS;j++) {
    for (int i=0;i<NUMDIGITS;i++) {
      bPrint(dat[i][j]); Serial.print(" ");
    }
    DPRINTLN(" ");
  }
  DPRINTLN("__________________________________");
}


void clearTubes() {}

void writeDisplaySingle() {
  char dispChar;
  byte tubeShift[] = {3,2,1,0,7,6,5,4}; 
  byte a;
  for (int i=0;i<NUMDIGITS;i++) {   //generate new line
    dispChar = digit[tubeShift[i]];
    if (dispChar<sizeof(asciiConvert)) dispChar = asciiConvert[(byte)dispChar]; 
    show(dispChar); 
    for (int k=0;k<NUMROWS;k++) {
      oldDat[i][k] = row[k];
    }
    if (digitDP[tubeShift[i]]) {
      show(DPchar); 
      for (int k=0;k<NUMROWS;k++) {
        oldDat[i][k] |= row[k];
      }
    }
  }
  for (int i=0;i<NUMDIGITS;i++) {  //generate old line
    dispChar = newDigit[tubeShift[i]];
    if (dispChar<sizeof(asciiConvert)) dispChar = asciiConvert[(byte)dispChar]; 
    show(dispChar); 
    for (int k=0;k<NUMROWS;k++) {
      newDat[i][k] = row[k];
    }
    if (digitDP[tubeShift[i]]) {
      show(DPchar); 
      for (int k=0;k<NUMROWS;k++) {
        newDat[i][k] |= row[k];
      }
    }
  }

  for (int i=0;i<NUMDIGITS;i++) {  //combine animation
    a = animMask[tubeShift[i]]/2;   if (a>7) a=7;
    for (int k=0;k<NUMROWS;k++) {
      if (a==0) dat[i][k] = oldDat[i][k];
      else {   //Animation!!!
        if ((a+k)<NUMROWS) { 
          dat[i][k] = oldDat[i][k+a];   
        }
        else {
          dat[i][k] = newDat[i][(k+a)-NUMROWS];
        }
      }  //endif Animation!!
    } //end for i
  } //end for 
  //printBitmap();
}

#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ZM1500.ino"
// ZM1500 Clock by UNFI
//  4x 74HC595N shift register + optocouplers

#ifdef ZM1500_ESP32

char tubeDriver[] = "ZM1500_ESP32";
#if defined(ESP32) 
#else
  #error "Only ESP32 Board is supported!"  
#endif

int DRAM_ATTR PWMrefresh=11000;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
int DRAM_ATTR PWM_min = 2000;
int DRAM_ATTR PWM_max = 10000;

//#define dataPin  14  //D5
//#define latchPin 27 //12  //D6
//#define clkPin   26 //13  //D7

byte DRAM_ATTR charDefinition[] = {
                   B11111100,   //0: abcdef
                   B01100000,   //1: bc 
                   B11011010,   //2: abdeg
                   B11110010,   //3: abcdg
                   B01100110,   //4: bcfg
                   B10110110,   //5: acdfg
                   B10111110,   //6: acdefg
                   B11100000,   //7: abc
                   B11111110,   //8: abcdefg
                   B11110110,   //9: abcdfg
                   B00000000,   // : BLANK (10)
                   B00000010,   //-: minus (11)
                   B00000001,   // decimal point (12)
                   B11101110,   // A  abcefg (13)
                   B11001110,   // P  abefg (14)
                   B10011100,   // C  adef (15)
                   B11000110,   //grad  (upper circle) abfg (16)
                   B10110100,   //%  acdf  (17)
                   B00111010,   //lower circle cdeg  (18)                  
                   B01100000,   //I  bc    (19)
                   B10001110    //F  aefg  (20)
};

#define MAXCHARS sizeof(charDefinition)
#define MAXSEGMENTS sizeof(segmentEnablePins)
int maxDigits = sizeof(digitEnablePins);

uint32_t DRAM_ATTR charTable[MAXCHARS];    //generated pin table from segmentDefinitions
uint32_t DRAM_ATTR segmentEnableBits[MAXSEGMENTS];   //bitmaps, generated from EnablePins tables
uint32_t DRAM_ATTR animationMaskBits[5];
uint32_t DRAM_ATTR digitEnableBits[12];

void IRAM_ATTR writeDisplaySingle() { }   

void setup_pins(){
  DPRINT("ZM1500 clock ");  DPRINT(maxDigits); DPRINTLN(" digits - setup pins");
  pinMode(dataPin, OUTPUT); regPin(dataPin,"dataPin"); 
  pinMode(latchPin,OUTPUT); regPin(latchPin,"latchPin"); 
  pinMode(clkPin,OUTPUT);   regPin(clkPin,"clkPin"); 
  digitalWrite(latchPin,LOW); 
  digitalWrite(clkPin,LOW);
  generateBitTable();
  startTimer();
}

void IRAM_ATTR writeDisplay(){  //void IRAM_ATTR  writeDisplay(){

  static DRAM_ATTR int pos = maxDigits;
  static DRAM_ATTR boolean state=0;
  static DRAM_ATTR int brightness;
  static DRAM_ATTR uint32_t PWMtimeBrightness=PWM_min;
  uint32_t timer = PWMrefresh;
  uint32_t val = 0;
  
  portENTER_CRITICAL_ISR(&timerMux);
  noInterrupts();
  intCounter++;
  
  if (state) {  //ON state
    pos++;  
    if (pos>maxDigits-1)  {   //go to the tube#0
      pos = 0; 
      brightness = displayON ? prm.dayBright : prm.nightBright;
      if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety
      if (autoBrightness && displayON) {   //change brightness only on the tube#0
        PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
        }
      else
        PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
    }  
    val = uint32_t(charTable[digit[pos]]);  //the full bitmap to send to MAX chip
    if (animMask[pos]>0) val &= uint32_t(animationMaskBits[animMask[pos]-1]);  //animationMode 6, mask characters from up to down and back
    if (digitDP[pos]) val = val | uint32_t(charTable[12]);    //Decimal Point
        
    timer = PWMtimeBrightness;
    timerON = timer;   //debug only
    timerOFF = PWMrefresh-PWMtimeBrightness;  //debug only
  }
  else {  //OFF state
    timer = PWMrefresh-PWMtimeBrightness;
    val = 0;
  }
  
  if (timer<500) timer = 500;  //safety only...
  if ( (brightness == 0) || (!radarON))  val = 0; //blank digit
  val = val | digitEnableBits[pos];
  
  for (int i=0; i<20; i++)  {
    if (val & uint32_t(1 << i))   //reverse: (1 << (19 - i))) 
      digitalWrite(dataPin, HIGH);  
    else 
      digitalWrite(dataPin, LOW);    
    digitalWrite(clkPin,HIGH);  //asm volatile ("nop");  //Clock impulse
    digitalWrite(clkPin,LOW);   //asm volatile ("nop");    
  } //end for      

  digitalWrite(latchPin,HIGH);  //asm volatile ("nop");
  digitalWrite(latchPin,LOW);
  state = !state;  
    
  portEXIT_CRITICAL_ISR(&timerMux);   
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
}


void generateBitTable() {
uint32_t out;

  DPRINTLN("--- Generating segment pins bitmap ---");
  for (int i=0;i<MAXSEGMENTS;i++) {
    segmentEnableBits[i] = uint32_t(1<<segmentEnablePins[i]);
    DPRINT(i); DPRINT(": "); DPRINTLN(segmentEnableBits[i],BIN);
  }
  animationMaskBits[0] = uint32_t(1<<segmentEnablePins[0]);  //a
  animationMaskBits[1] = animationMaskBits[0] | uint32_t(1<<segmentEnablePins[1]) | uint32_t(1<<segmentEnablePins[5]);  //bf
  animationMaskBits[2] = animationMaskBits[1] | uint32_t(1<<segmentEnablePins[6]);  //g
  animationMaskBits[3] = animationMaskBits[2] | uint32_t(1<<segmentEnablePins[4]) | uint32_t(1<<segmentEnablePins[2]);  //ec
  animationMaskBits[4] = animationMaskBits[3] | uint32_t(1<<segmentEnablePins[3]);  //d
  DPRINTLN("--- Animation mask ---");
  for (int i=0;i<5;i++) {
    animationMaskBits[i] = ~animationMaskBits[i]; //invert bits
    DPRINTLN(animationMaskBits[i],HEX);
  }

DPRINTLN("---- Generated Character / Pins table -----");
  for (int i=0;i<MAXCHARS;i++) {
    out = 0;
    DPRINT(i); DPRINT(":  ");
    DPRINT(charDefinition[i],BIN);  DPRINT(" = ");
    for (int j=0;j<=7;j++)   //from a to g
      if ((charDefinition[i] & 1<<(7-j)) != 0) {
        out = out | segmentEnableBits[j]; //DPRINT("1"); 
        }
    //else        DPRINT("0");
    //DPRINT("  >> ");  
    
    charTable[i] = out;
    DPRINTLN(charTable[i],BIN);
  }  //end for

  DPRINTLN("--- Generating digit pins bitmap ---");
  for (int i=0;i<maxDigits;i++) {
    digitEnableBits[i] = uint32_t(1 << uint32_t(digitEnablePins[i]+8));
    DPRINT(i); DPRINT(": "); DPRINT(digitEnableBits[i],HEX); DPRINTLN(" ");
  }

  DPRINTLN("--- DEMO ---");
  for (int i=0;i<maxDigits;i++) {
    uint32_t val = uint32_t(charTable[i]);  //the full bitmap to send to MAX chip
    val = val | digitEnableBits[i];
    DPRINT(i); DPRINT(": "); DPRINTLN(val,HEX);
  }  
}

void IRAM_ATTR clearTubes() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dallas_temp.ino"
#ifdef USE_DALLAS_TEMP   //DS18B20 sensor

// Change HERE   0<>1, to swap sensors!!!
#define SENSOR1 0
#define SENSOR2 1
#define TEMPERATURE_PRECISION 12

#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(TEMP_DALLAS_PIN);        // Set up a OneWire instance to communicate with OneWire devices
DallasTemperature tempSensors(&oneWire); // Create an instance of the temperature sensor class
DeviceAddress thermometer1, thermometer2;
boolean requested = false;
unsigned long lastRequest = millis();
boolean DallasOK = true;   //is any measure from Dallas thermometer?
float lastTemperature = 0;
char lastTemperatureStr[5] = "----";
unsigned long intervalTemp = 30000;      // Do a temperature measurement every 30sec
unsigned long DS_delay = 800;         // Reading the temperature from the DS18x20 can take up to 750ms

byte d1Ptr = 0;
byte d2Ptr = 0;

void setupDallasTemp() {
  regPin(TEMP_DALLAS_PIN,"TEMP_DALLAS_PIN"); 
  pinMode(TEMP_DALLAS_PIN,OUTPUT);
  #if defined(ESP8266)
    oneWire.reset();
    delay(200);  //200ms
  #endif  
  tempSensors.begin();                     // Start the temperature sensor  
  tempSensors.setResolution(TEMPERATURE_PRECISION);
  tempSensors.setWaitForConversion(false); // Don't block the program while the temperature sensor is reading
  delay(100);  //100ms
  tempSensors.requestTemperatures();
  delay(800);
  int counter = 0;
  while ((tempSensors.getDeviceCount() == 0) && (counter<5)) {
    if (counter>3) {
      DPRINT("No DS18x20 temperature sensor found on GPIO"); DPRINTLN(TEMP_DALLAS_PIN); 
      resetSensors();
    }
    counter++;
    delay(100);
  }  //end while
  
  useDallasTemp = tempSensors.getDeviceCount();
  if (useDallasTemp>0) {
    d1Ptr = useTemp;  //remember the pointers of sensors
    useTemp++; 
    if (useDallasTemp>1) {
      d2Ptr = useTemp; 
      useTemp++;
    }
    DPRINT("DS18B20 sensors found:"); DPRINTLN(useDallasTemp);
    if (!tempSensors.getAddress(thermometer1, 0)) DPRINTLN("Unable to find address for Device 0");
    if ((useDallasTemp>1) && !tempSensors.getAddress(thermometer2, 1)) DPRINTLN("Unable to find address for Device 1");
    requestDallasTemp(true);
    delay(200);
    getTemp();
  }
}

void requestDallasTemp(boolean force) {
    if (EEPROMsaving) return;
    
    //if (force || (!requested && ((millis()-lastRequest) > intervalTemp))) {       //request a new reading
    if (force || (!requested && (second()==prm.tempStart-1))) {       //request a new reading
      //disableDisplay();
      tempSensors.requestTemperatures(); // Request the temperature from the sensor (it takes some time to read it)
      //enableDisplay(0);
      requested = true;
      lastRequest = millis();
      DPRINTLN("Temperature requested");
      }
}

void getTemp() {              //get the earlier requested temperature data
static int errors = 0;  
float tmp_temp;

    if (!requested) return;
    if ((millis()- lastRequest) < DS_delay) return;  // 1000 ms after requesting the temperature
    if (EEPROMsaving) return;

    requested = false;
    if (tempSensors.getDeviceCount() == 0) {
          temperature[d1Ptr] = 0; temperature[d2Ptr] = 0;
          DallasOK = false;
          resetSensors();
          DPRINTLN("Missing TEMP sensor!");
          lastRequest -= intervalTemp;  //retry ASAP!
          }
    else {    
          delay(10);
          //disableDisplay();
          tmp_temp = tempSensors.getTempCByIndex(0); // Get the temperature from the sensor
          //enableDisplay(0);
          if (tmp_temp > -127) {
            tmp_temp = round(tmp_temp * 10.0) / 10.0; // round temperature to 1 digits
            DallasOK = true;
            temperature[0] = tmp_temp;     
            DPRINT("Dallas#1 temp:"); DPRINTLN(temperature[d1Ptr]);
            dtostrf(temperature[0], 4, 1, lastTemperatureStr);
            errors = 0;
            if (useDallasTemp>1) {
              //disableDisplay();
              temperature[d2Ptr] = round(tempSensors.getTempCByIndex(1) * 10.0) / 10.0; // round temperature to 1 digits
              //enableDisplay(0);
              DPRINT("Dallas#2 temp:"); DPRINTLN(temperature[d2Ptr]);
            }
            }
          else {
            DPRINT("Temp sensor error!!!");  
            DPRINTLN(tmp_temp);
            errors++;
            lastRequest -= intervalTemp;  //retry ASAP!
            resetSensors();
            }
    }
  if (errors>20) 
      useDallasTemp = 0; //forget BAD sensor!!! 
}

void resetSensors() {
  if (EEPROMsaving) return;
  DPRINTLN("Reset sensors...");
  //#if defined(ESP8266)
    oneWire.reset();
  //#endif
  delay(500);  //200ms

  tempSensors.begin(); //try to restart sensor
  tempSensors.setWaitForConversion(false);
  tempSensors.getDeviceCount();
  delay(500);  //200ms
}
//-----------------------------------------------------------------------------------------------------
#else
void setupDallasTemp() {}
void requestDallasTemp(boolean force) {}
void getTemp() {}
void resetSensors() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dummy.ino"
#ifdef DUMMY

//Dummy clock driver for sensor box / mqtt sensors

char tubeDriver[] = "DUMMY";
int maxDigits = 6;

void setup_pins() {
  DPRINTLN("Dummy display: no pins are used");
}

#if defined(ESP32)
void IRAM_ATTR writeDisplay() {} 
#else
void ICACHE_RAM_ATTR writeDisplay(){}
#endif

void clearTubes() {}
void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\gesture_apds9960.ino"
#if defined(USE_APDS9960_GESTURE)

#include <SparkFun_APDS9960.h>

SparkFun_APDS9960 gestureSensor;
static bool gestureSensorReady = false;
static unsigned long lastGestureMs = 0;
static const unsigned long GESTURE_DEBOUNCE_MS = 350;

#ifndef APDS9960_INT_PIN
  #define APDS9960_INT_PIN -1
#endif

void setupGestureSensor() {
  DPRINTLN("Setup APDS9960 gesture sensor...");

  #if APDS9960_INT_PIN >= 0
    pinMode(APDS9960_INT_PIN, INPUT_PULLUP);
    regPin(APDS9960_INT_PIN, "APDS9960_INT_PIN");
  #endif

  if (!gestureSensor.init()) {
    DPRINTLN("APDS9960 init failed.");
    gestureSensorReady = false;
    return;
  }

  if (!gestureSensor.enableGestureSensor(true)) {
    DPRINTLN("APDS9960 gesture mode enable failed.");
    gestureSensorReady = false;
    return;
  }

  gestureSensorReady = true;
  DPRINTLN("APDS9960 gesture ready.");
}

bool isGestureSensorPresent() {
  return gestureSensorReady;
}

void processGestureSensor() {
  if (!gestureSensorReady) return;

  unsigned long nowMs = millis();
  if ((nowMs - lastGestureMs) < GESTURE_DEBOUNCE_MS) return;

  if (!gestureSensor.isGestureAvailable()) return;

  int g = gestureSensor.readGesture();
  if (g == DIR_NONE || g == DIR_NEAR || g == DIR_FAR) return;

  switch (g) {
    case DIR_UP:
      executeGestureMappedAction(0);
      DPRINTLN("[GESTURE] UP");
      break;
    case DIR_DOWN:
      executeGestureMappedAction(1);
      DPRINTLN("[GESTURE] DOWN");
      break;
    case DIR_LEFT:
      executeGestureMappedAction(2);
      DPRINTLN("[GESTURE] LEFT");
      break;
    case DIR_RIGHT:
      executeGestureMappedAction(3);
      DPRINTLN("[GESTURE] RIGHT");
      break;
    default:
      return;
  }

  lastGestureMs = nowMs;
}

#else

void setupGestureSensor() {}
void processGestureSensor() {}
bool isGestureSensorPresent() { return false; }

#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\gps.ino"
#ifdef USE_GPS

#include <TinyGPS++.h>                                  // Tiny GPS Plus Library

#if defined(ESP8266)  
  #include "SoftwareSerial.h"     // Software Serial Library so we can use other Pins for communication with the GPS module
  static int RXPin = 3;     //RX pin  -  remove wire, when uploading program to 8266          
  static int TXPin = -1;    //not used   
  SoftwareSerial ss(RXPin, TXPin);  
#else
  #include <HardwareSerial.h>   //on ESP32 no SoftwareSerial is needed
  static int RXPin = 25;     //RX pin 
  static int TXPin = -1;    //not used   
  HardwareSerial ss(1);  
#endif
           
// The serial connection to the GPS device
static uint32_t GPSBaud = 9600;                   // Ublox GPS default Baud Rate is 9600
TinyGPSPlus gps;                                  // Create an Instance of the TinyGPS++ object called gps

void setupGPS() { 
  delay(1500);     
  DPRINTLN("Starting GPS..."); 
  if (RXPin>=0) regPin(RXPin,"RXPin");   
  if (TXPin>=0) regPin(TXPin,"TXPin");
  #if defined(ESP8266)
    ss.begin(GPSBaud);  
    //ss.begin(GPSBaud, SWSERIAL_8N1, RXPin, TXPin, false); 
  #else        
    ss.begin(GPSBaud, SERIAL_8N1, RXPin);    
  #endif
  delay(200);
  GPSexist = true;
  smartDelay(500);
  getGPS(); 
}

boolean getGPS(){ 
static unsigned long lastRun = 0;
boolean res = false;

   if ((lastRun!=0) && (millis() - lastRun) < 10000) //refresh rate
      return(res);   
      
   lastRun = millis();
   smartDelay(500);
   if (gps.time.isValid() && gps.date.isValid() && gps.time.isUpdated())  {
    if (gps.date.year()>2020) {
      DPRINTLN("Time refreshed from GPS");
      // Set time from GPS (GPS provides UTC)
      setTime(gps.time.hour(),gps.time.minute(),gps.time.second(),gps.date.day(),gps.date.month(),gps.date.year());
      // Convert UTC to local time based on enableDST setting
      if (prm.enableDST) {
        // Auto DST: use Timezone library for automatic DST handling
        setTime(myTZ.toLocal(now()));
      } else {
        // No DST: apply standard offset only
        setTime(now() + (prm.utc_offset * 3600));
      }
      lastTimeUpdate = millis();
    }
    #ifdef DEBUG
      printGPS();
    #endif
    
    res = true;
   } //endif valid date&time
   return (res);
}

void printGPS() {
  DPRINT("Latitude  : ");  DPRINT(gps.location.lat(), 5);  DPRINT("  Longitude : ");  DPRINTLN(gps.location.lng(), 4);
  DPRINT("Satellites: ");  DPRINT(gps.satellites.value());  DPRINT("  Elevation : ");  DPRINT(gps.altitude.feet()); DPRINTLN("ft"); 
  DPRINT("GPS Time UTC  : ");  DPRINT(gps.time.hour());   // GPS time UTC 
  DPRINT(":");  DPRINT(gps.time.minute());                // Minutes
  DPRINT(":");  DPRINTLN(gps.time.second());              // Seconds
}


void smartDelay(unsigned long ms)  {              // This custom version of delay() ensures that the gps object is being "fed".
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

//------------------------------------------------------------------------------------------

#else
void setupGPS() {GPSexist = false;}
boolean getGPS() {return(false);}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
#ifdef USE_MQTT
//Clock can receive data from a MASTER_CLOCK and send data to other clocks
//_______________________________________________________________________________________________________________________
//To use MQTT services the following settings should be defined in clocks.h
//#define USE_MQTT
//#define MQTT_PREFIX "clockforgeos"
//#define USE_MASTER_CLOCK  //enable it, if you want to get any data from MASTER CLOCK. This will be the sensor#0
//#define USE_MASTER_TEMP   //enable it, if you want to get temperature from MASTER CLOCK
//#define USE_MASTER_HUMID  //enable it, if you want to get humidity from MASTER CLOCK
//#define USE_MASTER_RADAR  //enable it, if you want to get radar from MASTER CLOCK
//#define MASTER_TEMPERATURE_TOPIC "homeassistant/sensor/10521c5e14c4/temperature/state"
//#define MASTER_HUMIDITY_TOPIC    "homeassistant/sensor/10521c5e14c4/humidity/state"
//#define MASTER_RADAR_TOPIC       "homeassistant/sensor/10521c5e14c4/radar/state"
//_______________________________________________________________________________________________________________________

//To subscribe to my sensors: You can check it with Chrome browser + MQTTlens addon)  MAC_ADDRESS: for example: 3e090b3e090b
// homeassistant/sensor/3e090b3e090b/temperature/value
// homeassistant/sensor/3e090b3e090b/humidity/value
// homeassistant/sensor/3e090b3e090b/pressure/value
// homeassistant/sensor/3e090b3e090b/lux/value

// old solution, now moved to prm.mqttBrokerAddr, available on clock's web page
//#define BROKER_ADDR      IPAddress(192,168,1,241) 
//#define BROKER_USERNAME  ""        //moved to prm.mqttBrokerUser
//#define BROKER_PASSWORD  ""        //moved to mqttBrokerPsw
//_______________________________________________________________________________________________________________________

#include <ArduinoHA.h>   //https://github.com/dawidchyrzynski/arduino-home-assistant
//https://pubsubclient.knolleary.net/     https://github.com/knolleary/pubsubclient

#if !defined(MQTT_PREFIX)
  #define MQTT_PREFIX "clockforgeos"
#endif

byte mac[6];
char masterTemperatureTopic[80] = "";
char masterHumidityTopic[80] = "";
char masterRadarTopic[80] = "";
char masterLuxTopic[80] = "";
char masterPressureTopic[80] = "";

byte MASTERtempPtr=0;
byte MASTERhumidPtr=0;
bool mqttMasterTempEnabled = false;
bool mqttMasterHumidEnabled = false;
bool mqttMasterRadarEnabled = false;
bool mqttMasterLuxEnabled = false;
bool mqttMasterPressureEnabled = false;

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);
//Sensors of local clock, to send data:
HASensor mqttTemp("temperature");
HASensor mqttHumid("humidity");
HASensor mqttPress("pressure");
HASensor mqttLux("lux");
HASensor mqttRadar("radar");
bool mqttConnected = false;

volatile unsigned long mqttLastInboundMs = 0;

static float parseNumericPayload(const char* s) {
  if (!s) return 0.0f;
  while (*s && !((*s >= '0' && *s <= '9') || *s == '-' || *s == '+')) s++;
  return atof(s);
}

static bool parseTemperaturePayload(const char* s, float& out) {
  if (!s) return false;
  if (s[0] == '{') {
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      if (doc["temperature"].is<float>() || doc["temperature"].is<int>()) { out = doc["temperature"].as<float>(); return true; }
      if (doc["temp"].is<float>() || doc["temp"].is<int>()) { out = doc["temp"].as<float>(); return true; }
      if (doc["tC"].is<float>() || doc["tC"].is<int>()) { out = doc["tC"].as<float>(); return true; }
      if (doc["tmp"]["tC"].is<float>() || doc["tmp"]["tC"].is<int>()) { out = doc["tmp"]["tC"].as<float>(); return true; }
      if (doc["tmp"]["value"].is<float>() || doc["tmp"]["value"].is<int>()) { out = doc["tmp"]["value"].as<float>(); return true; }
      if (doc["value"].is<float>() || doc["value"].is<int>()) { out = doc["value"].as<float>(); return true; }
    }
  }
  out = parseNumericPayload(s);
  return true;
}

static bool parseHumidityPayload(const char* s, float& out) {
  if (!s) return false;
  if (s[0] == '{') {
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      if (doc["humidity"].is<float>() || doc["humidity"].is<int>()) { out = doc["humidity"].as<float>(); return true; }
      if (doc["hum"].is<float>() || doc["hum"].is<int>()) { out = doc["hum"].as<float>(); return true; }
      if (doc["rh"].is<float>() || doc["rh"].is<int>()) { out = doc["rh"].as<float>(); return true; }
      if (doc["rel_humidity"].is<float>() || doc["rel_humidity"].is<int>()) { out = doc["rel_humidity"].as<float>(); return true; }
      if (doc["hum"]["value"].is<float>() || doc["hum"]["value"].is<int>()) { out = doc["hum"]["value"].as<float>(); return true; }
      if (doc["value"].is<float>() || doc["value"].is<int>()) { out = doc["value"].as<float>(); return true; }
    }
  }
  out = parseNumericPayload(s);
  return true;
}

static bool parsePressurePayload(const char* s, float& out) {
  if (!s) return false;
  if (s[0] == '{') {
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      if (doc["pressure"].is<float>() || doc["pressure"].is<int>()) { out = doc["pressure"].as<float>(); return true; }
      if (doc["press"].is<float>() || doc["press"].is<int>()) { out = doc["press"].as<float>(); return true; }
      if (doc["baro"].is<float>() || doc["baro"].is<int>()) { out = doc["baro"].as<float>(); return true; }
      if (doc["value"].is<float>() || doc["value"].is<int>()) { out = doc["value"].as<float>(); return true; }
    }
  }
  out = parseNumericPayload(s);
  return true;
}

static bool parseLuxPayload(const char* s, int& out) {
  if (!s) return false;
  float tmp = 0.0f;
  if (s[0] == '{') {
    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, s) == DeserializationError::Ok) {
      if (doc["lux"].is<float>() || doc["lux"].is<int>()) { out = (int)roundf(doc["lux"].as<float>()); return true; }
      if (doc["illuminance"].is<float>() || doc["illuminance"].is<int>()) { out = (int)roundf(doc["illuminance"].as<float>()); return true; }
      if (doc["light"].is<float>() || doc["light"].is<int>()) { out = (int)roundf(doc["light"].as<float>()); return true; }
      if (doc["value"].is<float>() || doc["value"].is<int>()) { out = (int)roundf(doc["value"].as<float>()); return true; }
    }
  }
  tmp = parseNumericPayload(s);
  out = (int)roundf(tmp);
  return true;
}

// MQTT topic filter matcher with + and # wildcard support.
static bool mqttTopicMatches(const char* filter, const char* topic) {
  if (!filter || !topic) return false;

  const char* f = filter;
  const char* t = topic;

  while (*f && *t) {
    if (*f == '#') {
      // '#' must be terminal and matches remaining topic levels.
      return (*(f + 1) == '\0');
    }

    if (*f == '+') {
      // '+' matches exactly one level.
      while (*t && *t != '/') t++;
      f++;
      if (*f == '/') {
        if (*t != '/') return false;
        f++;
        t++;
      } else if (*f != '\0') {
        return false;
      }
      continue;
    }

    if (*f != *t) return false;
    f++;
    t++;
  }

  if (*f == '#') return (*(f + 1) == '\0');
  return (*f == '\0' && *t == '\0');
}

static void syncMasterTopicsFromConfig() {
  // Priority: runtime UI settings > compile-time fallback macros.
  if (strlen(settings.mqttInTempTopic) > 0) {
    strncpy(masterTemperatureTopic, settings.mqttInTempTopic, sizeof(masterTemperatureTopic) - 1);
    masterTemperatureTopic[sizeof(masterTemperatureTopic) - 1] = '\0';
  }
  if (strlen(settings.mqttInHumidTopic) > 0) {
    strncpy(masterHumidityTopic, settings.mqttInHumidTopic, sizeof(masterHumidityTopic) - 1);
    masterHumidityTopic[sizeof(masterHumidityTopic) - 1] = '\0';
  }
  if (strlen(settings.mqttInRadarTopic) > 0) {
    strncpy(masterRadarTopic, settings.mqttInRadarTopic, sizeof(masterRadarTopic) - 1);
    masterRadarTopic[sizeof(masterRadarTopic) - 1] = '\0';
  }
  if (strlen(settings.mqttInLuxTopic) > 0) {
    strncpy(masterLuxTopic, settings.mqttInLuxTopic, sizeof(masterLuxTopic) - 1);
    masterLuxTopic[sizeof(masterLuxTopic) - 1] = '\0';
  }
  if (strlen(settings.mqttInPressureTopic) > 0) {
    strncpy(masterPressureTopic, settings.mqttInPressureTopic, sizeof(masterPressureTopic) - 1);
    masterPressureTopic[sizeof(masterPressureTopic) - 1] = '\0';
  }

  mqttMasterTempEnabled = (strlen(masterTemperatureTopic) > 0);
  mqttMasterHumidEnabled = (strlen(masterHumidityTopic) > 0);
  mqttMasterRadarEnabled = (strlen(masterRadarTopic) > 0);
  mqttMasterLuxEnabled = (strlen(masterLuxTopic) > 0);
  mqttMasterPressureEnabled = (strlen(masterPressureTopic) > 0);
}

static void mqttEnsureSubscriptions() {
  if (!mqtt.isConnected()) return;
  syncMasterTopicsFromConfig();
  if (mqttMasterTempEnabled) mqtt.subscribe(masterTemperatureTopic);
  if (mqttMasterHumidEnabled) mqtt.subscribe(masterHumidityTopic);
  bool localRadarEnabled = false;
  #ifdef RADAR_PIN
    localRadarEnabled = (RADAR_PIN >= 0);
  #endif
  if (mqttMasterRadarEnabled && !localRadarEnabled) mqtt.subscribe(masterRadarTopic);
  if (mqttMasterLuxEnabled) mqtt.subscribe(masterLuxTopic);
  if (mqttMasterPressureEnabled) mqtt.subscribe(masterPressureTopic);
  for (int i = 0; i < 4; ++i) {
    if (strlen(externalDevices[i].topic) > 0) {
      mqtt.subscribe(externalDevices[i].topic);
    }
  }
}

void onMqttMessage(const char* topic, const uint8_t* payload, uint16_t length) {
  float tempTMP,humidTMP;
  float pressureTMP;
  int luxTMP;
  char tmp[100];
    // This callback is called when message from MQTT broker is received.
    // Please note that you should always verify if the message's topic is the one you expect.
    // For example: if (memcmp(topic, "myCustomTopic") == 0) { ... }
    
    DPRINT("New message on topic: ");    DPRINT(topic);
    // Generic external device support
    for (int i = 0; i < 4; ++i) {
      if (strlen(externalDevices[i].topic) > 0 && mqttTopicMatches(externalDevices[i].topic, topic)) {
        float val = parseNumericPayload((const char*)payload);
        externalDevices[i].lastValue = val;
        updateExternalDeviceSensor(externalDevices[i].sensorSlot, val);
        DPRINT("External device updated: "); DPRINT(externalDevices[i].name); DPRINT(" value: "); DPRINTLN(val);
        return;
      }
    }

    const uint16_t copyLen = (length < (sizeof(tmp) - 1)) ? length : (sizeof(tmp) - 1);
    memcpy(tmp, payload, copyLen);
    tmp[copyLen] = '\0';
    DPRINT("  Data: ");    DPRINTLN(tmp);
    if (mqttMasterTempEnabled && mqttTopicMatches(masterTemperatureTopic, topic)) {
      parseTemperaturePayload(tmp, tempTMP);
      temperature[MASTERtempPtr] = tempTMP;
      mqttLastInboundMs = millis();
      DPRINT("MASTER clock's temperature:"); DPRINTLN(tempTMP);
    }
    if (mqttMasterHumidEnabled && mqttTopicMatches(masterHumidityTopic, topic)) {
      parseHumidityPayload(tmp, humidTMP);
      DPRINT("MASTER clock's humidity:"); DPRINTLN(humidTMP);
      humid[MASTERhumidPtr] = humidTMP;
      mqttLastInboundMs = millis();
    }
    if (mqttMasterRadarEnabled && mqttTopicMatches(masterRadarTopic, topic)) {
       DPRINT("MASTER radar:"); DPRINTLN(tmp[0]);
       if (tmp[0] == '0')  {
         mqttRadarON = false;
         radarON = mqttRadarON;
       }
       else {
         mqttRadarON = true;
         radarON = mqttRadarON;
         radarLastOn = millis();
       }
       mqttLastInboundMs = millis();
       DPRINT("mqttRadarON:"); DPRINTLN(mqttRadarON);
 
    }
    if (mqttMasterPressureEnabled && mqttTopicMatches(masterPressureTopic, topic)) {
      parsePressurePayload(tmp, pressureTMP);
      pressur[0] = pressureTMP;
      mqttLastInboundMs = millis();
      DPRINT("MASTER pressure:"); DPRINTLN(pressureTMP);
    }
    if (mqttMasterLuxEnabled && mqttTopicMatches(masterLuxTopic, topic)) {
      parseLuxPayload(tmp, luxTMP);
      lx = luxTMP;
      mqttLastInboundMs = millis();
      DPRINT("MASTER lux:"); DPRINTLN(luxTMP);
    }
    //mqtt.publish("myPublishTopic", "hello");
}

void onMqttConnected() {
    DPRINTLN("Connected to the broker!");
  mqttConnected = true;
    mqttEnsureSubscriptions();
}

void mqttReconfigure() {
  // Apply runtime MQTT config changes immediately without full device reboot.
  mqtt.disconnect(false);
  mqttConnected = false;
  setupMqtt();
}

void setupMqtt() {
  char mactmp[20];

    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));  //set device MAC, as unique ID
    sprintf(mactmp,"%02x%02x%02x%02x%02x%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    masterTemperatureTopic[0] = '\0';
    masterHumidityTopic[0] = '\0';
    masterRadarTopic[0] = '\0';
    masterLuxTopic[0] = '\0';
    masterPressureTopic[0] = '\0';
    #if defined(MASTER_TEMPERATURE_TOPIC)
      strncpy(masterTemperatureTopic, MASTER_TEMPERATURE_TOPIC, sizeof(masterTemperatureTopic) - 1);
      masterTemperatureTopic[sizeof(masterTemperatureTopic) - 1] = '\0';
    #endif
    #if defined(MASTER_HUMIDITY_TOPIC)
      strncpy(masterHumidityTopic, MASTER_HUMIDITY_TOPIC, sizeof(masterHumidityTopic) - 1);
      masterHumidityTopic[sizeof(masterHumidityTopic) - 1] = '\0';
    #endif  
    #if defined(MASTER_RADAR_TOPIC)
      strncpy(masterRadarTopic, MASTER_RADAR_TOPIC, sizeof(masterRadarTopic) - 1);
      masterRadarTopic[sizeof(masterRadarTopic) - 1] = '\0';
    #endif
    syncMasterTopicsFromConfig();
    DPRINT("Starting MQTT CLient: "); DPRINT(MQTT_PREFIX); DPRINT("/sensor/"); DPRINTLN(mactmp);
    
    mqtt.setDiscoveryPrefix(MQTT_PREFIX);
    device.setName(FW);
    device.setSoftwareVersion("3.0");
    if (useTemp>0) {
      mqttTemp.setUnitOfMeasurement("°C");
      mqttTemp.setDeviceClass("sensor");
      mqttTemp.setName("Inside temperature");
    }
    if (useHumid>0) {
      mqttHumid.setUnitOfMeasurement("%");
      mqttHumid.setDeviceClass("sensor");
      mqttHumid.setName("Inside humidity");
    }
    if (usePress>0) {
      mqttPress.setUnitOfMeasurement("hPa");
      mqttPress.setDeviceClass("sensor");
      mqttPress.setName("Air pressure");
    }
    if (useLux>0) {
      mqttLux.setUnitOfMeasurement("lux");
      mqttLux.setDeviceClass("sensor");
      mqttLux.setName("Inside lux");
    }
    #if RADAR_PIN>=0
      mqttRadar.setUnitOfMeasurement("on/off");
      mqttRadar.setDeviceClass("sensor");
      mqttRadar.setName("Radar sensor");
    #endif

    mqtt.onMessage(onMqttMessage);
    mqtt.onConnected(onMqttConnected);
    
    // Keep MQTT params in persistent buffers (ArduinoHA stores pointers for reconnect).
    static char mqttHost[sizeof(prm.mqttBrokerAddr)] = {0};
    static char mqttUser[sizeof(prm.mqttBrokerUser)] = {0};
    static char mqttPass[sizeof(prm.mqttBrokerPsw)] = {0};
    uint16_t brokerPort = 1883;

    String broker = String(prm.mqttBrokerAddr);
    broker.trim();
    String user = String(prm.mqttBrokerUser);
    user.trim();
    String pass = String(prm.mqttBrokerPsw);

    int colon = broker.lastIndexOf(':');
    if (colon > 0 && colon < (broker.length() - 1)) {
      String portPart = broker.substring(colon + 1);
      int parsedPort = portPart.toInt();
      if (parsedPort > 0 && parsedPort <= 65535) {
        brokerPort = (uint16_t)parsedPort;
        broker = broker.substring(0, colon);
      }
    }

    broker.toCharArray(mqttHost, sizeof(mqttHost));
    user.toCharArray(mqttUser, sizeof(mqttUser));
    pass.toCharArray(mqttPass, sizeof(mqttPass));

    DPRINT("MQTT broker: "); DPRINT(mqttHost); DPRINT(":"); DPRINTLN(brokerPort);
    DPRINT("MQTT user: "); DPRINT(mqttUser); DPRINT(", pass len: "); DPRINTLN(strlen(mqttPass));

    IPAddress tmp;
    bool hasIp = tmp.fromString(mqttHost);
    const char* authUser = (strlen(mqttUser) > 0) ? mqttUser : nullptr;
    const char* authPass = (strlen(mqttUser) > 0) ? mqttPass : nullptr;

    if (hasIp) {
      mqtt.begin(tmp, brokerPort, authUser, authPass);
    } else {
      mqtt.begin(mqttHost, brokerPort, authUser, authPass);
    }
      
    //mqtt.begin(BROKER_ADDR, BROKER_USERNAME, BROKER_PASSWORD);

    device.enableLastWill();
    mqttConnected = false;

    if (mqttMasterTempEnabled) {
      DPRINT("MASTER_TEMPERATURE_TOPIC:"); DPRINTLN(masterTemperatureTopic);
      MASTERtempPtr = (useTemp < 6) ? useTemp : 5;
      if (useTemp < 6) useTemp++;
    }
    if (mqttMasterHumidEnabled) {
      DPRINT("MASTER_HUMIDITY_TOPIC:"); DPRINTLN(masterHumidityTopic);
      MASTERhumidPtr = (useHumid < 6) ? useHumid : 5;
      if (useHumid < 6) useHumid++;
    }
    if (mqttMasterRadarEnabled) {
      DPRINT("MASTER_RADAR_TOPIC:"); DPRINTLN(masterRadarTopic);
    }
    if (mqttMasterLuxEnabled) {
      DPRINT("MASTER_LUX_TOPIC:"); DPRINTLN(masterLuxTopic);
      if (useLux == 0) useLux = 1;
    }
    if (mqttMasterPressureEnabled) {
      DPRINT("MASTER_PRESSURE_TOPIC:"); DPRINTLN(masterPressureTopic);
      if (usePress == 0) usePress = 1;
    }
}

void mqttSend() {
  static unsigned long lastRun = millis();
  //if (!mqtt.isConnected()) return;
  mqtt.loop();
  mqttConnected = mqtt.isConnected();
  mqttEnsureSubscriptions();
    
  if ((millis() - lastRun) < prm.mqttBrokerRefresh*1000) return;
  lastRun = millis();  
  if (mqtt.isConnected()) DPRINT("MQTT Connected."); 
  else {
    DPRINTLN("MQTT Failed to connect.");
    return;
  }
  DPRINT("MQTT send:");
  if (useTemp>0)  {mqttTemp.setValue(temperature[0]+ prm.corrT0);  DPRINT(temperature[0]+ prm.corrT0); DPRINT(", ");}
  if (useHumid>0) {mqttHumid.setValue(humid[0]+ prm.corrH0);  DPRINT(humid[0]+ prm.corrH0); DPRINT(", ");}
  if (usePress>0) {mqttPress.setValue(pressur[0]);  DPRINT(pressur[0]);  DPRINT(", ");}
  if (useLux>0)   {mqttLux.setValue(lx);   DPRINT(lx);  DPRINT(", ");}
  #if RADAR_PIN>=0
    mqttRadar.setValue(radarON);   DPRINT(radarON);
  #endif
  DPRINTLN(" ");
}
  
#else
  volatile unsigned long mqttLastInboundMs = 0;
  bool mqttConnected = false;
  void mqttSend() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mulev2.ino"
#ifdef MULE_V2
char tubeDriver[] = "MULE_V2";

//define here the digit enable pins from 4 to 8

//byte digitEnablePins[] = {15,13,12,14};   //fox example... But SET in clocks.h !!!
//byte ABCDPins[4] =  {2,4,5,0};
//int DP_PIN = -1; // decimalPoint inside Nixie tube, set -1, if not used!

int maxDigits = 6;  //sizeof(digitEnablePins);
int DRAM_ATTR maxDig = maxDigits;   //memory variable version

//byte convert[] = {1,0,9,8,7,6,5,4,3,2};   //tube pin conversion, is needed (for example: bad tube pin layout)
int DRAM_ATTR PWMrefresh=11000;   //msec, Multiplex time period. Greater value => slower multiplex frequency
int DRAM_ATTR PWM_min = 1000;
int DRAM_ATTR PWM_max = 10000;
//int DRAM_ATTR PWMtiming[11] = {0,1000,2000,3000,4000,5000,6000,7000,8000,9000,10000};

  #if defined(ESP32) 
  #else
    #error "Only ESP32 board is supported by this driver!"  
  #endif

void setup_pins() {
  char tmp[30];
    
  DPRINTLN("Nixie clock - setup pins -  Multiplex BCD mode (MULE_V2)...");
  DPRINTLN("BCD digitEnablePins: ");
  for (int i=0;i<maxDigits;i++) {
    pinMode(digitEnablePins[i], OUTPUT);
    sprintf(tmp,"digitEnPin[%d]",i);
    regPin(digitEnablePins[i],tmp); 
  }
  DPRINTLN(" ");
  DPRINTLN("ABCDPins: ");
  for (int i=0;i<4;i++) {
    pinMode(ABCDPins[i], OUTPUT);
    sprintf(tmp,"Pin[%c]",char('A'+i));
    regPin(ABCDPins[i],tmp); 
  }
  DPRINTLN(" ");
  #if DP_PIN>=0
    pinMode(DP_PIN,OUTPUT);  
    DPRINT("DP_PIN:"); DPRINTLN(DP_PIN);
  #endif  
  startTimer();
}

void IRAM_ATTR writeDisplay(){  //void IRAM_ATTR  writeDisplay(){

  static DRAM_ATTR byte pos = 0;
  static DRAM_ATTR byte state=0;
  static DRAM_ATTR int timer = PWMrefresh;
  static DRAM_ATTR int num,brightness;
  static DRAM_ATTR byte DPpos;
  static DRAM_ATTR int PWMtimeBrightness=PWM_min;
  
  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    return;  
  }
  
  portENTER_CRITICAL_ISR(&timerMux);
  noInterrupts();
  
  intCounter++;
  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  timer = PWMrefresh;

   switch (state) {   //state machine...
    case 0:
      pos++;  if (pos>maxDig-1)  {
        pos = 0;  //go to the first tube
        if (autoBrightness && displayON) {
          PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
        }
        else
          PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
      }
      if (animMask[pos] > 0) { //Animation?
        num = oldDigit[pos];  //show old character
        timer = (PWMtimeBrightness * (20-animMask[pos]))/20;
        state = 1;  //next state is: show newDigit
      }
      else {
        num = digit[pos];  //show active character
        timer = PWMtimeBrightness;  
        state = 2;  //next state is: BLANK display
      }
      break;
    case 1:  //show new character, if animation
      num =   newDigit[pos];
      timer = (PWMtimeBrightness * animMask[pos])/20;      
      state = 2;  //default next state is: BLANK display
      break;
    case 2:  //blank display
      timer = PWMrefresh-PWMtimeBrightness;
      state = 3;
      break;
   }  //end switch
   if (timer<500) timer = 500;  //safety only...
 
   //  if ((pos>0) && (num<=9)) num = convert[num];   //tube character conversion, if needed... (maybe bad pin numbering)

  if ((brightness == 0) || (state == 3) || (num >9) || (!radarON)) {  //blank digit
    state = 0; 
    for (int i=0;i<sizeof(digitEnablePins);i++)
      digitalWrite(digitEnablePins[i],HIGH);  //switch off anode 
    #if DP_PIN >=0
      digitalWrite(DP_PIN,LOW);
    #endif  
    }
  else {
      for (int i=0;i<4;i++) {digitalWrite(ABCDPins[i],num  & 1<<i); }
      for (int i=0;i<sizeof(digitEnablePins);i++) {digitalWrite(digitEnablePins[i],(pos+1)  & 1<<i);} //switch on the new digit}
      
      #if DP_PIN >=0
        if (LEFTDECIMAL) DPpos = min(maxDig-1,pos+1); else DPpos = pos;
        if (digitDP[DPpos]) digitalWrite(DP_PIN,HIGH); //switch ON decimal point, if needed
      #endif
  }
    
  #if COLON_PIN >= 0
    if (num==10) digitalWrite(COLON_PIN,LOW);      // Colon pin OFF
    else digitalWrite(COLON_PIN,colonBlinkState);  // Blink colon pin
  #endif
  
  #if DECIMALPOINT_PIN >=0 
        if (num==10) {digitalWrite(DECIMALPOINT_PIN,LOW);}
        else {digitalWrite(DECIMALPOINT_PIN,decimalpointON); }
  #endif

  portEXIT_CRITICAL_ISR(&timerMux);
  //ESP32timer = timerBegin(0, PRESCALER, true);  //set prescaler, true = edge generated signal
  //timerAttachInterrupt(ESP32timer, &writeDisplay, true);   
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
}

void clearTubes() {
  
  for (int i=0;i<sizeof(digitEnablePins);i++) digitalWrite(digitEnablePins[i],HIGH); 
    #if (DP_PIN>=0) 
      digitalWrite(DP_PIN,LOW);
    #endif  
}

void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\multiplex74141.ino"
#ifdef MULTIPLEX74141   //Nixie driver for 8266 only!
char tubeDriver[] = "MULTIPLEX74141";
//define here the digit enable pins from 4 to 8
//byte digitEnablePins[] = {15,13,12,14};   //fox example... But SET in clocks.h !!!
//byte ABCDPins[4] =  {2,4,5,0};
//#define DP_PIN -1             // decimalPoint inside Nixie tube, set -1, if not used!

#if defined(ESP8266)
#else
  #error "Board is not supported! For ESP32 use MULTIPLEX74141_ESP32 !"  
#endif

int maxDigits = sizeof(digitEnablePins);
int maxDig = maxDigits;   //memory variable version

//byte convert[] = {1,0,9,8,7,6,5,4,3,2};   //tube pin conversion, is needed (for example: bad tube pin layout)
int PWMrefresh=11000;   //msec, Multiplex time period. Greater value => slower multiplex frequency
int PWM_min = 1000;
int PWM_max = 10000;
//int PWMtiming[11] = {0,1000,2000,3000,4000,5000,6000,7000,8000,9000,10000};

void setup_pins() {
  char tmp[30];
    
  DPRINTLN("Nixie clock - setup pins -  Multiplex 74141 mode...");
  DPRINTLN("digitEnablePins: ");
  for (int i=0;i<maxDigits;i++) {
    pinMode(digitEnablePins[i], OUTPUT); 
    sprintf(tmp,"digitEnPin[%d]",i);
    regPin(digitEnablePins[i],tmp); 
  }
  DPRINTLN(" ");

  DPRINTLN("ABCDPins: ");
  for (int i=0;i<4;i++) {
    pinMode(ABCDPins[i], OUTPUT);
    sprintf(tmp,"Pin[%c]",char('A'+i));
    regPin(ABCDPins[i],tmp); 
  }
  DPRINTLN(" ");
  #if DP_PIN>=0
    pinMode(DP_PIN,OUTPUT);   regPin(DP_PIN,"DP_PIN");
    DPRINT("DP_PIN:"); DPRINTLN(DP_PIN);
  #endif  
  startTimer();
}

void ICACHE_RAM_ATTR writeDisplay(){        //https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
  static byte pos = 0;
  static volatile byte state=0;
  static int timer = PWMrefresh;
  static int num,brightness;
  static byte DPpos;
  static int PWMtimeBrightness=PWM_min;

  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
      digitalWrite(digitEnablePins[pos],LOW); 
      timer1_write(PWMrefresh);
    return;  
  }
  
  intCounter++;
  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  timer = PWMrefresh;

   switch (state) {   //state machine...
    case 0:
      pos++;  if (pos>maxDig-1)  {
        pos = 0;  //go to the first tube
        if (autoBrightness && displayON) {
          PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
        }
        else
          PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
        }
      
      if (animMask[pos] > 0) { //Animation?
        num = oldDigit[pos];  //show old character
        timer = (PWMtimeBrightness * (20-animMask[pos]))/20;
        state = 1;  //next state is: show newDigit
      }
      else {
        num = digit[pos];  //show active character
        timer = PWMtimeBrightness;  
        state = 2;  //next state is: BLANK display
      }
      break;
    case 1:  //show new character, if animation
      num =   newDigit[pos];
      timer = (PWMtimeBrightness * animMask[pos])/20;      
      state = 2;  //default next state is: BLANK display
      break;
    case 2:  //blank display
      timer = PWMrefresh-PWMtimeBrightness;
      state = 3;
      break;
   }  //end switch
   if (timer<500) timer = 500;  //safety only...

    //  if ((pos>0) && (num<=9)) num = convert[num];   //tube character conversion, if needed... (maybe bad pin numbering)

  if ((brightness == 0) || (state == 3) || (num >9) || (!radarON)) {  //blank digit
    state = 0; 
    digitalWrite(digitEnablePins[pos],LOW);  //switch off anode 
    #if DP_PIN >=0
      digitalWrite(DP_PIN,LOW);
    #endif 
    }
  else {
      for (int i=0;i<4;i++) {digitalWrite(ABCDPins[i],num  & 1<<i); }
      digitalWrite(digitEnablePins[pos],HIGH);    //switch on the new digit
      #if DP_PIN >=0
        if (LEFTDECIMAL) DPpos = min(maxDig-1,pos+1); else DPpos = pos;
        if (digitDP[DPpos]) digitalWrite(DP_PIN,HIGH); //switch ON decimal point, if needed
      #endif
  }
    
  #if COLON_PIN >= 0
    if (num==10) digitalWrite(COLON_PIN,LOW);      // Colon pin OFF
    else digitalWrite(COLON_PIN,colonBlinkState);  // Blink colon pin
  #endif
  
  #if DECIMALPOINT_PIN >=0 
        if (num==10) {digitalWrite(DECIMALPOINT_PIN,LOW);}
        else {digitalWrite(DECIMALPOINT_PIN,decimalpointON); }
  #endif

  timer1_write(timer);
}

void clearTubes() {
      for (int i=0;i<maxDig;i++) digitalWrite(digitEnablePins[i],LOW); 
      #if DP_PIN >= 0 
        digitalWrite(DP_PIN,LOW);
      #endif  
}

void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\multiplex74141_ESP32.ino"
#ifdef MULTIPLEX74141_ESP32
char tubeDriver[] = "MULTIPLEX74141_ESP32";

//define here the digit enable pins from 4 to 8

//byte digitEnablePins[] = {15,13,12,14};   //fox example... But SET in clocks.h !!!
//byte ABCDPins[4] =  {2,4,5,0};
//int DP_PIN = -1; // decimalPoint inside Nixie tube, set -1, if not used!

int maxDigits = sizeof(digitEnablePins);
int DRAM_ATTR maxDig = maxDigits;   //memory variable version

//byte convert[] = {1,0,9,8,7,6,5,4,3,2};   //tube pin conversion, is needed (for example: bad tube pin layout)
int DRAM_ATTR PWMrefresh=11000;   //msec, Multiplex time period. Greater value => slower multiplex frequency
int DRAM_ATTR PWM_min = 1000;
int DRAM_ATTR PWM_max = 10000;
//int DRAM_ATTR PWMtiming[11] = {0,1000,2000,3000,4000,5000,6000,7000,8000,9000,10000};

  #if defined(ESP32) 
  #else
    #error "Only ESP32 board is supported by this driver!"  
  #endif

void setup_pins() {
  char tmp[30];
    
  DPRINTLN("Nixie clock - setup pins -  Multiplex 74141 mode...");
  DPRINTLN("digitEnablePins: ");
  for (int i=0;i<maxDigits;i++) {
    pinMode(digitEnablePins[i], OUTPUT);
    sprintf(tmp,"digitEnPin[%d]",i);
    regPin(digitEnablePins[i],tmp); 
  }
  DPRINTLN(" ");
  DPRINTLN("ABCDPins: ");
  for (int i=0;i<4;i++) {
    pinMode(ABCDPins[i], OUTPUT);
    sprintf(tmp,"Pin[%c]",char('A'+i));
    regPin(ABCDPins[i],tmp); 
  }
  DPRINTLN(" ");
  #if DP_PIN>=0
    pinMode(DP_PIN,OUTPUT);  
    DPRINT("DP_PIN:"); DPRINTLN(DP_PIN);
  #endif  
  startTimer();
}

void IRAM_ATTR writeDisplay(){  //void IRAM_ATTR  writeDisplay(){

  static DRAM_ATTR byte pos = 0;
  static DRAM_ATTR byte state=0;
  static DRAM_ATTR int timer = PWMrefresh;
  static DRAM_ATTR int num,brightness;
  static DRAM_ATTR byte DPpos;
  static DRAM_ATTR int PWMtimeBrightness=PWM_min;
  
  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    return;  
  }
  
  portENTER_CRITICAL_ISR(&timerMux);
  noInterrupts();
  
  intCounter++;
  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

  timer = PWMrefresh;

   switch (state) {   //state machine...
    case 0:
      pos++;  if (pos>maxDig-1)  {
        pos = 0;  //go to the first tube
        if (autoBrightness && displayON) {
          PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
        }
        else
          PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
      }
      if (animMask[pos] > 0) { //Animation?
        num = oldDigit[pos];  //show old character
        timer = (PWMtimeBrightness * (20-animMask[pos]))/20;
        state = 1;  //next state is: show newDigit
      }
      else {
        num = digit[pos];  //show active character
        timer = PWMtimeBrightness;  
        state = 2;  //next state is: BLANK display
      }
      break;
    case 1:  //show new character, if animation
      num =   newDigit[pos];
      timer = (PWMtimeBrightness * animMask[pos])/20;      
      state = 2;  //default next state is: BLANK display
      break;
    case 2:  //blank display
      timer = PWMrefresh-PWMtimeBrightness;
      state = 3;
      break;
   }  //end switch
   if (timer<500) timer = 500;  //safety only...
 
   //  if ((pos>0) && (num<=9)) num = convert[num];   //tube character conversion, if needed... (maybe bad pin numbering)

  if ((brightness == 0) || (state == 3) || (num >9) || (!radarON)) {  //blank digit
    state = 0; 
    digitalWrite(digitEnablePins[pos],LOW);  //switch off anode 
    #if DP_PIN >=0
      digitalWrite(DP_PIN,LOW);
    #endif  
    #if COLON_PIN >= 0
      digitalWrite(COLON_PIN,LOW);      // Colon pin OFF
    #endif    
    #if DECIMALPOINT_PIN >=0 
        digitalWrite(DECIMALPOINT_PIN,LOW);
    #endif
    }
  else {
      for (int i=0;i<4;i++) {digitalWrite(ABCDPins[i],num  & 1<<i); }
      digitalWrite(digitEnablePins[pos],HIGH);    //switch on the new digit
      #if DP_PIN >=0
        if (LEFTDECIMAL) DPpos = min(maxDig-1,pos+1); else DPpos = pos;
        if (digitDP[DPpos]) digitalWrite(DP_PIN,HIGH); //switch ON decimal point, if needed
		else digitalWrite(DP_PIN,LOW); 
      #endif
      #if COLON_PIN >= 0
        digitalWrite(COLON_PIN,colonBlinkState);  // Blink colon pin
      #endif
      #if DECIMALPOINT_PIN >=0 
        digitalWrite(DECIMALPOINT_PIN,decimalpointON); 
      #endif
  }
    
  
  portEXIT_CRITICAL_ISR(&timerMux);
  //ESP32timer = timerBegin(0, PRESCALER, true);  //set prescaler, true = edge generated signal
  //timerAttachInterrupt(ESP32timer, &writeDisplay, true);   
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
}

void clearTubes() {
  
  for (int i=0;i<maxDig;i++) digitalWrite(digitEnablePins[i],LOW); 
    
    #if (DP_PIN>=0) 
      digitalWrite(DP_PIN,LOW);
    #endif  
}

void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\no_multiplex74141.ino"
// 4digit Nixie Clock 
//  2x 74HC595N shift register + 4x 74141

#ifdef NO_MULTIPLEX74141

char tubeDriver[] = "NO_MULTIPLEX74141";
#if defined(ESP8266) 
#else
  #error "Only 8266 Board is supported!"  
#endif

 //change it, if needed for the correct tube sequence
//byte tubes[] = {3,2,1,0};         //4 tubes,   old clock...   in clocks.h  
//byte tubes[] = {5,4,3,2,1,0};   //6 tubes, reverse order      in clocks.h

int maxDigits = sizeof(tubes);
int PWMrefresh=12000;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
int PWM_min = 1000;
int PWM_max = 12000;
//int PWMtiming[11] = {1000,1000,2000,3000,4000,5000,6000,7000,8000,10000,12000};

//#define dataPin  14  //D5
//#define latchPin 12  //D6
//#define clkPin   13  //D7

void writeDisplaySingle() { }   

void setup_pins(){
  DPRINTLN("Nixie clock - setup pins NON-MULTIPLEX 4(or 6) x 74141");
  pinMode(dataPin, OUTPUT); regPin(dataPin,"dataPin"); 
  pinMode(latchPin,OUTPUT); regPin(latchPin,"latchPin"); 
  pinMode(clkPin,OUTPUT);   regPin(clkPin,"clkPin"); 
  startTimer();
}

void ICACHE_RAM_ATTR writeBits(byte num) {   //shift out 4 bits
  for (int i=3;i>=0;i--) {
    digitalWrite(dataPin,num  & (1<<i)); 
    for (int t=0; t<10;t++) asm volatile ("nop");   //100nsec
    digitalWrite(clkPin,HIGH);
    for (int t=0; t<10;t++) asm volatile ("nop");   //100nsec
    digitalWrite(clkPin,LOW);
    for (int t=0; t<10;t++) asm volatile ("nop");   //100nsec
    }
}

void ICACHE_RAM_ATTR writeDisplay(){        //https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
  static volatile byte state=0;
  static int timer = PWMrefresh;
  static int brightness;
  static int PWMtimeBrightness=PWM_min;
  byte animM;
  
  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    timer1_write(PWMrefresh);
    return;  
  }

  animM = 0;
  for (int i=0;i<maxDigits;i++) {
    if (animMask[i] > 0) animM = animMask[i];    //find, if any animation is wanted?
  }

  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety
  
  digitalWrite(latchPin, LOW);

  if ((brightness ==0) || !radarON){
    for (int i=0;i<maxDigits;i++) writeBits(0xA);  //black display
    
  #if DECIMALPOINT_PIN>=0 
    digitalWrite(DECIMALPOINT_PIN,LOW);
  #endif  
  #if COLON_PIN >= 0 
    digitalWrite(COLON_PIN,LOW);  // colon pin OFF
  #endif
  digitalWrite(latchPin, HIGH);
  timer1_write(100*PWMrefresh);
  return;
  }
  else {
    for (int i=0;i<maxDigits;i++) {
      if (state==0) {
        if (animMask[i]==0) writeBits(digit[tubes[i]]);
        else writeBits(oldDigit[tubes[i]]);
      }  
      else if (state==1) writeBits(newDigit[tubes[i]]);
      else writeBits(0xF);
    } //end for
   
  //for (int i=0;i<maxDigits;i++) writeBits(digit[tubes[i]]);   //simple test, without animation
  
  switch (state) {   //state machine...
    case 0:
      #if DECIMALPOINT_PIN>=0
        if (decimalpointON) digitalWrite(DECIMALPOINT_PIN,HIGH);
      #endif  
      #if COLON_PIN>=0
        digitalWrite(COLON_PIN,colonBlinkState);  // Blink colon pin
      #endif  
      if (autoBrightness && displayON) {
          PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
      }
      else {
        PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
      }      
      if (animM > 0) { //Animation?
        timer =  (PWMtimeBrightness * (20-animM))/20;
        state = 1;  //next state is: show newDigits
      }
      else {
        timer = PWMtimeBrightness;  
        if (brightness>=MAXBRIGHTNESS) state = 0;
        else state = 2;  //default next state is: BLANK display
      }
      break;
    case 1:  //show new character, if animation
      timer = (PWMtimeBrightness * animM)/20;      
      if (brightness>=MAXBRIGHTNESS) state = 0;
      else state = 2;  //default next state is: BLANK display
      break;
    case 2:  //blank display
      #if DECIMALPOINT_PIN>=0 
        digitalWrite(DECIMALPOINT_PIN,LOW);
      #endif  
      #if COLON_PIN>=0 
        digitalWrite(COLON_PIN,LOW);  // colon pin OFF
      #endif  
      state = 0;
      timer = PWMrefresh-PWMtimeBrightness;
      break;
  }  //end switch
  } //end else
    
  digitalWrite(latchPin, HIGH);    
  
  if ((brightness == 0) || (!radarON)) {timer = PWM_max; state = 0;}  //no time sharing is needed
  if (timer<500) timer = 500;  //safety only...
  timer1_write(timer); 
}

void clearTubes() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\no_multiplex_ESP32.ino"
// 4digit Nixie Clock 
//  2x 74HC595N shift register + 4x 74141

#ifdef NO_MULTIPLEX_ESP32

char tubeDriver[] = "NO_MULTIPLEX_ESP32";
#if defined(ESP32) 
#else
  #error "Only ESP32 Board is supported!"  
#endif

 //change it, if needed for the correct tube sequence
//byte tubes[] = {3,2,1,0};         //4 tubes,   old clock...        use in clocks.h
//byte tubes[] = {5,4,3,2,1,0};   //6 tubes, reverse order            use in clocks.h

int maxDigits = sizeof(tubes);
int DRAM_ATTR PWMrefresh=25000;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
int DRAM_ATTR PWM_min = 5000;
int DRAM_ATTR PWM_max = 25000;

//#define dataPin  14  //D5
//#define latchPin 27 //12  //D6
//#define clkPin   26 //13  //D7

void IRAM_ATTR writeDisplaySingle() { }   

void setup_pins(){
  DPRINTLN("Nixie clock - setup pins NON-MULTIPLEX 4(or 6) x 74141");
  pinMode(dataPin, OUTPUT); regPin(dataPin,"dataPin"); 
  pinMode(latchPin,OUTPUT); regPin(latchPin,"latchPin"); 
  pinMode(clkPin,OUTPUT);   regPin(clkPin,"clkPin"); 
  startTimer();
}

void ICACHE_RAM_ATTR writeBits(byte num) {   //shift out 4 bits
  for (int i=3;i>=0;i--) {
    digitalWrite(dataPin,num  & (1<<i)); 
    for (int t=0; t<10;t++) asm volatile ("nop");   //100nsec
    digitalWrite(clkPin,HIGH);
    for (int t=0; t<10;t++) asm volatile ("nop");   //100nsec
    digitalWrite(clkPin,LOW);
    for (int t=0; t<10;t++) asm volatile ("nop");   //100nsec
    }
}

void IRAM_ATTR writeDisplay(){        //https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
  static DRAM_ATTR volatile byte state=0;
  static DRAM_ATTR int timer = PWMrefresh;
  static DRAM_ATTR int brightness;
  static DRAM_ATTR int PWMtimeBrightness=PWM_min;
  byte animM;
  
  if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    return;  
  }
  
  portENTER_CRITICAL_ISR(&timerMux);
  noInterrupts();

  animM = 0;
  for (int i=0;i<maxDigits;i++) {
    if (animMask[i] > 0) animM = animMask[i];    //find, if any animation is wanted?
  }

  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety
  
  digitalWrite(latchPin, LOW);

  if ((brightness ==0) || !radarON) {
    for (int i=0;i<maxDigits;i++) writeBits(0xA);  //black display
    
  #if DECIMALPOINT_PIN>=0 
    digitalWrite(DECIMALPOINT_PIN,LOW);
  #endif  
  #if COLON_PIN >= 0 
    digitalWrite(COLON_PIN,LOW);  // colon pin OFF
  #endif
  digitalWrite(latchPin, HIGH);
  portEXIT_CRITICAL_ISR(&timerMux);
  timerAlarmWrite(ESP32timer, 100*PWMrefresh, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
  return;
  }
  else {
    for (int i=0;i<maxDigits;i++) {
      if (state==0) {
        if (animMask[i]==0) writeBits(digit[tubes[i]]);
        else writeBits(oldDigit[tubes[i]]);
      }  
      else if (state==1) writeBits(newDigit[tubes[i]]);
      else writeBits(0xF);
    } //end for
   
  //for (int i=0;i<maxDigits;i++) writeBits(digit[tubes[i]]);   //simple test, without animation
  
  switch (state) {   //state machine...
    case 0:
      #if DECIMALPOINT_PIN>=0
        if (decimalpointON) digitalWrite(DECIMALPOINT_PIN,HIGH);
      #endif  
      #if COLON_PIN>=0
        digitalWrite(COLON_PIN,colonBlinkState);  // Blink colon pin
      #endif  
      if (autoBrightness && displayON) {
          PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
      }
      else {
        PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
      }      
      if (animM > 0) { //Animation?
        timer =  (PWMtimeBrightness * (20-animM))/20;
        state = 1;  //next state is: show newDigits
      }
      else {
        timer = PWMtimeBrightness;  
        if (brightness>=MAXBRIGHTNESS) state = 0;
        else state = 2;  //default next state is: BLANK display
      }
      break;
    case 1:  //show new character, if animation
      timer = (PWMtimeBrightness * animM)/20;      
      if (brightness>=MAXBRIGHTNESS) state = 0;
      else state = 2;  //default next state is: BLANK display
      break;
    case 2:  //blank display
      #if DECIMALPOINT_PIN>=0 
        digitalWrite(DECIMALPOINT_PIN,LOW);
      #endif  
      #if COLON_PIN>=0 
        digitalWrite(COLON_PIN,LOW);  // colon pin OFF
      #endif  
      state = 0;
      timer = PWMrefresh-PWMtimeBrightness;
      break;
  }  //end switch
  } //end else
    
  digitalWrite(latchPin, HIGH);    
  
  if ((brightness == 0) || (!radarON)) {timer = PWM_max; state = 0;}  //no time sharing is needed
  if (timer<500) timer = 500;  //safety only...
  portEXIT_CRITICAL_ISR(&timerMux);
  timerAlarmWrite(ESP32timer, timer, true);   
  timerAlarmEnable(ESP32timer);
  interrupts();
}

void IRAM_ATTR clearTubes() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\pcf_multiplex74141.ino"
#ifdef PCF_74141
char tubeDriver[] = "PCF_74141";
//PCF8574 I/O expander version, 4..8 tube Nixie Clock, driven by 74141
//define here the digit enable pins from 4 to 8 tubes

//if a digitEnablePort is on PCF chip, add 100 to the port number!   (for example  2-->102)

#define I2C_ADDR 0x20
#define PCF_SDA_PIN 4
#define PCF_SCL_PIN 5

//If a digitEnablePin is on pcf8574, add 100 to the pin number.   (P0 = 100,... P7 = 107)
byte digitEnablePins[] = {100,101,102,103,104,105};    //6 tube nixie driven by PCF 
byte ABCDPins[4] = {14,12,13,2};  //D5,D6,D7,D4 on 8266
#define DP_PIN  16 // decimalPoint on 8266's D0

int maxDigits = sizeof(digitEnablePins);

//byte convert[] = {1,0,9,8,7,6,5,4,3,2};   //tube pin conversion, is needed (for example: bad tube pin layout)
int PWMrefresh=15000;   ////msec, Multiplex time period. Greater value => slower multiplex frequency
int PWM_min = 2000;
int PWM_max = 14000;
//int PWMtiming[11] = {0,2000,3000,4000,5000,6000,7000,8000,10000,12000,14000};

#if defined(ESP8266) 
#else
  #error "Only 8266 Board is supported!"  
#endif

void ICACHE_RAM_ATTR delayMS(int d) {        //Delay microsec
  for (int i=0;i<d*30;i++) {asm volatile ("nop"); }
}

void ICACHE_RAM_ATTR shiftout(byte in) {
  for (int i=0;i<8;i++) {
    digitalWrite(PCF_SDA_PIN,in  & (1<<(7-i)));
    delayMS(1);
    digitalWrite(PCF_SCL_PIN,HIGH);
    //if (i==0) delayMS(8);
    delayMS(4);
    digitalWrite(PCF_SCL_PIN,LOW);
  }  

  //ACK is coming
  //pinMode(PCF_SDA_PIN,INPUT_PULLUP); 
  digitalWrite(PCF_SDA_PIN,LOW);  //LOW
  delayMS(12);
  digitalWrite(PCF_SCL_PIN,HIGH);  //clock pulse
  delayMS(4); 
  digitalWrite(PCF_SCL_PIN,LOW);
  //pinMode(PCF_SDA_PIN,OUTPUT);
    delayMS(4); 
 }

void ICACHE_RAM_ATTR sendBits(byte address,byte val){ 
  //Serial.print(address,HEX); Serial.print("/"); Serial.println(val);
  
  //void ICACHE_RAM_ATTR startCondition() {
  //digitalWrite(PCF_SDA_PIN,HIGH);   //PCF_SDA_PIN HIGH -> LOW, while PCF_SCL_PIN is HIGH
  //digitalWrite(PCF_SCL_PIN,HIGH);
  //delayMS(4); 
  digitalWrite(PCF_SDA_PIN,LOW);
  delayMS(3); 
  digitalWrite(PCF_SCL_PIN,LOW);
  delayMS(4); 
  
  shiftout(address<<1);   //shift address one bit left, because 0bit is READ/WRITE mode. 0=WRITE
  shiftout(val);
  
  //void ICACHE_RAM_ATTR stopCondition() {
  delayMS(4);
  digitalWrite(PCF_SDA_PIN,LOW);   //PCF_SDA_PIN LOW -> HIGH, while PCF_SCL_PIN is HIGH
  delayMS(2); 
  digitalWrite(PCF_SCL_PIN,HIGH);
  delayMS(4);
  digitalWrite(PCF_SDA_PIN,HIGH);
  delayMS(4); 
}


void setup_pins() {
  char tmp[30];
    
  DPRINTLN("PCF8574 MULTIPLEX driver");
  for (int i=0;i<maxDigits;i++)  {
    if (digitEnablePins[i]<100) {
      pinMode(digitEnablePins[i], OUTPUT);   //8266 pins setup, if needed
      sprintf(tmp,"digitEnPin[%d]",i);
      regPin(digitEnablePins[i],tmp); 
      }
  }
  for (int i=0;i<4;i++)  {
    pinMode(ABCDPins[i], OUTPUT);
    sprintf(tmp,"Pin[%c]",char('A'+i));
    regPin(ABCDPins[i],tmp); 
  }  
  pinMode(PCF_SDA_PIN,OUTPUT); regPin(PCF_SDA_PIN,"PCF_SDA_PIN");
  pinMode(PCF_SCL_PIN,OUTPUT); regPin(PCF_SCL_PIN,"PCF_SCL_PIN");
  pinMode(DP_PIN,OUTPUT);  regPin(DP_PIN,"DP_PIN");
  digitalWrite(PCF_SDA_PIN,HIGH);
  digitalWrite(PCF_SCL_PIN,HIGH);
  delay(100); 
  sendBits(I2C_ADDR,0);
  
  startTimer();
  ESP.wdtDisable();
  ESP.wdtEnable(WDTO_8S);
  }

void ICACHE_RAM_ATTR writeDisplay(){        //https://circuits4you.com/2018/01/02/esp8266-timer-ticker-example/
  static byte pos = 0;
  static byte state=0;
  static int timer = PWMrefresh;
  static byte num,brightness;
  static byte p,DPpos;
  static int PWMtimeBrightness = PWM_min;  
  
if (EEPROMsaving) {  //stop refresh, while EEPROM write is in progress!
    //digitalWrite(digitEnablePins[pos],LOW); 
    timer1_write(PWMrefresh);
    return;  
  }

  if (autoBrightness && displayON) {
    PWMtimeBrightness = max(PWM_min,PWM_max*lx/MAXIMUM_LUX);
  }
  else
    PWMtimeBrightness = max(PWM_min,PWM_max*brightness/MAXBRIGHTNESS);
  
  intCounter++;
  timer = PWMrefresh;
  brightness = displayON ?  prm.dayBright : prm.nightBright;
  if (brightness>MAXBRIGHTNESS) brightness = MAXBRIGHTNESS;  //only for safety

   switch (state) {   //state machine...
    case 0:
      pos++;  if (pos>maxDigits-1)  pos = 0;  //go to the first tube

      if (animMask[pos] > 0) { //Animation?
        num = oldDigit[pos];  //show old character
        timer = (PWMtimeBrightness * (20-animMask[pos]))/20;
        state = 1;  //next state is: show newDigit
      }
      else {
        num = digit[pos];  //show active character
        timer = PWMtimeBrightness;  
        state = 2;  //next state is: BLANK display
      }
      break;
    case 1:  //show new character, if animation
      num =   newDigit[pos];
      timer = (PWMtimeBrightness * animMask[pos])/20;      
      state = 2;  //default next state is: BLANK display
      break;
    case 2:  //blank display
      timer = PWMrefresh-PWMtimeBrightness;
      state = 3;
      break;
   }  //end switch
 
   //  if ((pos>0) && (num<=9)) num = convert[num];   //tube character conversion, if needed... (maybe bad pin numbering)
   
  p  = digitEnablePins[pos];

  if ((brightness == 0) || (state == 3) || (num >9) || (!radarON)) {  //blank digit
    state = 0; 
    for (int i=0;i<4;i++) {digitalWrite(ABCDPins[i],HIGH); }
      digitalWrite(DP_PIN,LOW);
    }
  else {
      if (p<100) digitalWrite(p,HIGH);      //switch ON new digit on 8266
      else sendBits(I2C_ADDR,1<<(p-100)) ;  // or on PCF port
      if (LEFTDECIMAL) DPpos = min(maxDigits-1,pos+1); else DPpos = pos;
      if (digitDP[DPpos] && (brightness>0)) digitalWrite(DP_PIN,HIGH); //switch ON decimal point, if needed
      for (int i=0;i<4;i++) {digitalWrite(ABCDPins[i],num  & 1<<i); }
  }
    
  #if COLON_PIN >= 0
    if (num==10) digitalWrite(COLON_PIN,LOW);      // Colon pin OFF
    else digitalWrite(COLON_PIN,colonBlinkState);  // Blink colon pin
  #endif
  
  #if DECIMALPOINT_PIN >=0 
        if (num==10) {digitalWrite(DECIMALPOINT_PIN,LOW);}
        else digitalWrite(DECIMALPOINT_PIN,decimalpointON); }
  #endif
  
  if (timer<500) timer = 500;  //safety only...
  timer1_write(timer); 
}

void ICACHE_RAM_ATTR clearTubes() {
    digitalWrite(DP_PIN,LOW);
    sendBits(I2C_ADDR,0);
    //for (int i=0;i<4;i++) digitalWrite(ABCDPins[i],HIGH); 
}

void writeDisplaySingle() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\pwmleds.ino"
#ifdef USE_PWMLEDS

#define FPS 60 //frame per sec
#define FPS_MSEC 1000/FPS
#define COLORSATURATION 255
#define WHITE_INDEX 192

long pwmBrightness;
int pwmColorStep = 1;
int pwmRed,pwmGreen,pwmBlue = 0;

void setPWMcolor(int WheelPos, byte brightness) {
#ifdef USE_PWMLEDS

  int pwmRed,pwmGreen,pwmBlue;

  if (WheelPos == WHITE_INDEX) {
    pwmRed = 255; pwmGreen = 255; pwmBlue = 255;
  }
  else if (WheelPos<0) {
    pwmRed =0; pwmGreen = 0; pwmBlue = 0;
  }
  else {  
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85)  {
      pwmRed =255 - WheelPos * 3; pwmGreen = 0; pwmBlue = WheelPos * 3;
    } else if(WheelPos < 170) {
      WheelPos -= 85;
      pwmRed = 0, pwmGreen = WheelPos * 3; pwmBlue = 255 - WheelPos * 3;
    } else  {
      WheelPos -= 170;
      pwmRed = WheelPos * 3; pwmGreen = 255 - WheelPos * 3; pwmBlue = 0;
    }
  }

  #if PWM1_PIN>=0
    ledcWrite(0, pwmRed * brightness / 255);
  #endif
  #if PWM2_PIN>=0
    ledcWrite(1, pwmGreen * brightness / 255);
  #endif
  #if PWM3_PIN>=0
    ledcWrite(2, pwmBlue * brightness / 255);
  #endif
#endif  
}


void pwmRainbow() {
 static byte j=0;
   
  setPWMcolor(j,pwmBrightness);
  if (prm.rgbDir) { j++; }
  else { j--; }
}

void pwmAlarmLight() {
  static unsigned long lastRun = 0;   
  static byte counter;
  
  if ((millis()-lastRun)<500) return;
  lastRun = millis();
  counter++;
  if (counter%2)
    setPWMcolor(WHITE_INDEX,255);
  else  
    setPWMcolor(-1,0);
}


void doAnimationPWM() {
static unsigned long lastRun = 0;

  if (EEPROMsaving) return;
  if (alarmON) {
    pwmAlarmLight();
    return;
  }
  
  if ((prm.rgbEffect <=1) && ((millis()-lastRun)<1000)) return;  //fix color
  if ((millis()-lastRun)<max(FPS_MSEC,258-prm.rgbSpeed)) return;
  lastRun = millis();

  if ((prm.rgbEffect == 0) || !displayON || !radarON) {   //switch RGB backlight OFF
    pwmBrightness = 0;
    setPWMcolor(-1,0);
    return;
  }
  
  pwmColorStep = max(1,prm.rgbSpeed/5);
  pwmBrightness = prm.rgbBrightness;
  if (autoBrightness) {
    pwmBrightness = (pwmBrightness * lx) /(long) MAXIMUM_LUX/2;
    if (pwmBrightness< c_MinBrightness) 
            pwmBrightness = c_MinBrightness;
  }
  pwmBrightness = min(pwmBrightness,long(RGB_MAX_BRIGHTNESS));  //safety only

  //DPRINTLN("  NeoBrightness:"); DPRINT(neoBrightness);
  
  if (prm.rgbEffect==1) setPWMcolor(prm.rgbFixColor,pwmBrightness);
  else pwmRainbow(); //flow
}

#else
void doAnimationPWM() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
#ifdef USE_RTC
//DS3231 realtime clock driver
//with manual clock setup buttons
#include <RtcDS3231.h>   //Makuna/RTC
RtcDS3231<TwoWire> Rtc(Wire);
RtcDateTime rtcNow;
RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
#define EPOCH_OFFSET 946684800l   //offset from 1970->2000 epoch start

static uint8_t rtcBcdToDec(uint8_t v) {
  return (uint8_t)(((v >> 4) * 10) + (v & 0x0F));
}

static uint8_t rtcDecToBcd(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static bool pcf8563ReadDateTime(int &yy, int &mo, int &dd, int &hh, int &mm, int &ss) {
  Wire.beginTransmission(0x51);
  Wire.write((uint8_t)0x02);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)0x51, (uint8_t)7) != 7) return false;

  uint8_t regSec = Wire.read();
  uint8_t regMin = Wire.read();
  uint8_t regHour = Wire.read();
  uint8_t regDay = Wire.read();
  (void)Wire.read(); // weekday
  uint8_t regMonth = Wire.read();
  uint8_t regYear = Wire.read();

  if (regSec & 0x80) {
    DPRINTLN("PCF8563: voltage low flag set (time may be invalid)");
  }

  ss = rtcBcdToDec(regSec & 0x7F);
  mm = rtcBcdToDec(regMin & 0x7F);
  hh = rtcBcdToDec(regHour & 0x3F);
  dd = rtcBcdToDec(regDay & 0x3F);
  mo = rtcBcdToDec(regMonth & 0x1F);

  uint8_t y2 = rtcBcdToDec(regYear);
  bool centuryBit = (regMonth & 0x80) != 0;
  yy = centuryBit ? (1900 + y2) : (2000 + y2);

  if (yy < 2000 || yy > 2099 || mo < 1 || mo > 12 || dd < 1 || dd > 31 || hh > 23 || mm > 59 || ss > 59) {
    return false;
  }
  return true;
}

static bool pcf8563WriteDateTime(int yy, int mo, int dd, int hh, int mm, int ss) {
  if (yy < 2000 || yy > 2099) return false;
  if (mo < 1 || mo > 12 || dd < 1 || dd > 31 || hh < 0 || hh > 23 || mm < 0 || mm > 59 || ss < 0 || ss > 59) return false;

  uint8_t y2 = (uint8_t)(yy % 100);
  uint8_t monthReg = rtcDecToBcd((uint8_t)mo); // century bit 0 => 20xx

  Wire.beginTransmission(0x51);
  Wire.write((uint8_t)0x02);
  Wire.write(rtcDecToBcd((uint8_t)ss) & 0x7F);
  Wire.write(rtcDecToBcd((uint8_t)mm) & 0x7F);
  Wire.write(rtcDecToBcd((uint8_t)hh) & 0x3F);
  Wire.write(rtcDecToBcd((uint8_t)dd) & 0x3F);
  Wire.write((uint8_t)0x00); // weekday not used
  Wire.write(monthReg & 0x1F);
  Wire.write(rtcDecToBcd(y2));
  return Wire.endTransmission() == 0;
}
//------ Mode Switch and Push Buttons ---------------------
#ifndef PIN_MODE_SWITCH
  #define PIN_MODE_SWITCH -1  
#endif
#ifndef PIN_FLD_BUTTON
  #define PIN_FLD_BUTTON -1
#endif
#ifndef PIN_SET_BUTTON
  #define PIN_SET_BUTTON -1    
#endif

boolean pushedFldButtonValue = LOW;   //wich state means pushed button
boolean pushedSetButtonValue = LOW;   //wich state means pushed button

//------- I2C bus definition   Any pins are usable  Please, SET in clocks.h --------
//#define PIN_SDA 4           //D2   original general setup for 8266
//#define PIN_SCL 5           //D1

#define MENU_UNSELECT_TIMEOUT 30000

int monthdays[13] = {0,31,29,31,30,31,30,31,31,30,31,30,31};

struct Menu {
   short lowlimit;        // alsó határ
   short uplimit;         // felső határ
   char name[5];
};

const struct Menu m1[]= {
  {0,0,"    "},
  {2022,2099,"Year"},
  {1,12,"Mon "},
  {1,31,"Day "},
  {0,23,"Hour"},
  {0,59,"Min "},
};

#define MAXFLD 5 

int mvar[MAXFLD+1];

enum switchStates {IS_OPEN, IS_OPENING, IS_PUSHED, IS_LONGPRESSING, IS_LONGPRESSED, IS_PUSHING, IS_LONGOPENING};

int fld = 0;
unsigned long LastModify = 0;

void setupRTC() {
  #if PIN_MODE_SWITCH>=0
    pinMode(PIN_MODE_SWITCH,INPUT_PULLUP);   
    regPin(PIN_MODE_SWITCH,"PIN_MODE_SWITCH"); 
  #endif
  #if PIN_FLD_BUTTON>=0  
    pinMode(PIN_FLD_BUTTON,INPUT_PULLUP);    
    regPin(PIN_FLD_BUTTON,"PIN_FLD_BUTTON"); 
    if (digitalRead(PIN_FLD_BUTTON) == LOW)  //inverted button?
      pushedFldButtonValue = HIGH;
  #endif    
  #if PIN_SET_BUTTON>=0  
    pinMode(PIN_SET_BUTTON,INPUT_PULLUP);    
    regPin(PIN_SET_BUTTON,"PIN_SET_BUTTON"); 
    if (digitalRead(PIN_SET_BUTTON) == LOW)  //inverted button?
      pushedSetButtonValue = HIGH;
  #endif    
  
  if (RTCexist) {
    if (RTCisPCF8563) {
      DPRINTLN("Connecting to RTC clock (PCF8563)...");
      int yy, mo, dd, hh, mm, ss;
      if (!pcf8563ReadDateTime(yy, mo, dd, hh, mm, ss)) {
        DPRINTLN("PCF8563 time invalid, setting compile time...");
        pcf8563WriteDateTime(compiled.Year(), compiled.Month(), compiled.Day(), compiled.Hour(), compiled.Minute(), compiled.Second());
      }
      return;
    }

    DPRINTLN("Connecting to RTC clock...");
    Rtc.Begin();
    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) {
            DPRINT("RTC communications error = ");  DPRINTLN(Rtc.LastError());
        }
        else {
            DPRINTLN("RTC lost confidence in the DateTime!");
            Rtc.SetDateTime(compiled);  //Set RTC time to firmware compile time
        }
    }

    if (!Rtc.GetIsRunning())  {
        DPRINTLN("RTC was not actively running, starting now");
        Rtc.SetIsRunning(true);
    }

    Rtc.Enable32kHzPin(false);
    Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);     
  }
/*  
while (true) {
  DPRINT(digitalRead(PIN_MODE_SWITCH)); DPRINT(" / "); 
  DPRINT(digitalRead(PIN_FLD_BUTTON));  DPRINT(" / ");
  DPRINTLN(digitalRead(PIN_SET_BUTTON));
  delay(1000);
  }
*/  
}


void editor() {
  static unsigned long lastDisplay = 0;
  if (RTCexist && !RTCisPCF8563) {rtcNow = Rtc.GetDateTime();}
  LastModify = 0;
  #if PIN_FLD_BUTTON >= 0 && PIN_SET_BUTTON >= 0   //Are buttons installed?
    editorRunning = true;
    while (true) {
      scanButFLD(100); 
      if (fld == 0) {
        editorRunning = false;
        break;
      }
      scanButSET(100);
      showValue();
      #if defined(WORDCLOCK)
        if ((millis()-lastDisplay)>100) {
          lastDisplay = millis();
          writeDisplay2();
        }
      #else
        writeDisplaySingle();
      #endif
      printDigits(1000);
      Fdelay(100);
    }
  #endif
  if (LastModify != 0) saveRTC();
}

void showValue() {
  memset(digit,10,sizeof(digit));   //clear array
  memset(digitDP,0,sizeof(digitDP));
  if(fld==1) {
    digit[3] = mvar[1] / 1000;
    digit[2] = (mvar[1] % 1000) / 100;    
    digit[1] = (mvar[1] % 100) / 10;
    digit[0] = mvar[1] % 10;
  }      
  else {
    digit[1] = mvar[fld] / 10;
    digit[0] = mvar[fld] % 10;
  }
  #if defined(MAX6921) || defined(MAX6921_ESP32)
    //character set not available yet
  #else
  if (!digitsOnly && (maxDigits>4)) {
    if (maxDigits>=8) digit[7] = m1[fld].name[0];
    if (maxDigits>=8) digit[6] = m1[fld].name[1];
    if (maxDigits>=6) digit[5] = m1[fld].name[2];
    else digit[5] = ':';
    digit[4] = m1[fld].name[3];
    
  }
  #endif
}

//------------------------------------- RTC Clock functions --------------------------------------------------------------------
void updateRTC() {
  DPRINTLN("Update RTC?");
  if (!RTCexist) return;
  if (year()<2020) return;    //invalid system date??

  if (RTCisPCF8563) {
    int yy, mo, dd, hh, mm, ss;
    if (!pcf8563ReadDateTime(yy, mo, dd, hh, mm, ss)) {
      DPRINTLN("PCF8563 read failed in updateRTC()");
      return;
    }
    if (abs(ss - second())>2 || (mm != minute()) || (hh != hour()) || (dd != day()) || (mo != month()) || (yy != year())) {
      DPRINTLN("Updating PCF8563 from TimeServer...");
      pcf8563WriteDateTime(year(), month(), day(), hour(), minute(), second());
    }
    return;
  }

  rtcNow = Rtc.GetDateTime();
  if (abs(rtcNow.Second() - second())>2 || (rtcNow.Minute() != minute()) || (rtcNow.Hour() != hour())  
      || (rtcNow.Day() != day()) || (rtcNow.Month() != month()) || (rtcNow.Year() != year()) )   {
      DPRINTLN("Updating RTC from TimeServer...");
      DPRINT("- DS3231  date:"); DPRINT(rtcNow.Year()); DPRINT("/"); DPRINT(rtcNow.Month()); DPRINT("/"); DPRINT(rtcNow.Day()); 
      DPRINT(" time:");  DPRINT(rtcNow.Hour()); DPRINT(":"); DPRINT(rtcNow.Minute()); DPRINT(":"); DPRINTLN(rtcNow.Second());    
      DPRINT("- Tserver date:"); DPRINT(year()); DPRINT("/"); DPRINT(month()); DPRINT("/"); DPRINT(day());      
      DPRINT(" time:");   DPRINT(hour()); DPRINT(":"); DPRINT(minute()); DPRINT(":"); DPRINTLN(second());  
      Rtc.SetDateTime(now()-EPOCH_OFFSET);   //update RTC with UNIX epoch timestamp to 2000year epoch
      }  //endif
}


void getRTC() {
  if (RTCexist) {
    if (RTCisPCF8563) {
      int yy, mo, dd, hh, mm, ss;
      if (pcf8563ReadDateTime(yy, mo, dd, hh, mm, ss)) {
        setTime(hh, mm, ss, dd, mo, yy);
      } else {
        DPRINTLN("PCF8563 read failed");
      }
      mvar[1] = year(); mvar[2] = month(); mvar[3] = day();
      mvar[4] = hour(); mvar[5] = minute();
      return;
    }

    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) {
            DPRINT("RTC communications error = "); DPRINTLN(Rtc.LastError());
        }
        else {
            DPRINTLN("RTC lost confidence in the DateTime!");
        }
    }
    else {
      rtcNow = Rtc.GetDateTime();
      //DPRINT("\nGet DS3231 data&time: ");  DPRINT(rtcNow.Year()); DPRINT("/"); DPRINT(rtcNow.Month()); DPRINT("/"); DPRINT(rtcNow.Day()); 
      //DPRINT(" ");  DPRINT(rtcNow.Hour()); DPRINT(":"); DPRINT(rtcNow.Minute()); DPRINT(":"); DPRINTLN(rtcNow.Second());  
      setTime(rtcNow.Hour(),rtcNow.Minute(),rtcNow.Second(),rtcNow.Day(),rtcNow.Month(),rtcNow.Year());  //set the time (hr,min,sec,day,mnth,yr)
    }
  }
  mvar[1] = year(); mvar[2] = month(); mvar[3] = day();
  mvar[4] = hour(); mvar[5] = minute(); 
}

void saveRTC() {
  setTime(mvar[4],mvar[5],0,mvar[3],mvar[2],mvar[1]);  //set the time (hr,min,sec,day,mnth,yr)

    if (RTCexist) {
      DPRINTLN("Updating RTC from Manual settings...");
      if (RTCisPCF8563) {
        pcf8563WriteDateTime(year(), month(), day(), hour(), minute(), second());
      } else {
        Rtc.SetDateTime(now()-EPOCH_OFFSET);
      }
    }
    DPRINT("New Date & Time:"); DPRINT(year()); DPRINT("/"); DPRINT(month()); DPRINT("/"); DPRINT(day()); 
    DPRINT(" time:");  DPRINT(hour()); DPRINT(":"); DPRINT(minute()); DPRINT(":"); DPRINTLN(second());  
}

//-------------------------- check buttons  ------------------------------------------------

void scanButFLD(unsigned long mill) {
static unsigned long lastRun = millis(); 
static unsigned long lastPush = millis();; 
static switchStates butState;
byte sw;

  if (PIN_FLD_BUTTON<0) return;
  if ((millis() - lastRun) < mill) return;   //refresh rate
  lastRun = millis(); 

  sw = digitalRead(PIN_FLD_BUTTON);
  if (pushedFldButtonValue == HIGH) sw = !sw;   //inverted button logic
  
  switch (butState) {
    case IS_OPEN:   
      if(sw == LOW) butState = IS_PUSHING;  
      break; 
    case IS_PUSHING: 
      butState = IS_PUSHED;  
      lastPush = millis();
      DPRINTLN(" PUSHED");
      break; 
    case IS_PUSHED:   
      if(sw == HIGH)  butState = IS_OPENING; 
      if((sw == LOW) && ((millis()-lastPush)>2000))  butState = IS_LONGPRESSING; 
      break; 
    case IS_LONGPRESSING:  
      butState = IS_LONGPRESSED; 
      DPRINTLN(" IS_LONGPRESSED");
      if (fld>0) {
        fld = 0;
        saveRTC();
      }
      else {
        fld = 1;
        getRTC();
      }
      break; 
    case IS_LONGPRESSED:  
      if (sw == HIGH) butState = IS_LONGOPENING; 
      break;      
    case IS_OPENING:
      butState = IS_OPEN;
      if ((millis()-lastPush)<500) {
           LastModify = millis();
          fld++;  if (fld>MAXFLD) fld = 1;
          DPRINT(m1[fld].name); DPRINTLN(fld);
       } //endif millis()
      break; 
    case IS_LONGOPENING: 
      butState = IS_OPEN;
      break;    
  } //end switch
}

void scanButSET(unsigned long mill) {
static unsigned long lastRun = millis(); 
static unsigned long lastPush = millis(); 
static switchStates butState;
byte sw;

  if (PIN_SET_BUTTON<0) return;
  if ((millis() - lastRun) < mill) return;   //refresh rate
  lastRun = millis(); 

  sw = digitalRead(PIN_SET_BUTTON);
  if (pushedSetButtonValue == HIGH) sw = !sw;   //inverted button logic  
  
  switch (butState) {
    case IS_OPEN:   
      if(sw == LOW) butState = IS_PUSHING;  
      break; 
    case IS_PUSHING: 
      butState = IS_PUSHED;  
      lastPush = millis();
      DPRINTLN(" PUSHED");
      break; 
    case IS_PUSHED:   
      if(sw == HIGH)  butState = IS_OPENING; 
      if((sw == LOW) && ((millis()-lastPush)>2000))  butState = IS_LONGPRESSING; 
      break; 
    case IS_LONGPRESSING:  
      butState = IS_LONGPRESSED; 
      DPRINTLN(" IS_LONGPRESSED");
      //if (noZero !=0) noZero =0; else noZero = 1;
      //EEPROM.update(EEPROM_NOZERO, noZero);  
      break; 
    case IS_LONGPRESSED:  
      if (sw == HIGH) butState = IS_LONGOPENING; 
       // if ((millis()-lastPush)>10000)  {factoryReset(); resetFunc();}   //reset to factory settings
       LastModify = millis();
       mvar[fld]+=1;
       if (mvar[fld] < m1[fld].lowlimit)  mvar[fld] = m1[fld].uplimit;
       if (mvar[fld] > m1[fld].uplimit)  mvar[fld] = m1[fld].lowlimit;
       DPRINT(fld); DPRINT(" : "); DPRINTLN(mvar[fld]);
       break;
    case IS_OPENING:
      butState = IS_OPEN;
      if ((millis()-lastPush)<500) {
          LastModify = millis();
          mvar[fld]++;
          if (mvar[fld] < m1[fld].lowlimit)  mvar[fld] = m1[fld].uplimit;
          if (mvar[fld] > m1[fld].uplimit)  mvar[fld] = m1[fld].lowlimit;
          //if (fld <= 2) mvar[2] = monthday_check(mvar[RTC_Ev], mvar[RTC_Ho], mvar[RTC_Nap]);    //szökőév ellenőrzés           
          DPRINT(fld); DPRINT(" : "); DPRINTLN(mvar[fld]);
          if (fld>MAXFLD) fld = 0;
          //flash(fld);
       } //endif millis()
      break; 
    case IS_LONGOPENING: 
      butState = IS_OPEN;
      break;    
  } //end switch
}


#if defined(PIN_SDA) && defined(PIN_SCL)
int I2C_ClearBus() {
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif
  DPRINTLN("I2C_ClearBus() started.");
  pinMode(PIN_SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(PIN_SCL, INPUT_PULLUP);

  Fdelay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(PIN_SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(PIN_SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(PIN_SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(PIN_SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(PIN_SCL, INPUT); // release SCL LOW
    pinMode(PIN_SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(PIN_SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(PIN_SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(PIN_SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(PIN_SDA, INPUT); // remove pullup.
  pinMode(PIN_SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(PIN_SDA, INPUT); // remove output low
  pinMode(PIN_SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(PIN_SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(PIN_SCL, INPUT);
  return 0; // all ok
}
#else
int I2C_ClearBus() {return 1;}
#endif

//------------------------------------------------------------------------------------------
#else
  void updateRTC() {}
  void setupRTC() {}
  void getRTC() {}
  void editor() {}
#endif

#line 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
// NeoPixel Tube Backlight

#ifdef USE_NEOPIXEL

#include "settings.h"  // persisted UI settings (debug, RGB fixed color, etc.)

long neoBrightness;
int colorStep = 1;

#define FPS 60 //frame per sec
#define FPS_MSEC 1000/FPS
#define COLORSATURATION 255
#define WHITE_INDEX 192
#define RANDOM_WHEEL_DISTANCE  30  //how far colors will get in random mode
#define RANDOM_MAX_COUNTER 100      //maximum how many times try to found a new color
#define RANDOM_FROM_ALL_PIXELS true  //true or false: when generating new colors, the distance must be calculated from all pixels or only from the actual pixel's color 

RgbColor red(COLORSATURATION, 0, 0);
RgbColor red2(COLORSATURATION/2, 0, 0);
RgbColor green(0, COLORSATURATION, 0);
RgbColor blue(0, 0, COLORSATURATION);
RgbColor purple(COLORSATURATION, 0, COLORSATURATION);
RgbColor white(COLORSATURATION/2,COLORSATURATION/2,COLORSATURATION/2);
RgbColor black(0,0,0);

//Definition: wich pixels are near wich tube?  Make sure to set all pixels of the stripe! 
// Valid: 0..(maxDigits-1) If a number is equal maxDigits or higher, it will stay always dark!
//byte tubePixels[] = {0,1,2,3};        //4 tubes, single leds
//byte tubePixels[] = {3,2,1,0};        //4 tubes, single leds, reverse direction
//byte tubePixels[] = {0,9,1,9,2,9,3};  //4 tubes, single leds, 3 leds not used
//byte tubePixels[] = {0,0,1,2,3,3};    //4 tubes, 6 leds
//byte tubePixels[] = {0,1,2,3,4,5};    //6 tubes, single leds
//byte tubePixels[] = {5,4,3,2,1,0};    //6 tubes, single leds, reverse direction
//byte tubePixels[] = {0,1,2,3,4,5,6,7};    //8 tubes, single leds
//byte tubePixels[] = {3,2,6,1,0};    //Numitron 4 tubes, 4 x single leds + 1. The extra led in the middle is not used, is always dark!
//byte tubePixels[] = {0,1,2,3,3,2,1,0};  //4 tubes, double row, 8 leds
//byte tubePixels[] = {0,0,1,1,2,2,3,3};  //4 tubes, double row, 8 leds
//byte tubePixels[] = {3,3,2,2,1,1,0,0, 0,0,1,1,2,2,3,3};  //4 tubes, double row, 16 leds (GB)
//byte tubePixels[] = {0,0,1,1,2,2,3,3,3,3,2,2,1,1,0,0,0};  //4 tubes, double row, 17 leds (GP)
//byte tubePixels[] = {0,0,0,1,1,2,2,3,3,3,3,  3,3,2,2,2,1,1,0,0};  //4 tubes, double row, 20 leds (Robi)

const int TubePixelMapCount = sizeof(tubePixels);
const int PixelCount = TubePixelMapCount + 2;
const int StripPixelCount = PixelCount + 4;

inline bool isMappedActivePixel(int idx) {
  return (idx >= 0) && (idx < TubePixelMapCount) && (tubePixels[idx] < maxDigits);
}

//NeoGrbFeature give me BRGW (g and b swapped)
//NeoRgbFeature give me RBGW (g and b swapped)
//NeoBrgFeature give me BGRW (g and r swapped)

#if defined(ESP32)
  byte PixelPin = NEOPIXEL_PIN;  //on ESP32 usable any pin below 32 
  NeoPixelBrightnessBus<NeoGrbFeature, NeoEsp32Rmt7Ws2812xMethod> strip(StripPixelCount,PixelPin);
#else
  #define NEOPIXEL_PIN 3
  byte PixelPin = 3;  // on 8266 it MUST use GPIO3 (RX pin)    
  NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> strip(StripPixelCount);   
#endif
                                                                              
NeoGamma<NeoGammaTableMethod> colorGamma;

volatile uint16_t neoAppliedCurrentmA = 0;
volatile uint8_t neoAppliedBrightness = 0;
static const bool NEO_DOUBLE_SHOW = false;

static inline void showStableFrame(bool forceDoubleFrame = false) {
  strip.Show();
  #if defined(ESP32)
  if (forceDoubleFrame || NEO_DOUBLE_SHOW) {
    delayMicroseconds(300);
    strip.Show();
  }
  #endif
}

// Estimate LED current for the current frame and clamp brightness so we don't exceed prm.maxLedmA.
// Assumptions: WS2812B ~60mA per pixel at full white (R=G=B=255) at brightness 255.
static void applyCurrentLimitAndShow(uint8_t requestedBrightness) {
  // Always keep unmapped/extra pixels off to avoid stale or random flashes.
  for (int i = 0; i < StripPixelCount; i++) {
    if (!isMappedActivePixel(i)) {
      strip.SetPixelColor(i, black);
    }
  }

  // If brightness is zero: just turn off
  if (requestedBrightness == 0) {
    neoAppliedCurrentmA = 0;
    neoAppliedBrightness = 0;
    strip.SetBrightness(0);
    showStableFrame(true);
    return;
  }

  uint16_t limitmA = prm.maxLedmA;
  if (limitmA == 0) {
    // Limiting disabled
    neoAppliedCurrentmA = 0;
    neoAppliedBrightness = requestedBrightness;
    strip.SetBrightness(requestedBrightness);
    showStableFrame();
    return;
  }

  // Sum RGB for active pixels (masked pixels are excluded)
  uint32_t sumRGB = 0;
  for (int i = 0; i < PixelCount; i++) {
    if (!isMappedActivePixel(i)) continue;
    RgbColor c = strip.GetPixelColor(i);
    sumRGB += (uint32_t)c.R + c.G + c.B;
  }

  // Estimated current at requested brightness:
  // I ≈ 60mA * sumRGB / (3*255) * (brightness/255)
  // => I ≈ 60 * sumRGB * brightness / (3*255*255)
  uint32_t estmA = (60UL * sumRGB * (uint32_t)requestedBrightness) / (3UL * 255UL * 255UL);

  if (estmA == 0) {
    neoAppliedCurrentmA = 0;
    neoAppliedBrightness = requestedBrightness;
    strip.SetBrightness(requestedBrightness);
    showStableFrame();
    return;
  }

  uint8_t appliedBrightness = requestedBrightness;
  if (estmA > limitmA) {
    uint32_t newB = ((uint32_t)requestedBrightness * (uint32_t)limitmA) / estmA;
    if (newB < 1) newB = 1;
    if (newB > 255) newB = 255;
    appliedBrightness = (uint8_t)newB;
  }

  uint32_t appliedEstmA = (estmA * (uint32_t)appliedBrightness) / (uint32_t)requestedBrightness;
  if (appliedEstmA > 65535UL) appliedEstmA = 65535UL;
  neoAppliedCurrentmA = (uint16_t)appliedEstmA;
  neoAppliedBrightness = appliedBrightness;
  strip.SetBrightness(appliedBrightness);
  showStableFrame();
}

void setupNeopixel() {
    DPRINTLN("Setup NeoPixel LEDS");  
    regPin(PixelPin,"NEOPIXEL_PIN");
    DPRINT("Pixel count: "); DPRINTLN(PixelCount);
    DPRINT("Brightness:"); DPRINT(c_MinBrightness); DPRINT(" - "); DPRINTLN(c_MaxBrightness);
    neoBrightness = prm.rgbBrightness;
    strip.Begin();
    strip.ClearTo(black);
    strip.SetBrightness(0);
    showStableFrame();
    strip.SetBrightness(neoBrightness);
    fixColor();
}

RgbColor Wheel(byte WheelPos) {
    WheelPos = WheelPos % 256; // Ensure the position is within the valid range
    //DPRINT("WheelPos: "); DPRINTLN(WheelPos);

    if (WheelPos == WHITE_INDEX) {
        //DPRINTLN("Returning white color");
        return white;
    }

    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        return RgbColor(255 - WheelPos * 3, 0, WheelPos * 3); // Red to blue
    } else if (WheelPos < 170) {
        WheelPos -= 85;
        return RgbColor(0, WheelPos * 3, 255 - WheelPos * 3); // Green to blue
    } else {
        WheelPos -= 170;
        return RgbColor(WheelPos * 3, 255 - WheelPos * 3, 0); // Blue to red
    }
}

inline byte randomWheelNoWhite() {
  byte c;
  do {
    c = (byte)random(0, 256);
  } while (c == WHITE_INDEX);
  return c;
}

void alarmLight() {
  static unsigned long lastRun = 0;   
  static byte counter;
  
  if ((millis()-lastRun)<500) return;
  lastRun = millis();
  counter++;
  strip.SetBrightness(255);
  for (int i = 0; i < StripPixelCount; i++) {
    if ((counter % 2) && isMappedActivePixel(i)) {
      strip.SetPixelColor(i, RgbColor(255,255,255));
    } else {
      strip.SetPixelColor(i, black);
    }
  }
  neoAppliedCurrentmA = 0;
  neoAppliedBrightness = 255;
  showStableFrame(true);
}

void darkenNeopixels() {
    neoBrightness = 0;
    strip.SetBrightness(0);
    for (int i=0;i<StripPixelCount;i++) {
      strip.SetPixelColor(i,black);
    } 
  applyCurrentLimitAndShow(0);
}


void showSolidNeopixels(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  RgbColor c(r, g, b);
  for (int i = 0; i < PixelCount; i++) {
    if (!isMappedActivePixel(i)) strip.SetPixelColor(i, black);
    else strip.SetPixelColor(i, c);
  }
  applyCurrentLimitAndShow(neoBrightness);
}

void rainbow() {
    static byte j = 0;
    static unsigned long lastRun = 0;
    unsigned long spd = max(10, (258 - prm.rgbSpeed)); // Minimum delay of 10ms

    // Enforce frame delay based on speed
    if ((millis() - lastRun) < spd) return;
    lastRun = millis();

    for (int i = 0; i < PixelCount; i++) {
        int index = (i + j) & 255; // Cycle through the color wheel
        if (index == WHITE_INDEX) index++; // Skip white
      if (isMappedActivePixel(i)) {
            strip.SetPixelColor(i, Wheel(index));
        } else {
            strip.SetPixelColor(i, black);
        }
    }
    j++; // Increment to shift the rainbow
    applyCurrentLimitAndShow(neoBrightness);
}

void rainbow2() {
    static int16_t j = 0;
    static int16_t i = 0;
    static unsigned long lastRun = 0;
    int steps = 15;
    unsigned long spd = max(0, steps * (258 - prm.rgbSpeed));

    if ((millis() - lastRun) < spd) return;
    lastRun = millis();

    if (i >= maxDigits) {
        if (j > 255) j = 0;
        i = 0;
        j += steps;
    }
    if (i < 0) {
        if (j > 255) j = 0;
        i = maxDigits - 1;
        j += steps;
    }

    RgbColor color = Wheel(j);
    setPixels(i, color);

    if (prm.rgbDir) i++;
    else i--;

    applyCurrentLimitAndShow(neoBrightness);
}

#include <math.h> // For sin() and PI

void effect1() { // Ultra-Smooth Color Dimmer with Stability Fix
    static float brightness = 0.0;        // Current brightness (0.0 to 1.0)
    static float brightnessStep = 0.01;  // Step size for brightness changes
    static float colorBlendRatio = 0.0;  // Ratio for blending between two colors
    static RgbColor currentColor = Wheel(0); // Starting color
    static RgbColor nextColor = Wheel(randomWheelNoWhite()); // Target color
    static unsigned long lastFrame = 0;

    // Frame timing: Update every ~5ms (200 FPS)
    if (millis() - lastFrame < 5) return;
    lastFrame = millis();

    // Adjust brightness smoothly
    brightness += brightnessStep;
    if (brightness >= 1.0) {
        brightness = 1.0;
        brightnessStep = -fabs(brightnessStep); // Start dimming
    } else if (brightness <= 0.0) {
        brightness = 0.0;
        brightnessStep = fabs(brightnessStep); // Start brightening
        currentColor = nextColor; // Transition to the next color
        nextColor = Wheel(randomWheelNoWhite()); // Pick a new target color
    }

    // Adjust color blending smoothly
    colorBlendRatio += fabs(brightnessStep) * 0.5; // Faster blending at higher speed
    if (colorBlendRatio > 1.0) colorBlendRatio = 1.0;
    RgbColor blendedColor = blendColors(currentColor, nextColor, colorBlendRatio);

    // Apply brightness scaling and blended color to all LEDs
    RgbColor dimmedColor = scaleColor(blendedColor, brightness);
    for (int i = 0; i < PixelCount; i++) {
      if (!isMappedActivePixel(i)) {
            strip.SetPixelColor(i, black);
        } else {
            strip.SetPixelColor(i, dimmedColor);
        }
    }

    // Stabilize LED updates
    applyCurrentLimitAndShow(neoBrightness);
// Push updated colors
    delayMicroseconds(50); // Ensure data line is stable before next frame
}

// Helper Function: Blend Two Colors Smoothly
RgbColor blendColors(RgbColor color1, RgbColor color2, float ratio) {
    byte r = (1 - ratio) * color1.R + ratio * color2.R;
    byte g = (1 - ratio) * color1.G + ratio * color2.G;
    byte b = (1 - ratio) * color1.B + ratio * color2.B;
    return RgbColor(r, g, b);
}

void effect2() {   //Color FlowChanger (smooth full-strip transition)
  static bool firstRun = true;
  static uint8_t currWheel = 0;
  static uint8_t nextWheel = 0;
  static uint8_t blend = 0;   //0..255
  static unsigned long lastRun = 0;

  const uint8_t blendStep = (uint8_t)max(1, prm.rgbSpeed / 18);      //speed-scaled transition step
  const unsigned long frameDelay = (unsigned long)max(8, 120 - (prm.rgbSpeed / 2));

  if ((millis() - lastRun) < frameDelay) return;
  lastRun = millis();

  if (firstRun) {
    firstRun = false;
    currWheel = randomWheelNoWhite();
    nextWheel = randomWheelNoWhite();
    while (colorDistance(currWheel, nextWheel) < RANDOM_WHEEL_DISTANCE) {
      nextWheel = randomWheelNoWhite();
    }
    blend = 0;
  }

  if (blend >= 255) {
    currWheel = nextWheel;
    nextWheel = randomWheelNoWhite();
    while (colorDistance(currWheel, nextWheel) < RANDOM_WHEEL_DISTANCE) {
      nextWheel = randomWheelNoWhite();
    }
    blend = 0;
  }

  RgbColor fromColor = Wheel(currWheel);
  RgbColor toColor = Wheel(nextWheel);
  float ratio = (float)blend / 255.0f;
  RgbColor flowColor = blendColors(fromColor, toColor, ratio);

  for (volatile int i = 0; i < PixelCount; i++) {
    if (!isMappedActivePixel(i)) strip.SetPixelColor(i, black);
    else strip.SetPixelColor(i, flowColor);
  }

  uint16_t nextBlend = (uint16_t)blend + blendStep;
  blend = (nextBlend > 255) ? 255 : (uint8_t)nextBlend;

  applyCurrentLimitAndShow(neoBrightness);
}

int colorDistance(int c1,int c2) {
  int d1,d2;
  d1 = abs(c1-c2);
  d2 = (c1<c2) ? c1 + 255-c2 : c2 + 255-c1;
  //DPRINT("c1:"); DPRINT(c1); DPRINT("  c2:"); DPRINT(c2); DPRINT("  d1:"); DPRINT(d1); DPRINT("  d2:"); DPRINTLN(d2);
  return min(d1,d2);
}

void effect3(boolean enableRandom,boolean eachPixelRandom) {
  static const int c[] = {255,5,12,22,30,40,54,62,78,85,100,110,122,137,177,210,227,240};
  static const int cMax = sizeof(c) / sizeof(c[0]);  //size of array
  static int newColor[10];
  static int oldColor[10];
  static int actColor[10];
  static int i = 2;
  static int step = 1;
  static int idx = 0;
  static boolean firstRun = true;
  static int counter = 0;
  static int dir = 1;
  int newC = 0;
  boolean changeColor = false;
  boolean colorOK;
  
  if (firstRun) {
    firstRun = false;
    for (volatile int k=0;k<maxDigits;k++) {
      oldColor[k] = 0;
      newColor[k] = 100;
      actColor[k] = 0;
    }
    step = max(1,abs(newColor[2]-oldColor[2])/20);
  }
  
  if (newColor[i] == actColor[i]) {      //newColor reached... 
    if (prm.rgbDir) i++;  else i--;   //goto next pixel
    
    if (i>=maxDigits) { i=0; changeColor = true;}
    else if (i<0) {i=maxDigits-1; changeColor = true;}
    
    if (eachPixelRandom) {   //each pixel is random color
      changeColor = true;
    }
    
    if (changeColor) {
      changeColor = false;
      if (enableRandom) {
        counter = 0;
        while (true) {  //random color
          newC = randomWheelNoWhite();   //get a new random color (exclude white)
          colorOK = true;
          if (RANDOM_FROM_ALL_PIXELS) {
            for (int j=0;j<maxDigits;j++) {
              if (colorDistance(newC,newColor[j]) < RANDOM_WHEEL_DISTANCE) 
                colorOK = false;   //here the oldColor is just stored in the newColor... :)
            }   
          }   
          else {  
            if (colorDistance(newC,newColor[i]) < RANDOM_WHEEL_DISTANCE)    //check random only from actual pixel
                colorOK = false;   //here the oldColor is just 
          }               
          counter++;
          if (colorOK || (counter>RANDOM_MAX_COUNTER))  break;
        } //end while 
        //DPRINT("Pix:"); DPRINT(i); DPRINT("  old:"); DPRINT(newColor[i]); DPRINT("  / newC:"); DPRINT(newC); DPRINT("  No of tries:"); DPRINTLN(counter);
      }
      else {  //new color from table
        idx++; if (idx>=cMax) idx = 0;
        newC =  c[idx]; 
      }
      
      if (eachPixelRandom) {
        oldColor[i] = newColor[i]; 
        newColor[i] = newC;
      }
      else { 
        for (volatile int k=0;k<maxDigits;k++) {
          oldColor[k] = newColor[k]; 
          newColor[k] = newC;
          }
      }   
      
    step = max(1,abs(newColor[i]-oldColor[i])/20);
    if (newColor[i] > oldColor[i])  
      dir = 1;
    else
      dir = -1;  
      
    //DPRINT("Old-New simple distance:"); DPRINT(abs(newC-oldColor[i]));  DPRINT("  step:"); DPRINTLN(step);      
    } //endif changeColor
    
    actColor[i] = oldColor[i];  //starting color} 
    }  //endif (newColor[i] == actColor[i])

  
  if (actColor[i] != newColor[i]) {   //next rainbow color
    if (dir==1) actColor[i] = min(newColor[i],actColor[i]+step);
    else actColor[i] = max(newColor[i],actColor[i]-step);
   }

  for (int j=0;j<maxDigits;j++) 
    setPixels(j, Wheel(actColor[j])); 
     
  applyCurrentLimitAndShow(neoBrightness);
//DPRINT("Pix:"); DPRINT(i); DPRINT(" Old:"); DPRINT(oldColor[i]); DPRINT(" ActCol:"); DPRINT(actColor[i]); DPRINT(" New:"); DPRINT(newColor[i]); DPRINT("  Step:"); DPRINTLN(step);
}


void effect4() {    //every pixel is random changer
  static int newColor[10];
  static int oldColor[10];
  static int actColor[10];
  static int step[10];
  static boolean firstRun = true;
  //unsigned long spd = max(0, 25*(258-prm.rgbSpeed));


  //if ((millis()-lastRun)<spd return;

  if (firstRun) {
    firstRun = false;
    for (int i=0;i<maxDigits;i++) {
      newColor[i] = randomWheelNoWhite();
      oldColor[i] = randomWheelNoWhite();
      actColor[i] = oldColor[i];
      step[i] = 1;
    }
  }
  
  for (int t=0;t<maxDigits;t++) {

  if (actColor[t] == newColor[t]) {  //change color
    oldColor[t] = newColor[t];
    do {
      newColor[t] = randomWheelNoWhite();   //get a new random color (exclude white)
    } while (colorDistance(newColor[t],oldColor[t])<RANDOM_WHEEL_DISTANCE);
    
    actColor[t] = oldColor[t];  //starting color} 
    step[t] = 1;   //max(1,colorDistance(newColor[t],oldColor[t])/20);
  } //endif changeColor

  //DPRINT(i); DPRINT("/"); DPRINTLN(j);    
  setPixels(t, Wheel(actColor[t]));  

  if (newColor[t] != actColor[t]) {   //next rainbow color
    if (oldColor[t] < newColor[t]) actColor[t] = min(newColor[t],actColor[t]+step[t]);
    else actColor[t] = max(newColor[t],actColor[t]-step[t]);
    
   }
     //DPRINT("Pix:"); DPRINT(i); DPRINT(" ActCol:"); DPRINTLN(actColor); 
  }  //end for t
  applyCurrentLimitAndShow(neoBrightness);
}

void setPixels(byte tubeNo, RgbColor c) {
  for (volatile int i=0;i<PixelCount;i++)  {
    if ((i < TubePixelMapCount) && (tubeNo == tubePixels[i])) {
      strip.SetPixelColor(i,c);
    }
  }    
}

void fixColor() {
    // Fixed RGB (user-selected)
    RgbColor c(settings.rgbFixR, settings.rgbFixG, settings.rgbFixB);
    for (int i = 0; i < PixelCount; i++) {
      if (!isMappedActivePixel(i)) strip.SetPixelColor(i, black);
      else strip.SetPixelColor(i, c);
    }
    applyCurrentLimitAndShow(neoBrightness);
}
// Backward-compatible wrapper: legacy calls used fixColor(-1)
void fixColor(int /*legacy*/) { fixColor(); }



void kitt() {
  static int dir = 1;  //direction
  static int counter = 0;
  static int counter2 = 0;

  counter2 ++;
  if (counter2 >=10) { 
    counter2 = 0;
    counter += dir;
    if (counter >= maxDigits-1) dir = -1;
    else if (counter <=0 ) dir = 1; 
  }
  for (int i=0;i<PixelCount;i++) {
    strip.SetPixelColor(i,black);
  }

  setPixels(counter, RgbColor(settings.rgbFixR, settings.rgbFixG, settings.rgbFixB));
  applyCurrentLimitAndShow(neoBrightness);
}

void heartbeat() {
    static float brightness = 0.0;        // Current brightness (0.0 to 1.0)
    static float brightnessStep = 0.0;   // Current step size for brightness changes
    static int phase = 0;                // Phase of the heartbeat cycle
    static unsigned long lastFrame = 0;

    unsigned long spd = max(5, (258 - prm.rgbSpeed)); // Frame delay, responsive to speed

    // Timing for 200 FPS updates
    if (millis() - lastFrame < spd) return;
    lastFrame = millis();

    // Adjust brightness and phase based on heartbeat cycle
    switch (phase) {
        case 0: // Ramp up for the first beat
            brightness += 0.05;
            if (brightness >= 1.0) {
                brightness = 1.0;
                phase = 1; // Move to hold at peak brightness
            }
            break;

        case 1: // Hold brightness briefly
            brightnessStep += 0.05;
            if (brightnessStep >= 0.2) { // Hold for 0.2 cycles
                brightnessStep = 0.0;
                phase = 2; // Move to ramp down
            }
            break;

        case 2: // Ramp down after the first beat
            brightness -= 0.1;
            if (brightness <= 0.5) {
                brightness = 0.5;
                phase = 3; // Move to second beat
            }
            break;

        case 3: // Ramp up for the second beat
            brightness += 0.05;
            if (brightness >= 1.0) {
                brightness = 1.0;
                phase = 4; // Hold at peak brightness for second beat
            }
            break;

        case 4: // Hold brightness briefly
            brightnessStep += 0.05;
            if (brightnessStep >= 0.1) { // Hold for 0.1 cycles
                brightnessStep = 0.0;
                phase = 5; // Ramp down after second beat
            }
            break;

        case 5: // Ramp down to complete the heartbeat
            brightness -= 0.02;
            if (brightness <= 0.0) {
                brightness = 0.0;
                phase = 0; // Restart the heartbeat cycle
            }
            break;
    }

    // Ensure brightness stays within bounds
    brightness = constrain(brightness, 0.0, 1.0);

    // Apply brightness to all LEDs (extra conservative: cap peak to reduce transient spikes)
    RgbColor heartbeatColor = RgbColor(180, 0, 0);
    RgbColor dimmedColor = scaleColor(heartbeatColor, brightness);

    // Build frame from known state each cycle
    for (int i = 0; i < StripPixelCount; i++) {
      strip.SetPixelColor(i, black);
    }

    for (int i = 0; i < PixelCount; i++) {
      if (!isMappedActivePixel(i)) {
            strip.SetPixelColor(i, black); // Turn off unused LEDs
        } else {
            strip.SetPixelColor(i, dimmedColor); // Apply heartbeat color
        }
    }
    applyCurrentLimitAndShow(neoBrightness);
}

// Helper Function: Scale Color by Brightness
RgbColor scaleColor(RgbColor color, float brightness) {
    byte r = color.R * brightness;
    byte g = color.G * brightness;
    byte b = color.B * brightness;
    return RgbColor(r, g, b);
}

void doAnimationMakuna() {
static unsigned long lastRun = 0;
static bool neoPaused = false;
static bool alarmWasActive = false;
bool forceImmediate = false;

  if (EEPROMsaving) return;
    
  if (alarmON) {
    alarmWasActive = true;
    neoPaused = false;
    alarmLight();
    return;
  }

  if (alarmWasActive) {
    alarmWasActive = false;
    neoPaused = false;
    forceImmediate = true;
  }
  
  if (!forceImmediate) {
    if ((prm.rgbEffect <=1) && ((millis()-lastRun)<1000)) return;  //fix color
    if ((millis()-lastRun)<max(FPS_MSEC,258-prm.rgbSpeed)) return;
  }
  lastRun = millis();

  if ((prm.rgbEffect == 0) || !displayON || !radarON) {   //switch RGB backlight OFF
    if (!neoPaused) {
      darkenNeopixels();
      neoPaused = true;
    }
    return;
  }

  neoPaused = false;
  
  colorStep = max(1,prm.rgbSpeed/5);
  neoBrightness = prm.rgbBrightness;
  if (autoBrightness) {
    neoBrightness = (neoBrightness * lx) / long(MAXIMUM_LUX);
    if (neoBrightness< c_MinBrightness) 
            neoBrightness = c_MinBrightness;
  }
  neoBrightness = min(neoBrightness,long(RGB_MAX_BRIGHTNESS));  //safety only
  strip.SetBrightness(neoBrightness);
  //DPRINTLN("  NeoBrightness:"); DPRINT(neoBrightness);
  
  if (prm.rgbEffect==1) fixColor();
  else if (prm.rgbEffect==2) rainbow(); //flow
  else if (prm.rgbEffect==3) rainbow2();  //stepper
  else if (prm.rgbEffect==4) effect1();  //color dimmer
  else if (prm.rgbEffect==5) effect2();  //color stepper
  else if (prm.rgbEffect==6) effect3(false,false);  //color stepflow table
  else if (prm.rgbEffect==7) effect3(true,false);  //color stepflow random
  else if (prm.rgbEffect==8) effect3(true,true);  //color stepflow each pixel random
  else if (prm.rgbEffect==9) effect4();  //color stepflow random all pixels
  else if (prm.rgbEffect==10) kitt();  //Knight Rider's KITT car
  else if (prm.rgbEffect==11) heartbeat();  //HeartBeat - Dr. Pricopi
  else fixColor();   //darken leds, if any error happens
}

#else
void setupNeopixel() {}
void doAnimationMakuna() {}
void darkenNeopixels() {}
#endif

