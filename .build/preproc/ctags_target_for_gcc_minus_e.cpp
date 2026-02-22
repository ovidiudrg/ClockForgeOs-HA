# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
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
# 26 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
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
# 46 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2

# 46 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
//#define byte uint8_t

// ClockForgeOS Configuration Files
# 50 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 51 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2

// Platform-specific includes


# 56 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2


// ============================= PLATFORM DETECTION =============================
# 76 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
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
# 161 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
//#define USE_MDNS

unsigned long timeserverErrors = 0; //timeserver refresh errors

int timerON=0; //for debugging
int timerOFF=0;
unsigned long intCounter = 0; //for testing only, interrupt counter

byte c_MinBrightness = 8 /*Neopixel leds minimum brightness*/; //minimum LED brightness
byte c_MaxBrightness = 255 /*Neopixel leds maximum brightness*/; //maximum LED brightness
//--------------------------------------------------------------------------------------------------







String usedPinsStr;
String driverSetupStr;
# 216 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
# 217 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 218 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 219 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 220 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 221 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2

# 223 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 224 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 225 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2

# 227 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2



  hw_timer_t * ESP32timer = 
# 230 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
                           __null
# 230 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
                               ;
  portMUX_TYPE timerMux = {.owner = 0xB33FFFFF,.count = 0} /**< Spinlock initializer */;


  const char ESPpinout[] = {"OOOOOO      OOOOOOOO OOO OOO    OOIII  I"}; //GPIO 0..5, 12..19, 21..23, 25..27, 32..33, Input:34..36, 39)  usable pins 




# 240 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
DNSServer dnsServer;
# 242 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 243 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 244 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2

# 246 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 247 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 248 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 249 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2
# 250 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2


# 253 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 2


extern void writeDisplay();
extern void writeDisplaySingle();
extern void setup_pins();
extern void clearTubes();
extern int maxDigits;
extern char tubeDriver[];

//Web handlers (forward declarations)
void handleConfigChanged(AsyncWebServerRequest *request);

void mqttReconfigure();

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
  return (strcmp("NixieClock" /* Web UI admin password (change this) */, "ChangeMeNow!") == 0);
}

static String generateWebAuthToken() {
  static const char hex[] = "0123456789abcdef";
  char out[33];
  for (int i = 0; i < 32; i++) {

      uint8_t v = (uint8_t)(esp_random() & 0x0F);



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

char webName[] = "IV-11 VFD Nixie Clock";
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


  Serial.println(line);


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
  
# 560 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
 __builtin_va_start(
# 560 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
 args
# 560 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
 ,
# 560 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
 fmt
# 560 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
 )
# 560 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
                    ;
  vsnprintf(buf, sizeof(buf), fmt, args);
  
# 562 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
 __builtin_va_end(
# 562 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
 args
# 562 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
 )
# 562 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
             ;
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
    
# 585 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
   __builtin_va_start(
# 585 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
   args
# 585 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
   ,
# 585 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
   fmt
# 585 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
   )
# 585 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
                      ;
    vsnprintf(buf, sizeof(buf), fmt, args);
    
# 587 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
   __builtin_va_end(
# 587 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
   args
# 587 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
   )
# 587 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
               ;
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
# 620 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
//______________ WiFi finding variables ____________________________________________
byte bestChn; //the recent found best wifi AP's channel No.
uint8_t bestBssid[6]; //the recent found best wifi AP's BSSID (mac address)
uint8_t oldBssid[6]; //the current wifi AP's BSSID (mac address)
int bestRSSI = -200; //the recent found best wifi AP's RSSI value
unsigned long lastWifiScan = 0; //last wifi scan timestamp in CPU msec millis()
boolean wifiOK = false;
char myIp[20]="";

//Display buffers

byte __attribute__((section(".dram1" "." "125"))) digit[12];
byte __attribute__((section(".dram1" "." "126"))) newDigit[12];
byte __attribute__((section(".dram1" "." "127"))) oldDigit[12];
boolean __attribute__((section(".dram1" "." "128"))) digitDP[12]; //actual value to put to display
boolean digitsOnly = true; //only 0..9 numbers are possible to display?
byte __attribute__((section(".dram1" "." "129"))) animMask[12]; //0 = no animation mask is used


// --- Soft blanking wrapper (Clock64 safe) ---
// Forward declaration (tubesPowerState is defined later in the file).
extern bool tubesPowerState;

void darkenNeopixels();
extern volatile uint16_t neoAppliedCurrentmA;
extern volatile uint8_t neoAppliedBrightness;


// When tubesPowerState is false, we temporarily blank digits before pushing them to the driver.
static inline void writeDisplaySingleGuarded() {
  auto rawWriteFn = &writeDisplaySingle; // keep original driver function


  // Track whether we've already forced LEDs off while the tubes are off,
  // to avoid calling darkenNeopixels() on every multiplex refresh.
  static bool ledsForcedOff = false;


  if (!tubesPowerState) {

    if (!ledsForcedOff) {
      darkenNeopixels();
      ledsForcedOff = true;
    }

    byte saved[12];
    boolean savedDP[12];
    memcpy(saved, digit, sizeof(saved));
    memcpy(savedDP, digitDP, sizeof(savedDP));
    memset(digit, 10, sizeof(digit)); // 10 = BLANK in this firmware
    memset(digitDP, 0, sizeof(digitDP));
    rawWriteFn();
    memcpy(digit, saved, sizeof(saved));
    memcpy(digitDP, savedDP, sizeof(savedDP));
  } else {

    ledsForcedOff = false; // tubes are on again; allow normal LED programs to run

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




//#define THERMOMETER_CLOCK     //it means, first digit is +- sign, last digit is C/F, 4 digit is for numbers



  boolean thermometerClock = false;


// 8266 internal pin registers
// https://github.com/esp8266/esp8266-wiki/wiki/gpio-registers
// example: https://github.com/mgo-tec/OLED_1351/blob/master/src/OLED_SSD1351.cpp
# 718 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
// ===================== EXTENDED DEBUG (Clock_64) =====================
// This block provides a stable, extensible serial debug snapshot (1 line / second)
// plus event logs (motion edge, display ON/OFF edge). Keep it in the project long-term.




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
  if (now - last < 1000UL) return;
  last = now;

  DebugOut.printf("[DBG] motion=%d sleep=%d wakeOnMotion=%d manualOff=%d enable=%d mqttRadar=%d radarAllowed=%d tubes=%d " "wake=%us left=%us wakeMs=%lu now=%lu lastMotion=%lu time=%s\n", (int)dbg.motion, (int)dbg.tubesSleep, (int)dbg.wakeOnMotionEnabled, (int)dbg.manualDisplayOff, (int)dbg.enableTimeDisplay, (int)dbg.mqttRadarON, (int)dbg.radarAllowed, (int)dbg.tubesPowerState, (unsigned)dbg.tubesWakeSeconds, (unsigned)dbg.wakeLeftSeconds, dbg.wakeMs, dbg.nowMs, dbg.lastMotionMs, dbgTimeSourceStr(dbg.timeSource))




                                                                        ;
}



// =================== END EXTENDED DEBUG ===================





bool colonBlinkState = false;
boolean radarON = true;
boolean mqttRadarON = true;
unsigned long radarLastOn = 0;
boolean makeFirmwareUpdate = false;
boolean makeCathodeProtect = false;
volatile boolean stopCathodeProtect = false;
int cathProtMin = 5;
unsigned long lastTimeUpdate = 0; //last time refresh from GPS or internet timeserver
boolean RTCexist = false;
boolean RTCisPCF8563 = false;
boolean GPSexist = false;
boolean BME280exist = false;
boolean BMP280exist = false;
boolean AHTX0exist = false;
boolean SHT21exist = false;
boolean BH1750exist = false;
boolean LDRexist = false;

byte useDallasTemp = 0; //number of Dallas temperature sensors: 0,1,2
byte useTemp = 0; //Total number of any temperature sensors: 0..6
byte usePress = 0; //Total number of pressure sensors
byte useHumid = 0; //Total number of humidity sensors
byte useLux = 0;
float temperature[6] = {0,0,0,0,0,0};
float humid[6] = {0,0,0,0,0,0};
float pressur[6] = {0,0,0,0,0,0};
int lx = 0; //Enviroment LUX value, set by Light Sensor
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
  boolean alarmEnable = false; //Yes or No
  byte alarmHour = 7; //Alarm time
  byte alarmMin = 0;
  byte alarmPeriod = 15; //Alarm length, sec
//RGB settings ____________________________________________________________________________________________
  byte rgbEffect = 1; //0=OFF, 1=FixColor
  byte rgbBrightness = 100; // 0..255
  unsigned int rgbFixColor = 150; //0..255
  byte rgbSpeed = 50; //0..255msec / step
  boolean rgbDir = false; //false = right, true = left
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
  int mqttBrokerRefresh = 10; //sec
  boolean mqttEnable = false;
  char firmwareServer[78];
//Tube settings  ______________________________________________________________________________________
  int tempRepeatMin = 1; //temperature and humidity display repaeat (min)
  int utc_offset = 1; //time zone offset
  bool enableDST = true; // Flag to enable DST (summer time...)
  bool set12_24 = true; // Flag indicating 12 vs 24 hour time (false = 12, true = 24);
  bool showZero = true; // Flag to indicate whether to show zero in the hour ten's place
  bool enableBlink = true; // Flag to indicate whether center colon should blink
  int interval = 15; // prm.interval in minutes, with 0 = off
  bool enableAutoShutoff = true; // Flag to enable/disable nighttime shut off
  byte dayHour = 8; // start of daytime
  byte dayMin = 0;
  byte nightHour = 22; // start of night time
  byte nightMin = 0;
  byte dayBright = 100; // display daytime brightness
  byte nightBright = 5; // display night brightness
  byte animMode = 7; //0=no anim,  if 1 or 2 is used, animation, when a digit changes
  byte dateMode = 2; // 0:dd/mm/yyyy 1:mm/dd/yyyy 2:yyyy/mm/dd
  boolean tempCF = false; //Temperature Celsius=false / Fahrenheit=true
  boolean enableTimeDisplay; //ENABLE_CLOCK_DISPLAY
  byte dateStart; //Date is displayed start..end
  byte dateEnd;
  byte tempStart; //Temperature display start..end
  byte tempEnd;
  byte humidStart; //Humidity% display start..end
  byte humidEnd;
  byte dateRepeatMin; //show date only every xxx minute. If zero, datum is never displayed!  
  boolean enableDoubleBlink; //both separator points are blinking (6 or 8 tubes VFD clock)
  boolean enableAutoDim = false; //Automatic dimming by luxmeter
  boolean enableRadar = false; //Radar sensor
  int radarTimeout = 5; //min
  uint16_t tubesWakeSeconds = 10; // Wake duration in seconds when motion is detected (Clock64)
   uint16_t maxLedmA = 350; // Max LED current budget (mA) for NeoPixel limiter (0=disabled)
  bool manualDisplayOff = false; // Manual Display OFF override (UI)
  bool wakeOnMotionEnabled = true; // Wake on motion enabled (false = display always ON)
  uint8_t touchShortAction = 0; // ESP32 touch GPIO33 short press action
  uint8_t touchDoubleAction = 0; // ESP32 touch GPIO33 double press action
  uint8_t touchLongAction = 0; // ESP32 touch GPIO33 long press action
  float corrT0 = 0;
  float corrT1 = 0;
  float corrH0 = 0;
  float corrH1 = 0;
//____________________________________________________________________________________________________  
    //Manual time fallback (used when no NTP/GPS/RTC available)
  bool manualTimeValid = false;
  uint32_t manualEpoch = 0; //local epoch seconds
int magic = 305 /*EEPROM version*/; //magic value, to check EEPROM version when starting the clock
  // --- New (v8): Manual tubes sleep mode (wake on motion) ---
  bool tubesSleep = false; //if true, tubes are normally OFF and wake on motion
} prm;



static const int SETTINGS_EEPROM_ADDR = EEPROM_addr + (int)sizeof(prm);
Settings settings;
volatile bool settingsDirty = false;
//-------------------------------------------------------------------------------------------------------

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, prm.NtpServer, 0, 7200000 /*2h, Refresh time in millisec   86400000 = 24h*/); // Refresh time in millisec
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
bool tubesPowerState = true; // current HV power state (ON/OFF)
unsigned long tubesLastMotionMs = 0; //if tubesSleep is enabled, tubes stay ON until this time (millis)

bool onboardLedState = false;
# 948 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
static inline void applyOnboardLedState() {






}

static inline void forceOnboardLedOffEarly() {





}

bool initProtectionTimer = false; // Set true at the top of the hour to synchronize protection timer with clock
  // Set true at the top of the hour to synchronize protection timer with clock
bool decimalpointON = false;
bool alarmON = false; //Alarm in progress
unsigned long alarmStarted = 0; //Start timestamp millis()

enum TouchAction : uint8_t {
  TOUCH_ACTION_NONE = 0,
  TOUCH_ACTION_ALARM_OFF = 1,
  TOUCH_ACTION_COLOR_CHANGE = 2,
  TOUCH_ACTION_DISPLAY_OFF = 3,
  TOUCH_ACTION_DISPLAY_TOGGLE = 4,
  TOUCH_ACTION_COLOR_PREV = 5
};






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

  const uint32_t freeHeap = heap_caps_get_free_size((1<<2) /*|< Memory must allow for 8/16/...-bit data accesses*/);
  if (freeHeap < heapMinFreeBytes) heapMinFreeBytes = freeHeap;

  if (freeHeap <= HEAP_WARN_BYTES) {
    if (!heapProtectionActive) {
      heapProtectionActive = true;
      DebugOut.println("[HEAP] Low memory protection enabled.");
    }

    if (uiDebugEnabled) {
      uiDebugEnabled = false;
      heapAutoDisabledUiDebug = true;
      DebugOut.println("[HEAP] UI debug stream disabled to reduce memory load.");
    }

    wifiScanResults = "";

    if ((freeHeap <= HEAP_CRITICAL_BYTES) || ((nowMs - lastHeapCleanupMs) > 30000UL)) {
      clearLogRing();
      closeTelnetClients();
      lastHeapCleanupMs = nowMs;
      DebugOut.println("[HEAP] Cleanup applied (log ring + telnet clients).");
    }
    return;
  }

  if (heapProtectionActive && freeHeap >= HEAP_RECOVER_BYTES) {
    heapProtectionActive = false;
    DebugOut.println("[HEAP] Memory recovered. Protection relaxed.");
    if (heapAutoDisabledUiDebug && settings.debugEnabled) {
      uiDebugEnabled = true;
      DebugOut.println("[HEAP] UI debug stream restored.");
    }
    heapAutoDisabledUiDebug = false;
  }
}



char pinTxt[sizeof(ESPpinout)-1][30 +1];

void regPin(byte p,const char * txt) { //register used pins

  DebugOut.print("- "); DebugOut.print(txt); DebugOut.print(": GPIO"); DebugOut.print(p);
  if (p == 255) { DebugOut.print("  (unused pin)"); DebugOut.println(" "); return; }
  if (p >= sizeof(ESPpinout)-1) {
    DebugOut.print("  ERROR: PIN# DOESN'T EXIST.");
    return;
  }
  else if (ESPpinout[p]==' ') {
    DebugOut.print("  ERROR: RESERVED PIN.");
  }
  else if (ESPpinout[p]=='I') {
    DebugOut.print("  Warning: input only pin");
  }
  DebugOut.println(" ");
  if (strlen(pinTxt[p])>0) {
    DebugOut.print("*** ERROR *** "); DebugOut.print(txt); DebugOut.print(" on PIN#"); DebugOut.print(p);
    DebugOut.print(" ALREADY DEFINED AS "); DebugOut.println(pinTxt[p]);
    strncpy(pinTxt[p],"Error:MULTI DEF",30);
    pinTxt[p][30] = '\0';
  }
  else {
    strncpy(pinTxt[p],txt,30);
    pinTxt[p][30] = '\0';
  }
}

void listPins() {
  DebugOut.println("_______________________");
  DebugOut.println("___ USED CLOCK PINS ___");

    usedPinsStr = String("ESP32 used pins:<br>");



  for (byte i=0;i<sizeof(ESPpinout)-1;i++) {
    if (strlen(pinTxt[i])>0) {
      usedPinsStr += String(i) + ": " + String(pinTxt[i]) + "<br>";
      DebugOut.print(i); DebugOut.print(": "); DebugOut.println(pinTxt[i]);
    }
  }
  DebugOut.println("_______________________");
  //DPRINT("MAX_PIN"); DPRINTLN(MAX_PIN);
}

void startTimer() { //ESP_INTR_FLAG_IRAM







  //  https://techtutorialsx.com/2017/10/07/esp32-arduino-timer-interrupts/
  ESP32timer = timerBegin(0, 15 /*multiplex timer prescaler, about 80Hz for 6 tubes*/, true); //set prescaler to 80 -> 1MHz signal, true = edge generated signal
  timerAttachInterrupt(ESP32timer, &writeDisplay, true);
  timerAlarmWrite(ESP32timer, 1000, true); //100millisec, no repeat
  timerAlarmEnable(ESP32timer);
  //DPRINTLN("Starting ESP32 timer...");



}

void stopTimer() {



}
# 1150 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void clearDigits() {
  memset(oldDigit, 10, sizeof(oldDigit));
  memset(digit, 10, sizeof(digit));
  memset(newDigit, 10, sizeof(newDigit));
  memset(digitDP, 0, sizeof(digitDP));
}

void Fdelay(unsigned long d) {
  unsigned long dStart = millis();
  bool neopixelsForcedOff = false;


  if (tubesPowerState) {
    doAnimationMakuna();
  } else {
    darkenNeopixels();
    neopixelsForcedOff = true;
  }



  while ((millis() - dStart) < d) {
    if (WiFi.getMode() == 2) dnsServer.processNextRequest();
    enableDisplay(2000);
    writeDisplay2();
    getLightSensor();

    if (tubesPowerState) {
      neopixelsForcedOff = false;
      doAnimationMakuna();
    } else {
      if (!neopixelsForcedOff) {
        darkenNeopixels();
        neopixelsForcedOff = true;
      }
    }



    if (tubesPowerState) {
      doAnimationPWM();
    }
    alarmSound();



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
  if (s == "true" || s == "1" || s == "on" || s == "yes") return true;
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

void disableDisplay() {
  // Deprecated in this project version (kept for compatibility).
  dState = true;
  EEPROMsaving = false;
}

boolean findBestWifi() {
  int n;
  int newRssi;
  boolean found = false;

  if (WiFi.status() == WL_CONNECTED) { //save wifi settings to prm
    WiFi.disconnect();
  }
  WiFi.scanNetworks(false);
  n = WiFi.scanComplete();
  DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>(("Scanning WiFi, found networks:"))))); DebugOut.println(n);
  if (n>=0){
    bestRSSI = -200;
    memset(bestBssid,0,sizeof(bestBssid));
    for (int i = 0; i < n; ++i){
      if (WiFi.SSID(i) == String(prm.wifiSsid)) {
        newRssi = WiFi.RSSI(i);
        DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>(("scan: RSSI"))))); DebugOut.print(newRssi);
        if (bestRSSI<newRssi) { //better rsssi found...
          bestChn = WiFi.channel(i);
          bestRSSI = newRssi;
          memcpy(bestBssid,WiFi.BSSID(i),sizeof(bestBssid));
          DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>(("  FOUND: Channel:"))))); DebugOut.print(bestChn);
          DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>(("  BSSID:"))))); DebugOut.println(WiFi.BSSIDstr(i));
          found = true;
          lastWifiScan = millis();
        }
        else {
          DebugOut.println(" ");
        }
      }
    }
    WiFi.scanDelete();
    } //end else
    else {
     // WiFi.scanNetworks(false);
    }
    return(found);
}

void wifiManager() {
# 1324 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
}

void startNewWifiMode() { // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
 if (((millis()-lastWifiScan)<90000) && (bestRSSI > -95) && (WiFi.status() != WL_CONNECTED)) {
  DebugOut.println(((reinterpret_cast<const __FlashStringHelper *>(("_____________ Connecting WiFi ___________________")))));
  if (strlen(prm.wifiSsid)==0) return;
  WiFi.disconnect();
  delay(200);
  WiFi.mode(WIFI_MODE_NULL);
  delay(200);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setAutoReconnect(true);

    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setHostname(webName);
    WiFi.setSleep(false);




  char tmp[20];
  sprintf(tmp,"%02X:%02X:%02X:%02X:%02X:%02X",bestBssid[0],bestBssid[1],bestBssid[2],bestBssid[3],bestBssid[4],bestBssid[5]);
  DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>(("Connecting to best WiFi AP:"))))); DebugOut.print(prm.wifiSsid); DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>(("  PSW:"))))); DebugOut.print(prm.wifiPsw);
  DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>((" BSSID:"))))); DebugOut.print(tmp); DebugOut.print(" Chn:"); DebugOut.println(bestChn);
  WiFi.begin(prm.wifiSsid, prm.wifiPsw, bestChn, bestBssid, true);

  Fdelay(500);
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    DebugOut.print('.');
    Fdelay(3000);
    if (counter++>3) {
      DebugOut.println("Connecting to WiFi failed.");
      return;
    }
  }
  ip = WiFi.localIP();
  sprintf(myIp,"%d.%d.%d.%d",ip[0],ip[1],ip[2],ip[3]);
  DebugOut.print("\nConnected to "); DebugOut.print(WiFi.SSID());
  DebugOut.print(". LocalIP:"); DebugOut.print(myIp);
  DebugOut.print(" BSSID:"); DebugOut.print(WiFi.BSSIDstr());
  DebugOut.print(" RSSI:"); DebugOut.print(WiFi.RSSI());
  DebugOut.print(" CHN:"); DebugOut.println(WiFi.channel());
  DebugOut.println("\n\nNetwork Configuration:");
  DebugOut.println("----------------------");
  DebugOut.print("         SSID: "); DebugOut.println(WiFi.SSID());
  DebugOut.print("  Wifi Status: "); DebugOut.println(WiFi.status());
  DebugOut.print("Wifi Strength: "); DebugOut.print(WiFi.RSSI()); DebugOut.println(" dBm");
  DebugOut.print("          MAC: "); DebugOut.println(WiFi.macAddress());
  DebugOut.print("           IP: "); DebugOut.println(myIp);
  DebugOut.print("       Subnet: "); DebugOut.println(WiFi.subnetMask());
  DebugOut.print("      Gateway: "); DebugOut.println(WiFi.gatewayIP());
  DebugOut.print("        DNS 1: "); DebugOut.println(WiFi.dnsIP(0));
  DebugOut.print("        DNS 2: "); DebugOut.println(WiFi.dnsIP(1));
  DebugOut.print("        DNS 3: "); DebugOut.println(WiFi.dnsIP(2));
  DebugOut.println("_____________________________________________________________________________");
  memcpy(oldBssid,WiFi.BSSID(),sizeof(oldBssid));
}
  DebugOut.println("\r\n");
}


void startWifiMode() {
  wifiRecoveryApMode = false;
  disableDisplay();
  DebugOut.println("Starting Clock in WiFi Mode!");
  if (strlen(prm.wifiSsid)==0) {
    DebugOut.println("WiFi SSID not defined!");
    wifiManager();
    return;
  }
  WiFi.disconnect(true);
  //WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_MODE_STA);
  delay(100);
  //Fdelay(1000);

    WiFi.setHostname(webName);
    WiFi.setSleep(false);




    esp_wifi_set_ps (WIFI_PS_NONE); //power saving disable!

  //if (WiFi.status() == WL_CONNECTED) return;

  DebugOut.print("\nConnecting to WiFi SSID:("); DebugOut.print(prm.wifiSsid); DebugOut.print(")  PSW:("); DebugOut.print(prm.wifiPsw); DebugOut.println(")");
  int counter = 0;
  if (strlen(prm.wifiPsw)!=0)
    WiFi.begin(prm.wifiSsid, prm.wifiPsw);
  else
    WiFi.begin(prm.wifiSsid);
  while (WiFi.status() != WL_CONNECTED) {
    DebugOut.print('.');
    //playTubes();
    //Fdelay(3000);
    delay(1000);
    if (counter++>10) {
      wifiManager();
      return;
    }
  }
  DebugOut.println(" ");
  ip = WiFi.localIP();
  WiFi.setAutoReconnect(true);
  enableDisplay(100);
}


boolean updateTimefromTimeserver() { //true, if successful
  static unsigned long lastTimeFailure = 0;
  boolean res = false;
  int count = 1;

  if (((millis()-lastTimeUpdate)<7200000 /*2h, Refresh time in millisec   86400000 = 24h*/) && (lastTimeUpdate!=0))
    return(res);

  if ((lastTimeFailure>0) && ((millis()-lastTimeFailure)<300000)) return(res); //wait for 5min to retry, if no success

  if (WiFi.status() == WL_CONNECTED) {
    while (true) {
      DebugOut.print("Connecting to timeserver: "); DebugOut.println(count);
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
        DebugOut.print("Timeserver date:"); DebugOut.print(year()); DebugOut.print("/"); DebugOut.print(month()); DebugOut.print("/"); DebugOut.print(day());
        DebugOut.print(" time:"); DebugOut.print(hour()); DebugOut.print(":"); DebugOut.print(minute()); DebugOut.print(":"); DebugOut.println(second());
        DebugOut.println("Clock refreshed from timeserver.");
        lastTimeFailure = 0;
      }
      count ++;
      if (res) break; //success!!!
      Fdelay(1000);
      if (count > 5) { //failure, but stop trying
        lastTimeFailure = millis();
        break;
      }
    } //end while
  }
  return (res);
}





void startMDNS() {
  static boolean mdnsStarted = false;

  if (mdnsStarted) return; //running and not changed
# 1501 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
}

void startStandaloneMode() {
  wifiRecoveryApMode = true;
  DebugOut.println("Starting Clock in Standalone Mode!");
  DebugOut.print("Clock's AP SSID:"); DebugOut.print(prm.ApSsid);
  DebugOut.print("   PSW:"); DebugOut.println(prm.ApPsw);
  DebugOut.println("IP: 192.168.4.1");
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  boolean nwState;
  if (strlen(prm.ApPsw)) {
    nwState = WiFi.softAP(prm.ApSsid, prm.ApPsw);
  }
  else {
    nwState = WiFi.softAP(prm.ApSsid);
  }
  Fdelay(2000); //info: https://github.com/espressif/arduino-esp32/issues/2025

  DebugOut.print("AP status:"); DebugOut.println(nwState ? "Ready" : "Failed!"); //channel
  DebugOut.print("Mac address:"); DebugOut.println(WiFi.softAPmacAddress());
  ip = WiFi.softAPIP();
  DebugOut.print("SOFT_AP IP:"); DebugOut.println(WiFi.softAPIP());

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());

  enableDisplay(0);
}

void doFirmwareUpdate(){
    if (WiFi.status() != WL_CONNECTED) {
      DebugOut.println("Wifi disconnected. FirmwareUpdate failed.");
      return;
    }

    DebugOut.println("Webserver stopped...");
    server.reset(); //Stop server
    delay(2000);
    disableDisplay();
    yield();
    String fname = String(prm.firmwareServer)+"/"+String("fw42v1" /*firmware name*/)+".bin";
    DebugOut.print("Update firmware: "); DebugOut.println(fname);
    t_httpUpdate_return ret,ret2;
    boolean succ = false;
    WiFiClient client;
    ESPhttpUpdate.rebootOnUpdate(false);

      ret = ESPhttpUpdate.update(fname); //ESP32 and old 8266 Core



    switch(ret) {
            case HTTP_UPDATE_FAILED:
                DebugOut.printf("HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                DebugOut.println(" ");
                break;

            case HTTP_UPDATE_NO_UPDATES:
                DebugOut.println("HTTP_UPDATE_NO_UPDATES");
                break;

            case HTTP_UPDATE_OK:
                DebugOut.println("HTTP_UPDATE_OK");
                succ = true;
                break;
    }

    fname = String(prm.firmwareServer)+"/"+String("fw42v1" /*firmware name*/)+".spiffs.bin";
    DebugOut.print("Update SPIFFS: "); DebugOut.println(fname);

      ret2 = ESPhttpUpdate.updateSpiffs(fname); //ESP32 or old 8266 Core



            switch(ret2) {
                case HTTP_UPDATE_FAILED:
                    DebugOut.printf("HTTP_UPDATE_FAILED Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
                    DebugOut.println(" ");
                    break;

                case HTTP_UPDATE_NO_UPDATES:
                    DebugOut.println("HTTP_UPDATE_NO_UPDATES");
                    break;

                case HTTP_UPDATE_OK:
                    DebugOut.println("HTTP_UPDATE_OK");
                    succ = true;
                    break;
            }

    if (succ) {
      DebugOut.println(" ");
      delay(1000);
      doReset();
    }
    DebugOut.println(" ");
    startServer(); //restart webserver
}

void doCathodeProtect() {
  unsigned long started = millis();
  byte num =0;
  int db = prm.dayBright; //save brightness values
  int nb = prm.nightBright;
  boolean ab = autoBrightness;
  byte onOff = 0;

  prm.dayBright=100;
  prm.nightBright = 100;
  autoBrightness = false;
  cathodeProtRunning = true;
  stopCathodeProtect = false;

  DebugOut.print("Cathode Protect is running for "); DebugOut.print(cathProtMin); DebugOut.println(" minutes.");
  memset(digitDP, 0, sizeof(digitDP));
  memset(animMask,0,sizeof(animMask));
  while (true) {
   for (int i=0;i<maxDigits;i++) {
    if (i%2) digit[i] = num;
    else digit[i] = 9-num;
    newDigit[i] = digit[i];
    digitDP[i] = (onOff%2 == i%2);
   }
   writeDisplaySingleGuarded();
   Fdelay(100);
   num++; if (num>9) num = 0;
   onOff++;
   if (stopCathodeProtect) {
     DebugOut.println("Cathode Protect manually stopped.");
     break;
   }
   if ((millis()-started)>uint32_t(cathProtMin)*60000l) break;
  } //end while

  prm.dayBright = db; //restore brightness values
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

  DebugOut.println("Starting Async Webserver...");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if(SPIFFS.exists("/index.html")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/index.html", "text/html");
      response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
      request->send(response);
    }
    else {
    request->send( 204, "text/html", "File Not Found" );
    DebugOut.println("/index.html not found");
    }
  });

  server.on("/jquery_351.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/jquery_351.js")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/jquery_351.js", "application/javascript");
      response->addHeader("Cache-Control", "max-age=31536000" /*maximum is: 31536000*/);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DebugOut.println("/jquery_351.js not found");
    }
  });

  server.on("/jquery_360.js", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/jquery_360.js")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/jquery_360.js", "application/javascript");
      response->addHeader("Cache-Control", "max-age=31536000" /*maximum is: 31536000*/);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DebugOut.println("/jquery_360.js not found");
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
      response->addHeader("Cache-Control", "max-age=31536000" /*maximum is: 31536000*/);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DebugOut.println("/page.js not found");
    }
  });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/favicon.ico")){
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/favicon.ico", "image/png");
      response->addHeader("Cache-Control", "max-age=31536000" /*maximum is: 31536000*/);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DebugOut.println("/favicon.ico not found");
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
# 1770 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
  });

//____________________________________________________________________________
  server.on("/site.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    disableDisplay();
    if (SPIFFS.exists("/site.css")) {
      AsyncWebServerResponse *response = request->beginResponse(SPIFFS, "/site.css", "text/css");
      response->addHeader("Cache-Control", "max-age=31536000" /*maximum is: 31536000*/);
      request->send(response);
    }
    else {
      request->send( 204, "text/html", "File Not Found" );
      DebugOut.println("/site.css not found");
    }
  });
//____________________________________________________________________________
  server.on("/reset", HTTP_POST, [] (AsyncWebServerRequest *request) {
    if (!requireWebAuth(request)) return;
    DebugOut.println("/reset:");

    request->send(200, "text/plain", "Reset: Restarting the Box!");
    delay(200);
    doReset();
  });
//____________________________________________________________________________
  server.on("/factoryreset", HTTP_POST, [] (AsyncWebServerRequest *request) {
    if (!requireWebAuth(request)) return;
    DebugOut.println("/factoryreset:");

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

} //end of procedure

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
    response->addHeader("Cache-Control", "max-age=31536000" /*maximum is: 31536000*/);
    request->send(response);
    return;
  }

  int params = request->params();
  for (int i = 0; i < params; i++) {

    AsyncWebParameter* p = request->getParam(i);
    if (p->isFile()) { //p->isPost() is also true
      Serial.printf("FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
    } else if (p->isPost()) {
      Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
    } else {
      Serial.printf("GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
    }

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

  DebugOut.println(message);
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
  if (!secureEquals(password, "NixieClock" /* Web UI admin password (change this) */)) {
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
    DebugOut.print(key); DebugOut.print(" = "); DebugOut.println(value);

    boolean paramFound = true;

    if (key == "utc_offset") {
      prm.utc_offset = value.toInt();
      if (old_utc_offset != prm.utc_offset) { //Change time zone
        setTime(now()+(prm.utc_offset-old_utc_offset)*3600);
        updateRTC();
      }
    }
    else if (key == "set12_24") {
      prm.set12_24 = (value == "true");
      DebugOut.print("set12_24:");
      DebugOut.println(prm.set12_24);
    }
    else if (key == "showZero") {
      prm.showZero = (value == "true");
      DebugOut.print("showZero:");
      DebugOut.println(prm.showZero);
    }
    else if (key == "enableBlink") {
      prm.enableBlink = (value == "true");
    }
    else if (key == "enableDST") {
      prm.enableDST = (value == "true");
      if (oldDST && !prm.enableDST) { //Switching off DST
        setTime(now()-3600);
        updateRTC();
      }
      if (!oldDST && prm.enableDST) { //Switching on DST
        setTime(now()+3600);
        updateRTC();
      }
    }
    else if (key == "interval") {
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
else if (key == "dayTimeHours") {
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
    else if (key == "animMode") {
      prm.animMode = value.toInt();
    }
    else if (key == "manualOverride") {
      boolean v = value == "false";
      if (v != displayON) {
        manualOverride = true;
        displayON = v;
      }
    }
    else if (key == "wifiMode") {
      prm.wifiMode = (value == "true");
    }
    else if (key == "alarmEnable") {
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
    else if (key == "alarmTimeHours") {
      prm.alarmHour = value.toInt();
      settingsDirty = true;
      requestSaveEEPROM();
    }
    else if (key == "alarmTimeMinutes") {
      prm.alarmMin = value.toInt();
      settingsDirty = true;
      requestSaveEEPROM();
    }
    else if (key == "alarmPeriod") {
      prm.alarmPeriod = value.toInt();
      settingsDirty = true;
      requestSaveEEPROM();
    }

    //RGB LED values
    else if (key == "rgbEffect") {
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
      settings.rgbFixR = (uint8_t)((value.toInt())<(0)?(0):((value.toInt())>(255)?(255):(value.toInt())));
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
      settings.rgbFixG = (uint8_t)((value.toInt())<(0)?(0):((value.toInt())>(255)?(255):(value.toInt())));
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
      settings.rgbFixB = (uint8_t)((value.toInt())<(0)?(0):((value.toInt())>(255)?(255):(value.toInt())));
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
else if (key == "rgbSpeed") {
      prm.rgbSpeed = value.toInt();
    }
    else if (key == "rgbDir") {
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
      for (int i=0;i<(int)strlen(prm.ApSsid);i++) { //repair bad chars in AP SSID
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
    else if (key == "enableDoubleBlink") {
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
        lx = 100;
      }
    }
    else if (key == "enableRadar") {
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
        settings.uiWidth = (uint16_t)((value.toInt())<(400)?(400):((value.toInt())>(1600)?(1600):(value.toInt())));
      }
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Width set to %s", (settings.uiWidth == 0xFFFF) ? "100%" : String(settings.uiWidth).c_str());
    }
    else if (key == "uiBgColor") {
      // Parse hex color string (e.g., "#RRGGBB")
      uint32_t col = strtol(value.c_str() + (value[0] == '#' ? 1 : 0), 
# 2479 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
                                                                      __null
# 2479 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
                                                                          , 16);
      settings.uiBgR = (uint8_t)((col >> 16) & 0xFF);
      settings.uiBgG = (uint8_t)((col >> 8) & 0xFF);
      settings.uiBgB = (uint8_t)(col & 0xFF);
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Background set to #%02X%02X%02X", settings.uiBgR, settings.uiBgG, settings.uiBgB);
    }
    else if (key == "uiPanelColor") {
      // Parse hex color string (e.g., "#RRGGBB")
      uint32_t col = strtol(value.c_str() + (value[0] == '#' ? 1 : 0), 
# 2489 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
                                                                      __null
# 2489 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
                                                                          , 16);
      settings.uiPanelR = (uint8_t)((col >> 16) & 0xFF);
      settings.uiPanelG = (uint8_t)((col >> 8) & 0xFF);
      settings.uiPanelB = (uint8_t)(col & 0xFF);
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Panel set to #%02X%02X%02X", settings.uiPanelR, settings.uiPanelG, settings.uiPanelB);
    }
    else if (key == "uiAccentColor") {
      // Parse hex color string (e.g., "#RRGGBB")
      uint32_t col = strtol(value.c_str() + (value[0] == '#' ? 1 : 0), 
# 2499 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
                                                                      __null
# 2499 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
                                                                          , 16);
      settings.uiAccentR = (uint8_t)((col >> 16) & 0xFF);
      settings.uiAccentG = (uint8_t)((col >> 8) & 0xFF);
      settings.uiAccentB = (uint8_t)(col & 0xFF);
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Accent set to #%02X%02X%02X", settings.uiAccentR, settings.uiAccentG, settings.uiAccentB);
    }
    else if (key == "uiTextColor") {
      // Parse hex color string (e.g., "#RRGGBB")
      uint32_t col = strtol(value.c_str() + (value[0] == '#' ? 1 : 0), 
# 2509 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino" 3 4
                                                                      __null
# 2509 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
                                                                          , 16);
      settings.uiTextR = (uint8_t)((col >> 16) & 0xFF);
      settings.uiTextG = (uint8_t)((col >> 8) & 0xFF);
      settings.uiTextB = (uint8_t)(col & 0xFF);
      settingsMarkDirty();
      requestSaveEEPROM();
      debugLogf("[UI] Text set to #%02X%02X%02X", settings.uiTextR, settings.uiTextG, settings.uiTextB);
    }
    else {
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

      if (mqttNeedsReconfigure) {
        mqttReconfigure();
      }

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
    tm.Year = ((yy) - 1970);
    tm.Month = (uint8_t)mm;
    tm.Day = (uint8_t)dd;
    tm.Hour = (uint8_t)hh;
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
  doc["FW"] = "fw42v1" /*firmware name*/;
  doc["tubeDriver"] = tubeDriver;
  doc["maxDigits"] = maxDigits;
  doc["maxBrightness"] = 100;

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
  char buf[20]; //conversion buffer

  DebugOut.println("Sending configuration to web client.");

  //Global data
  doc["version"] = webName;
  doc["FW"] = "fw42v1" /*firmware name*/;
  doc["tubeDriver"] = tubeDriver;
  doc["maxDigits"] = maxDigits; //number of digits (tubes)
  doc["maxBrightness"] = 100; //Maximum tube brightness usually 10, sometimes 12

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
  doc["enableDST"] = prm.enableDST; // Flag to enable DST (summer time...)
  doc["set12_24"] = prm.set12_24; // Flag indicating 12 vs 24 hour time (false = 12, true = 24);
  doc["showZero"] = prm.showZero; // Flag to indicate whether to show zero in the hour ten's place
  doc["enableBlink"] = prm.enableBlink; // Flag to indicate whether center colon should blink
  doc["interval"] = prm.interval; // doc["interval in minutes, with 0 = off

  //Day/Night dimmer parameters
  doc["enableAutoShutoff"] = prm.enableAutoShutoff; // Flag to enable/disable nighttime shut off
  doc["tubesSleep"] = prm.tubesSleep; // Manual tubes sleep mode (wake on motion)
  doc["tubesPower"] = tubesPowerState; // Status: tubes currently ON/OFF
  doc["displayPower"] = !prm.manualDisplayOff; // Manual display power (true=ON)
  doc["manualDisplayOff"] = prm.manualDisplayOff;
  doc["wakeOnMotionEnabled"] = prm.wakeOnMotionEnabled;
  doc["tubesWakeSeconds"] = prm.tubesWakeSeconds; // Motion wake duration (seconds)
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
  doc["animMode"] = prm.animMode; //Tube animation
  doc["manualOverride"] = manualOverride;
//Alarm values
  doc["alarmEnable"] = prm.alarmEnable; //1 = ON, 0 = OFF
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

    doc["rgbEffect"] = prm.rgbEffect; // if 255, no RGB exist!




  doc["rgbBrightness"] = prm.rgbBrightness; // c_MinBrightness..255
  doc["maxLedmA"] = prm.maxLedmA; // 0 disables limiter
  doc["rgbFixR"] = settings.rgbFixR; // Fixed color R component (persisted)
  doc["rgbFixG"] = settings.rgbFixG; // Fixed color G component (persisted)
  doc["rgbFixB"] = settings.rgbFixB; // Fixed color B component (persisted)
  doc["rgbSpeed"] = prm.rgbSpeed; // 1..255
  doc["rgbDir"] = prm.rgbDir; // 0 = right direction, 1 = left direction
  doc["rgbMinBrightness"] = c_MinBrightness; //minimum brightness for range check!!

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

    doc["mqttBrokerRefresh"] = prm.mqttBrokerRefresh;



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

    if (prm.radarTimeout<1) prm.radarTimeout = 1;
    doc["radarTimeout"] = prm.radarTimeout;



    doc["tubesWakeSeconds"] = prm.tubesWakeSeconds; // Motion wake duration (seconds)
  doc["displayPower"] = !prm.manualDisplayOff; // Manual display power (true=ON)
doc["corrT0"] = prm.corrT0;
  doc["corrT1"] = prm.corrT1;
  doc["corrH0"] = prm.corrH0;
  doc["corrH1"] = prm.corrH1;
  doc["cathProtMin"] = 3; //default value for cathProtMin slider
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

  DebugOut.println("SendClockDetails");
  dat = String("<br><br><strong>FirmwareID:") + "fw42v1" /*firmware name*/ + "<br>";
  dat += String("  Tube driver:") + tubeDriver + "<br>";
  dat += String("  Mac:") + String(WiFi.macAddress()) + "</strong><br>";
  dat += String("MAXBRIGHTNESS:") + String(100) + "<br>";
  if (RTCexist) dat += String("RTC exist: YES <br>");
  if (validTempSensors > 0) dat += String("Temperature sensors:") + validTempSensors + "<br>";
  if (validHumidSensors > 0) dat += String("Humidity sensors:") + validHumidSensors + "<br>";
  if (usePress>0) dat += String("Pressure sensors:") + usePress + "<br>";
  dat += usedPinsStr + "<br>";
  dat += driverSetupStr + "<br>";
  DebugOut.println(dat);
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

  DebugOut.println("SendSystemInfo");

  // ESP32 Status
  doc["cpuTemp"] = (int)temperatureRead(); // Internal temperature sensor, returns Celsius
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
  uint32_t freeHeap = heap_caps_get_free_size((1<<2) /*|< Memory must allow for 8/16/...-bit data accesses*/);
  uint32_t totalHeap = ESP.getHeapSize();
  uint32_t largestHeapBlock = heap_caps_get_largest_free_block((1<<2) /*|< Memory must allow for 8/16/...-bit data accesses*/);
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





    doc["totalBytes"] = SPIFFS.totalBytes();
    doc["usedBytes"] = SPIFFS.usedBytes();
    doc["freeBytes"] = SPIFFS.totalBytes() - SPIFFS.usedBytes();


  // EEPROM Info
  doc["eepromSize"] = (sizeof(prm) + sizeof(Settings));
  doc["eepromUsed"] = (sizeof(prm) + sizeof(Settings)); // Total used (both prm + settings structs)

  // Firmware Info
  doc["osVersion"] = "ClockForgeOS 1.0";
  doc["firmwareID"] = "fw42v1" /*firmware name*/;
  doc["tubeDriver"] = tubeDriver;
  doc["maxDigits"] = maxDigits;
  doc["maxBrightness"] = 100;

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


  doc["virtualSensors"] = virtualSensors.length() ? virtualSensors : "None";

  // Used pins (as array of {num, label})
  JsonArray pinsArr = doc.createNestedArray("usedPins");
  for (byte i = 0; i < sizeof(ESPpinout)-1; i++) {
    if (strlen(pinTxt[i]) > 0) {
      JsonObject p = pinsArr.createNestedObject();
      p["num"] = i;
      p["label"] = String(pinTxt[i]);
    }
  }

  // HV5122 pin settings (as array of arrays)
# 3066 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
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
    DebugOut.println("Returning cached WiFi scan results");
    debugLogf("[WiFiScan] returning cached results");
    request->send(200, "application/json", r);
    return;
  }

  // If a scan is in progress, poll completion without disrupting WiFi link.
  if (wifiScanInProgress) {
    int n = WiFi.scanComplete();
    if (n == (-1)) {
      request->send(200, "application/json", String("{\"status\":\"scanning\"}"));
      return;
    }
    if (n < 0) {
      DebugOut.println("WiFi scan failed");
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
  wifi_mode_t scanMode = WiFi.getMode();
  if (scanMode == WIFI_MODE_NULL) {
    WiFi.mode(WIFI_MODE_STA);
  } else if (scanMode == WIFI_MODE_AP) {
    WiFi.mode(WIFI_MODE_APSTA);
    delay(20);
  }

  DebugOut.println("Starting async WiFi scan (non-disruptive)");
  debugLogf("[WiFiScan] start async scan (mode=%d)", (int)scanMode);
  int startResult = WiFi.scanNetworks(true, false);
  if (startResult == (-2)) {
    DebugOut.println("Failed to start WiFi scan");
    debugLogf("[WiFiScan] failed to start async scan");
    request->send(200, "application/json", String("[]"));
    return;
  }

  wifiScanInProgress = true;
  request->send(200, "application/json", String("{\"status\":\"scanning\"}"));
}

static inline void setWifiModeForSwitchKeepAp() {
  wifi_mode_t wm = WiFi.getMode();
  if ((wm == WIFI_MODE_AP) || (wm == WIFI_MODE_APSTA)) {
    WiFi.mode(WIFI_MODE_APSTA);
  } else {
    WiFi.mode(WIFI_MODE_STA);
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
    DebugOut.printf("WiFi.begin(%s, %s)\n", ssid.c_str(), psw.c_str());
    WiFi.begin(ssid.c_str(), psw.c_str());
  } else {
    DebugOut.printf("WiFi.begin(%s)\n", ssid.c_str());
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
  DebugOut.print("ConnectWifi: "); DebugOut.println(ssid);
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

  DebugOut.print("ConnectWifiPost: "); DebugOut.println(ssid);
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
  char buf[20]; //conversion buffer
  //Actual time and environment data
  sprintf(buf, "%4d.%02d.%02d", year(), month(), day());
  doc["currentDate"] = buf;
  sprintf(buf, "%02d:%02d", hour(), minute());
  doc["currentTime"] = buf;
  //DPRINT("useTemp:"); DPRINT(useTemp); DPRINT("useHumid:"); DPRINT(useHumid);
  if (useTemp > 0) {
    doc["temperature1"] = temperature[0] + prm.corrT0;
    doc["temperature"] = temperature[0] + prm.corrT0; //for compatibility with the old web page
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
    doc["humidity"] = humid[0] + prm.corrH0; //for compatibility with the old web page
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


doc["ledCurrentmA"] = neoAppliedCurrentmA;
doc["ledAppliedBrightness"] = neoAppliedBrightness;




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



  forceOnboardLedOffEarly();
  delay(1000);
  WiFi.mode(WIFI_MODE_NULL);
  EEPROM.begin((sizeof(prm) + sizeof(Settings)));
  memset(pinTxt,0,sizeof(pinTxt));
  DebugOut.begin(115200); DebugOut.println(" ");
  DebugOut.println(((reinterpret_cast<const __FlashStringHelper *>(("=================================================")))));
  DebugOut.print("Starting "); DebugOut.println(webName);
  DebugOut.print("Firmware code:"); DebugOut.print("fw42v1" /*firmware name*/); DebugOut.print("   Tube driver:"); DebugOut.println(tubeDriver);
  DebugOut.print("MAXBRIGHTNESS:"); DebugOut.println(100);

  auto resetReasonToText = [](esp_reset_reason_t rr) -> const char* {
    switch (rr) {
      case ESP_RST_POWERON: return "POWERON";
      case ESP_RST_EXT: return "EXTERNAL";
      case ESP_RST_SW: return "SOFTWARE";
      case ESP_RST_PANIC: return "PANIC";
      case ESP_RST_INT_WDT: return "INT_WDT";
      case ESP_RST_TASK_WDT: return "TASK_WDT";
      case ESP_RST_WDT: return "WDT";
      case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
      case ESP_RST_BROWNOUT: return "BROWNOUT";
      case ESP_RST_SDIO: return "SDIO";
      default: return "UNKNOWN";
    }
  };
  esp_reset_reason_t rr = esp_reset_reason();
  DebugOut.print("Reset reason: ");
  DebugOut.print(resetReasonToText(rr));
  DebugOut.print(" (");
  DebugOut.print((int)rr);
  DebugOut.println(")");

  DebugOut.println(((reinterpret_cast<const __FlashStringHelper *>(("=================================================")))));
  clearDigits();
  onboardLedState = false;
  applyOnboardLedState();
  setup_pins();

    pinMode(2 /*Alarm buzzer pin*/, 0x03); regPin(2 /*Alarm buzzer pin*/,"ALARMSPEAKER_PIN");
    digitalWrite(2 /*Alarm buzzer pin*/,!0x1 /*How to switch ON alarm buzzer*/);
    DebugOut.print("  - ON state:");
    if (0x1 /*How to switch ON alarm buzzer*/ == 0x1) {DebugOut.println("HIGH"); }
    else {DebugOut.println("LOW"); }


    pinMode(0 /*Alarm switch off button pin*/, 0x05); regPin(0 /*Alarm switch off button pin*/,"ALARMBUTTON_PIN");
# 3467 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
    regPin(33,"TOUCH_BUTTON_PIN");





    pinMode(34, 0x01); regPin(34,"RADAR_PIN");
    DebugOut.print("  - RADAR Timeout:"); DebugOut.println(prm.radarTimeout);


    pinMode(27, 0x03); regPin(27,"TUBE_POWER_PIN");
    digitalWrite(27,0x1);
    DebugOut.print("  - ON state:");
    if (0x1 == 0x1) {DebugOut.println("HIGH"); }
    else {DebugOut.println("LOW"); }
# 3524 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
    setupI2Csensors();

  setupGestureSensor();


    setupRTC();






  decimalpointON = false;
  DebugOut.print("Number of digits:"); DebugOut.println(maxDigits);
# 3549 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
  loadEEPROM();
  if (prm.magic != 305 /*EEPROM version*/) {
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
    DebugOut.println("Manual time loaded from EEPROM.");
  }

  // NTPClient is constructed at global scope, so at that time prm.NtpServer may still be empty
  // (EEPROM not loaded yet). Ensure we set the pool server name now, after EEPROM/factory defaults.
  if (strlen(prm.NtpServer) == 0) {
    strncpy(prm.NtpServer, "pool.ntp.org", sizeof(prm.NtpServer));
    prm.NtpServer[sizeof(prm.NtpServer) - 1] = '\0';
  }
  timeClient.setPoolServerName(prm.NtpServer);
  timeClient.setUpdateInterval(7200000 /*2h, Refresh time in millisec   86400000 = 24h*/);

  setupNeopixel();
  listPins();
  writeAlarmPin(0x1 /*How to switch ON alarm buzzer*/); writeAlarmPin(!0x1 /*How to switch ON alarm buzzer*/);
  getDHTemp(); //get the first DHT temp+humid measure

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
    DebugOut.println("An Error has occurred while mounting SPIFFS");
  }
  else {

      DebugOut.print("SPIFFS Total bytes:    "); DebugOut.println(SPIFFS.totalBytes());
      DebugOut.print("SPIFFS Used bytes:     "); DebugOut.println(SPIFFS.usedBytes());

    DebugOut.println("SPIFFS started.");
    if(!SPIFFS.exists("/index.html")) {DebugOut.println("/index.html not found!");}
    if(!SPIFFS.exists("/site.css")) {DebugOut.println("/site.css not found!");}
    if(!SPIFFS.exists("/page.js")) {DebugOut.println("/page.js not found!");}
    if(!SPIFFS.exists("/jquery_360.js")) {DebugOut.println("/jquery_360.js not found!");}
  }
  memset(digit,10,sizeof(digit)); //clear display
  memset(newDigit,10,sizeof(newDigit));
  memset(oldDigit,10,sizeof(oldDigit));

  wifiConnectRunning = true;
  writeDisplay2();
  if (bootNeedsInitialSetup) {
    DebugOut.println("Initial setup mode: starting standalone AP for configuration.");
    startStandaloneMode();
  }
  else if (prm.wifiMode) {
    //findBestWifi();
    startWifiMode();
# 3660 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
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


    setupMqtt();


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

  if (WiFi.status() == WL_CONNECTED) { //check wifi connection
    refreshed = updateTimefromTimeserver(); //update time from wifi
  }
  if (GPSexist) { //update time from GPS, if exist
    refreshed2 = getGPS();
  }

  if (RTCexist) { //update time from RTC, if exist
    if (refreshed || refreshed2)
      updateRTC(); //update RTC, if needed
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
    requestDallasTemp(false); //start measure
    getTemp(); //get result, if ready
  }
  getDHTemp();
  getI2Csensors(); //check all existing I2C sensors






  if (now() != prevTime) { // Update time every second
    prevTime = now();
    evalShutoffTime(); // Check whether display should be turned off (Auto shutoff mode)
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
    if (maxDigits >= 8) displayTime8();
    else if (maxDigits == 6) displayTime6();
    else displayTime4();
    //   if (COLON_PIN>=0) digitalWrite(COLON_PIN,colonBlinkState);  // Blink colon pin






    changeDigit();
    printDigits(0);

    if (prm.interval > 0) { // protection is enabled
      // At the first top of the hour, initialize protection logic timer
      if (!initProtectionTimer && (minute() == 0)) {
        protectTimer = 0; // Ensures protection logic will be immediately triggered
        initProtectionTimer = true;
      }
      if ((now() - protectTimer) >= 60 * prm.interval) {
        protectTimer = now();
        // The current time can drift slightly relative to the protectTimer when NIST time is updated
        // Need to make a small adjustment to the timer to ensure it is triggered at the minute change
        protectTimer -= ((second() + 30) % 60 - 30);
        if (displayON && (millis() > 50000)) newCathodeProtect(maxDigits*1500,random(3)-1); //dont play in the first 50sec
      }
    }
  }
}

void loadEEPROM() {
  disableDisplay();
  DebugOut.print("Loading setting from EEPROM.  Size:"); DebugOut.print((sizeof(prm) + sizeof(Settings)));

  EEPROM.get(EEPROM_addr, prm);
  sanitizeAsciiCString(prm.mqttBrokerAddr, sizeof(prm.mqttBrokerAddr));
  sanitizeAsciiCString(prm.mqttBrokerUser, sizeof(prm.mqttBrokerUser));
  sanitizeAsciiCString(prm.mqttBrokerPsw, sizeof(prm.mqttBrokerPsw));
  // Load separate user settings struct
  settingsLoadFromEEPROM();
  // Backward-compatible sanity for newly added fields (older EEPROM will have 0xFFFF).
  if (prm.maxLedmA == 0xFFFF || prm.maxLedmA > 2000) prm.maxLedmA = 350;
  // 0 disables the limiter intentionally; keep 0 as-is.
  DebugOut.print("  version:"); DebugOut.println(prm.magic);
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
  DebugOut.println("Settings saved to EEPROM!");
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
  settings.uiWidth = 800; // 800px default
  settings.uiBgR = 3;
  settings.uiBgG = 2;
  settings.uiBgB = 2;
  settings.uiPanelR = 15;
  settings.uiPanelG = 6;
  settings.uiPanelB = 6;
  settings.uiAccentR = 0xD9; // #D96025 (orange accent)
  settings.uiAccentG = 0x60;
  settings.uiAccentB = 0x25;
  settings.uiTextR = 0xDD; // #DDDDDD (light text)
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
  eepromSavePending = false; // Clear any pending general save
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
  DebugOut.println("Factory Reset!!!");
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



    strcpy(prm.wifiSsid,"");




    strcpy(prm.wifiPsw,"");

  for (int i=0;i<(int)strlen(prm.ApSsid);i++) { //repair bad chars in AP SSID
    if ((prm.ApSsid[i]<32) || (prm.ApSsid[i]>126)) prm.ApSsid[i]='_';
  }
  strncpy(prm.ApSsid, "NixieClock", sizeof(prm.ApSsid) - 1);
  prm.ApSsid[sizeof(prm.ApSsid) - 1] = '\0';
  strncpy(prm.ApPsw, "18273645", sizeof(prm.ApPsw) - 1);
  prm.ApPsw[sizeof(prm.ApPsw) - 1] = '\0';
  strncpy(prm.NtpServer,"pool.ntp.org",sizeof(prm.NtpServer));
  strcpy(prm.mqttBrokerAddr,"10.10.0.202");
  strcpy(prm.mqttBrokerUser,"mqtt");
  strcpy(prm.mqttBrokerPsw,"mqtt");
  strncpy(prm.firmwareServer,"",sizeof(prm.firmwareServer));
  prm.mqttEnable = false;
  prm.mqttBrokerRefresh = 10; //sec
  prm.utc_offset = 1;
  prm.enableDST = true; // Flag to enable DST (summer time...)
  prm.set12_24 = true; // Flag indicating 12 vs 24 hour time (false = 12, true = 24);
  prm.showZero = true; // Flag to indicate whether to show zero in the hour ten's place
  prm.enableBlink = true; // Flag to indicate whether center colon should blink
  prm.interval = 15; // prm.interval in minutes, with 0 = off
  prm.enableAutoShutoff = true; // Flag to enable/disable nighttime shut off
  prm.dayHour = 7;
  prm.dayMin = 0;
  prm.nightHour = 20;
  prm.nightMin = 0;
  prm.dayBright = (byte)(((int)100 * 30 + 50) / 100);
  if (prm.dayBright < 1) prm.dayBright = 1;
  prm.nightBright = 3;
  prm.animMode = 7;
  prm.dateMode = 0; // 0:dd/mm/yyyy 1:mm/dd/yyyy 2:yyyy/mm/dd
  prm.tempCF = false; //Temperature Celsius / Fahrenheit
  prm.enableTimeDisplay = true /*false, if no clock display is needed (for example: thermometer + humidity only)*/;
  prm.dateStart = 35; //Date display window: 35..40 sec
  prm.dateEnd = 40;
  prm.tempStart = 40; //Temperature display window: 40..45 sec
  prm.tempEnd = 45;
  prm.humidStart = 45; //Humidity display window: 45..50 sec
  prm.humidEnd = 50;
  prm.dateRepeatMin = 1; //show date only every xxx minute. If zero, datum is never displayed!  
  prm.tempRepeatMin = 1;
  prm.enableDoubleBlink = true; //both separator points are blinking (6 or 8 tubes VFD clock)
  prm.enableAutoDim = true; //Automatic dimming by luxmeter
  prm.enableRadar = false; //Radar sensor
  prm.radarTimeout = 5; //min
  prm.tubesSleep = false; // Motion-based tubes sleep
  prm.tubesWakeSeconds = 10; // Wake duration (sec)
  prm.manualDisplayOff = false; // Manual display OFF
  prm.wakeOnMotionEnabled = true; // Wake on motion enabled by default
  prm.touchShortAction = TOUCH_ACTION_ALARM_OFF;
  prm.touchDoubleAction = TOUCH_ACTION_COLOR_CHANGE;
  prm.touchLongAction = TOUCH_ACTION_DISPLAY_TOGGLE;
  prm.corrT0 = 0;
  prm.corrT1 = 0;
  prm.corrH0 = 0;
  prm.corrH1 = 0;
  prm.manualTimeValid = false;
  prm.manualEpoch = 0;
  prm.magic = 305 /*EEPROM version*/; //magic value to check the EEPROM version
  settingsSetDefaults();
  // Factory reset must persist immediately before reboot
  settingsDirty = true;
  saveEEPROM();
  eepromSavePending = false;
  calcTime();
}


void newCathodeProtect(unsigned long t,int dir) { //t = time in msec, dir = direction -1,0,1    (0=random) 
  byte tmp[10];
  boolean tmpDP[10];
  unsigned long started = millis();
  boolean finish, stopThis;
  byte fin[10];
  byte nextStoppedDigit;
  int sum = 0;

  cathodeProtRunning = true;
  stopCathodeProtect = false;
  DebugOut.print("Cathode Protect running! dir:"); DebugOut.println(dir);
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

      if (finish && stopThis) { //this digit stops
        fin[i] = 1;
        digitDP[i] = tmpDP[i]; //restore original DP value
        digit[i] = tmp[i];
        t +=1000;
        if (dir>0) { //find next stop digit
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
          if (fin[nextStoppedDigit] ==0) break; //found a running digit
          }
        }
      }
      else if (fin[i]==0) { //get next number
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
    for (int i=0;i<maxDigits;i++) sum += fin[i];
    //DPRINT("Sum:"); DPRINTLN(sum);
    if (sum >= maxDigits) //all digits ready
      break;
    if (stopCathodeProtect) {
      DebugOut.println("Cathode Protect manually stopped.");
      break;
    }
    if ((millis()-started) >= (t+5000)) {
      DebugOut.println("Safety exit");
      break;
    }
  } //end while

  memcpy(oldDigit, digit, sizeof(oldDigit));
  lastCathodeProt = minute();
  cathodeProtRunning = false;
  stopCathodeProtect = false;
}

void cathodeProtect() { //original version
  int hour12_24 = prm.set12_24 ? (byte)hour() : (byte)hourFormat12();
  byte hourShow = (byte)hour12_24;
  byte minShow = (byte)minute();
  byte secShow = (byte)second();
  byte dh1 = (hourShow / 10), dh2 = (hourShow % 10);
  byte dm1 = (minShow / 10), dm2 = (minShow % 10);
  byte ds1 = (secShow / 10), ds2 = (secShow % 10);

  DebugOut.println("Cathode Protect running!");
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
# 4219 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayTemp(byte ptr) {
  float t = temperature[ptr];
  if (ptr==0) t += prm.corrT0;
  if (ptr==1) t += prm.corrT1;
  if (prm.tempCF) {
    t = round1((temperature[ptr] * 9/5)+32);
  }
  int digPtr = maxDigits-1;



  for (int i = 0; i < maxDigits; i++) {
    digitDP[i] = false;
    newDigit[i] = 10;
  }





  t = abs(t);
  newDigit[digPtr] = t / 10;
  if (newDigit[digPtr] == 0) newDigit[digPtr] = 10; //BLANK!!!
  newDigit[--digPtr] = int(t) % 10;
  digitDP[digPtr] = true;
  newDigit[--digPtr] = int(t * 10) % 10;
  if ((maxDigits > 5) && (16 >= 0)) {
    digPtr -=1;
    newDigit[digPtr] = 16; //grad
  }
  if (15 >= 0) {
    if (prm.tempCF) newDigit[--digPtr] = 15 +5; //  'F'
    else newDigit[--digPtr] = 15; //  'C'
  }

  if (prm.animMode == 0) memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
  colonBlinkState = true;
  decimalpointON = true;
}
# 4285 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
void displayHumid(byte ptr) {
  int digPtr = maxDigits-1;



  for (int i = 0; i < maxDigits; i++) {
    digitDP[i] = false;
    newDigit[i] = 10;
  }
  float h = humid[ptr];
  if (ptr==0) h += prm.corrH0;
  if (ptr==1) h += prm.corrH1;



  newDigit[digPtr] = int(h) / 10;
  if (newDigit[digPtr] == 0) newDigit[digPtr] = 10; //BLANK if zero!!!
  newDigit[--digPtr] = int(h) % 10;
  digitDP[digPtr] = true;
  newDigit[--digPtr] = int(h * 10) % 10;
  if (maxDigits > 5) {
    if (digitsOnly) {
      newDigit[--digPtr] = 10; //empty character
      newDigit[--digPtr] = 17; //  "%"
    }
    else {
      newDigit[--digPtr] = 10; //empty character
      newDigit[--digPtr] = 16; //upper circle = 16
      newDigit[--digPtr] = 18; // lower circle = 18
    }
  } //4 tubes only
  else {
    newDigit[--digPtr] = 17; //  "%"
  }

  if (prm.animMode == 0) memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
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

  if (prm.animMode == 0) memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
  colonBlinkState = true;
  decimalpointON = true;
}


void displayDate() {
  byte m = prm.dateMode;
  if (m>2) m=2;
  byte p[3][8] =
  { //tube# 76543210    543210    3210
    {2,1,0}, //ddmmyyyy    ddmmyy    ddmm
    {2,0,1}, //mmddyyyy    mmddyy    mmdd
    {0,1,2} //yyyymmdd    yymmdd    mmdd
  };

  int t = 0;
  if (thermometerClock) t = 1;
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
      } //!thermometerClock
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
  if (prm.animMode == 1) memcpy(oldDigit, newDigit, sizeof(oldDigit)); //don't do animation
}

void displayTime4() {
  for (int i = 0; i < maxDigits; i++) digitDP[i] = false;
  digitDP[4] = true; digitDP[2] = true;
  int hour12_24 = prm.set12_24 ? (byte)hour() : (byte)hourFormat12();
  if (showDate && prm.enableTimeDisplay) displayDate();
  else if (showTemp1) displayTemp(1);
  else if (showTemp0) displayTemp(0);
  else if (showHumid1) displayHumid(1);
  else if (showHumid0) displayHumid(0);
  else if (showPress0) displayPressure(0);
  else if (prm.enableTimeDisplay) {
    showClock = true;
    newDigit[3] = hour12_24 / 10;
    if ((!prm.showZero) && (newDigit[3] == 0)) newDigit[3] = 10;
    newDigit[2] = hour12_24 % 10;
    newDigit[1] = minute() / 10;
    newDigit[0] = minute() % 10;
    if (prm.enableBlink && (second() % 2 == 0)) digitDP[2] = false;
  }
}


void displayTime6() {
  for (int i = 0; i < maxDigits; i++) digitDP[i] = false;
  digitDP[4] = true; digitDP[2] = true;
  int hour12_24 = prm.set12_24 ? (byte)hour() : (byte)hourFormat12();
  if (showDate && prm.enableTimeDisplay) displayDate();
  else if (showTemp1) displayTemp(1);
  else if (showTemp0) displayTemp(0);
  else if (showHumid1) displayHumid(1);
  else if (showHumid0) displayHumid(0);
  else if (showPress0) displayPressure(0);
  else if (prm.enableTimeDisplay) {
    showClock = true;
    if (thermometerClock) {
      newDigit[4] = hour12_24 / 10;
      if ((!prm.showZero) && (newDigit[4] == 0)) newDigit[4] = 10;
      newDigit[3] = hour12_24 % 10;
      newDigit[2] = minute() / 10;
      newDigit[1] = minute() % 10;
      newDigit[0] = 10;
      newDigit[5] = 10;
      if (prm.enableBlink && (second() % 2 == 0)) digitDP[3] = false;
    }
    else { //normal clock
      newDigit[5] = hour12_24 / 10;
      if ((!prm.showZero) && (newDigit[5] == 0)) newDigit[5] = 10;
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
  if (showDate && prm.enableTimeDisplay) displayDate();
  else if (showTemp1) displayTemp(1);
  else if (showTemp0) displayTemp(0);
  else if (showHumid1) displayHumid(1);
  else if (showHumid0) displayHumid(0);
  else if (showPress0) displayPressure(0);
  else if (prm.enableTimeDisplay) {
      showClock = true;
      newDigit[8] = 10; //sign digit = BLANK
      newDigit[7] = hour12_24 / 10;
      if ((!prm.showZero) && (newDigit[7] == 0)) newDigit[7] = 10;
      newDigit[6] = hour12_24 % 10;
      newDigit[5] = 11; //- sign
      newDigit[4] = minute() / 10;
      newDigit[3] = minute() % 10;
      newDigit[2] = 11; //- sign
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





  anim = prm.animMode;







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
        digit[i] = newDigit[i]; //show special characters ASAP or if special char changes to numbers
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
              Fdelay(50 /*Animation speed in millisec */);
            } //end for i
            writeDisplaySingleGuarded();
          } //endif
        } //end for tube
        break;
      case 2:
        for (int i = 0; i <= 9; i++) {
          for (int tube = j; tube < maxDigits; tube++) {
            if ((newDigit[tube] != oldDigit[tube]) && (newDigit[tube] <= 9))
              digit[tube] = (oldDigit[tube] + i) % 10;
          } //end for tube
          writeDisplaySingleGuarded();
          Fdelay(50 /*Animation speed in millisec */);
        } //end for i
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
          Fdelay(50 /*Animation speed in millisec */);
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
          Fdelay(50 /*Animation speed in millisec */);
        } //end for i
        break;
      case 5:
        memset(animMask, 0, sizeof(animMask));
# 4623 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
        for (int i = 1; i <= 5; i++) {
          for (int tube = j; tube < maxDigits; tube++) {
            if (oldDigit[tube] != newDigit[tube]) animMask[tube] = i; //digit is changed
          } //end for tube
          writeDisplaySingleGuarded();
          Fdelay(50 /*Animation speed in millisec */);
        } //end for i
        memcpy(digit, newDigit, sizeof(digit));
        for (int i = 1; i <= 5; i++) {
          for (int tube = j; tube < maxDigits; tube++) {
            if (oldDigit[tube] != newDigit[tube]) animMask[tube] = 6 - i; //digit is changed
          } //end for tube
          writeDisplaySingleGuarded();
          Fdelay(50 /*Animation speed in millisec */);
        } //end for i

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
              Fdelay(max(18, (50 /*Animation speed in millisec */ * 3) / 4));
            }

            digit[order[locked]] = newDigit[order[locked]];
            writeDisplaySingleGuarded();
            Fdelay(max(22, 50 /*Animation speed in millisec */));
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
            Fdelay(50 /*Animation speed in millisec */);
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

    static boolean oldState = !0x1 /*How to switch ON alarm buzzer*/;
    if (oldState != newState) {
      oldState = newState;
      //if (newState == ALARM_ON) DPRINTLN("Alarm ON"); else DPRINTLN("Alarm OFF");
      digitalWrite(2 /*Alarm buzzer pin*/, newState);
    }

}

void alarmSound(void) {
  static const unsigned int t[] = {0, 3000, 6000, 6200, 9000, 9200, 9400, 12000, 12200, 12400, 15000, 15200, 15400};
  static int count = 0;
  static unsigned long nextEvent;
  const int cMax = sizeof(t) / sizeof(t[0]); //number of time steps

  if (prm.alarmEnable) {
    if ( (!alarmON && prm.alarmHour == hour()) && (prm.alarmMin == minute()) && (second() <= 3)) { //switch ON alarm sound
      DebugOut.println("Alarm started!");
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
    writeAlarmPin(!0x1 /*How to switch ON alarm buzzer*/);
    return; //nothing to do
  }

  if ((millis() - alarmStarted) > 1000 * (long)prm.alarmPeriod) {
    alarmON = false; //alarm period is over
    darkenNeopixels();
    DebugOut.println("Alarm ended.");
  }

  if (0 /*Alarm switch off button pin*/ >= 0) { //is button installed?
    if (digitalRead(0 /*Alarm switch off button pin*/) == 0x0) { //stop alarm
      DebugOut.println("Alarm stopped by button press.");
      alarmON = false;
    }
  }

  if (!prm.alarmEnable || !alarmON) { //no alarm, switch off
    writeAlarmPin(!0x1 /*How to switch ON alarm buzzer*/);
    return;
  }

  //-------- Generate sound --------------------------------

  if (millis() > nextEvent) { //go to the next event
    if (count % 2 == 0) {
      nextEvent += 500;
      writeAlarmPin(0x1 /*How to switch ON alarm buzzer*/);
      //DPRINT(" Sound ON");  DPRINT("  Next:"); DPRINTLN(nextEvent);
    }
    else {
      writeAlarmPin(!0x1 /*How to switch ON alarm buzzer*/);
      nextEvent = (count / 2 < cMax) ? alarmStarted + t[count / 2] : nextEvent + 500;
      //DPRINT("   OFF"); DPRINT("  Next:"); DPRINTLN(nextEvent);
    }
    count++;
  }
}


void evalShutoffTime(void) { // Determine whether  tubes should be turned to NIGHT mode

  if (!prm.enableAutoShutoff) return;

  int mn = 60 * hour() + minute();
  int mn_on = prm.dayHour * 60 + prm.dayMin;
  int mn_off = prm.nightHour * 60 + prm.nightMin;

  static bool prevShutoffState = true;
  if ( (((mn_off < mn_on) && (mn >= mn_off) && (mn < mn_on))) || // Nighttime
       ((mn_off > mn_on) && ((mn >= mn_off) || (mn < mn_on)))) {
    if (!manualOverride) displayON = false;
    if (prevShutoffState == true) manualOverride = false;
    prevShutoffState = false;
  }
  else { // Tubes should be on
    if (!manualOverride) displayON = true;
    if (prevShutoffState == false) manualOverride = false;
    prevShutoffState = true;
  }
  return;
  DebugOut.print("mn="); DebugOut.print(mn);
  DebugOut.print("  mn_on="); DebugOut.print(mn_on);
  DebugOut.print("  mn_off="); DebugOut.print(mn_off);
  DebugOut.print("  manOverride:"); DebugOut.print(manualOverride);
  DebugOut.print("  displayON:"); DebugOut.println(displayON);
  DebugOut.print("  enableBlink:"); DebugOut.println(prm.enableBlink);
  DebugOut.print("  blinkState:"); DebugOut.println(colonBlinkState);
}


void writeIpTag(byte iptag) {




  byte offset = 0;


  memset(newDigit, 10, sizeof(newDigit));
  if (!digitsOnly && (maxDigits >= 6)) {
    newDigit[5] = 19; //'I'
    newDigit[4] = 14; //'P'
  }
  //if (iptag >= 100) 
  newDigit[2 + offset] = iptag / 100;
  newDigit[1 + offset] = (iptag % 100) / 10;
  newDigit[0 + offset] = iptag % 10;
  memcpy(digit,newDigit,sizeof(digit));
  printDigits(0);
  changeDigit();
}

void showMyIp(void) { //at startup, show the web page internet address
  clearDigits();

  for (int i=0;i<4;i++) {
    ipShowRunning = true;
    writeIpTag(ip[i]);
    Fdelay(1500);
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
  wifi_mode_t wm = WiFi.getMode();
  if ((wm == WIFI_MODE_AP) || (wm == WIFI_MODE_APSTA)) {
    return;
  }

  const unsigned long nowMs = millis();
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiRecoveryApMode) {
      DebugOut.println("WiFi recovered. Leaving AP recovery mode.");
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
    DebugOut.println("WiFi lost: starting recovery.");
  }

  // Throttle attempts so loop timing remains stable.
  if ((nowMs - lastAttemptMs) < 30000UL) return; // every 30 sec
  lastAttemptMs = nowMs;

  DebugOut.println("Lost WiFi. Reconnect attempt.");
  WiFi.disconnect();
  delay(50);
  WiFi.reconnect();
  delay(50);
  if (WiFi.status() == WL_CONNECTED) {
    DebugOut.println("WiFi reconnected.");
    lostSinceMs = 0;
    failedReconnectAttempts = 0;
    return;
  }

  failedReconnectAttempts++;
  if (failedReconnectAttempts >= 2) {
    DebugOut.println("WiFi reconnect failed twice. Switching to standalone AP mode for web access.");
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
        DebugOut.println("WiFi switch successful, credentials saved.");
      }
      wifiSwitchPending = false;
      wifiSwitchRollbackRunning = false;
      wifiRecoveryApMode = false;
      return;
    }

    // Connected, but not to target. Keep waiting until timeout, then rollback if needed.
    if ((nowMs - wifiSwitchStartMs) > WIFI_SWITCH_TIMEOUT_MS) {
      if (!wifiSwitchRollbackRunning && wifiSwitchCanRollback && wifiSwitchPrevSsid.length() > 0) {
        DebugOut.println("WiFi switch timed out on wrong SSID, rolling back.");
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
      DebugOut.println("WiFi switch failed, starting rollback to previous SSID.");
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
  DebugOut.print("Testing tubes: ");
  for (int i = 0; i < 10; i++) {
    DebugOut.print(i); DebugOut.print(" ");
    for (int j = 0; j < maxDigits; j++) {
      newDigit[j] = i;
      digit[i] = i;
      digitDP[j] = i % 2;
    }
    changeDigit();
    Fdelay(dely);
  }
  DebugOut.println(" ");
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
    DebugOut.print("Temperature ("); DebugOut.print(validTempCount); DebugOut.print("): ");
    bool first = true;
    for (int i=0; i<useTemp; i++) {
      if (!isValidTemperatureReading(temperature[i])) continue;
      if (!first) DebugOut.print(", ");
      DebugOut.print(temperature[i]);
      first = false;
    }
    DebugOut.println(" ");
  }

  int validHumidCount = 0;
  for (int i=0; i<useHumid; i++) {
    if (isValidHumidityReading(humid[i])) validHumidCount++;
  }
  if (validHumidCount > 0) {
    DebugOut.print("Humidity    ("); DebugOut.print(validHumidCount); DebugOut.print("): ");
    bool first = true;
    for (int i=0; i<useHumid; i++) {
      if (!isValidHumidityReading(humid[i])) continue;
      if (!first) DebugOut.print(", ");
      DebugOut.print(humid[i]);
      first = false;
    }
    DebugOut.println(" ");
  }
}

void printChar(int i) {
  if (i < 10) {DebugOut.print(i);}
    else if (i==10) {DebugOut.print(" ");}
    else if (i==15) {DebugOut.print("C");}
    else if (i==16) {DebugOut.print("°");}
    else if (i==18) {DebugOut.print(".");}
    else if (i==17) {DebugOut.print("%"); }
    else if (i==19) {DebugOut.print("I");}
    else if (i==14) {DebugOut.print("P");}
    else DebugOut.print("-");

    if (digitDP[i]) {DebugOut.print(".");} else {DebugOut.print(" ");}
}

void printDigits(unsigned long timeout) {
  static unsigned long lastRun = millis();

  if ((millis() - lastRun) < timeout) return;
  lastRun = millis();


  DebugOut.print("   digit: ["); for (int i = maxDigits - 1; i >= 0; i--) {printChar(digit[i]);} DebugOut.print("]");
  DebugOut.print(colonBlinkState ? " * " : "   ");
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
# 5163 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
  //DPRINT("INT:"); DPRINT(intCounter); intCounter = 0;  //show multiplex interrupt counter
  //DPRINT(" ESaving:"); DPRINT(EEPROMsaving);
  if (useLux>0) {
    DebugOut.print("  Lux:"); DebugOut.print(lx);
  }
  if (34>=0) {
    //DPRINT("  RadarPin:"); DPRINT(digitalRead(RADAR_PIN)); DPRINT("  lastOn:"); DPRINT(radarLastOn);  DPRINT("  sec:"); DPRINTLN((millis()-radarLastOn)/1000);  
  }
  //DPRINT("  tON:"); DPRINT(timerON); DPRINT("  tOFF:"); DPRINT(timerOFF);   //Multiplex timing values for testing
  if (WiFi.status() != WL_CONNECTED) DebugOut.print("  no WIFI");
  DebugOut.println(" ");
  printSensors();

}



// Returns current motion state (radar/PIR), from GPIO if available, else from MQTT state.
bool isMotionDetectedNow() {

  if (34 >= 0) {
    return digitalRead(34);
  }

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
    DebugOut.printf("RADAR: motion detected (GPIO%d)\n", (int)34);
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
    DebugOut.print("DISPLAY: ");
    DebugOut.println(tubesPowerState ? "ON" : "OFF");
    if (!tubesPowerState) {
      // Ensure tubes and LEDs immediately go dark when display is OFF.
      clearDigits();
      rawWrite(); // push "all off" to HV driver

    darkenNeopixels();

    }
  }


  // ---- Debug snapshot fields (updated every loop, printed once/sec by debugSnapshot()) ----

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


  // If you have a real HV power pin, use it as well.
  if ((((1ULL << ((gpio_num_t)27)) & ((0xFFFFFFFFFFULL & ~(0ULL | 0x01000000 | 0x10000000 | 0x20000000 | 0x40000000 | 0x80000000)) & ~(0ULL | (0x00000004ULL << 32) | (0x00000008ULL << 32) | (0x00000010ULL << 32) | (0x00000020ULL << 32) | (0x00000040ULL << 32) | (0x00000080ULL << 32)))) != 0)) {

    digitalWrite((int)27, tubesPowerState ? 0x1 : !0x1);



  }

}

//Calculation parameters are defined in clocks.h
//https://www.pcboard.ca/ldr-light-dependent-resistor

int luxMeter(void) {
# 5310 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ClockForgeOS.ino"
  return (0);

}

void getLightSensor(void) {
  static unsigned long lastRun = 0;
  static int oldLx = 0;
  int tmp;
  if ((millis()-lastRun)<500) return;
  lastRun = millis();

  if (BH1750exist) {
    tmp = getBH1750();
    if ((abs(tmp-oldLx)>1) || (tmp >= 100)) {
      lx=tmp; oldLx = lx;
    }
    if (lx>=100 -2) lx = 100;
    autoBrightness = prm.enableAutoDim;
  }
  else if (LDRexist) {
    tmp = luxMeter();
    if ((abs(tmp-oldLx)>1) || (tmp >= 100)) {
      lx=tmp; oldLx = lx;
    }
    if (lx>=100 -2) lx = 100;
    autoBrightness = prm.enableAutoDim;
  }
  else {
    lx = 100;
    autoBrightness = false;
  }
}

void checkWifiMode() {
static boolean oldMode = prm.wifiMode;

  if (oldMode != prm.wifiMode) {
    if (prm.wifiMode) {
        startNewWifiMode();
      }
      else {
        startStandaloneMode();
      }
       oldMode = prm.wifiMode;
    }
}

void checkPrm() {
  static byte oldPrm[sizeof(prm)];
  int tmp = memcmp(oldPrm,&prm,sizeof(prm));
  if (tmp!=0){
    DebugOut.print("Prm mismatch!"); DebugOut.println(tmp);
    memcpy(oldPrm,&prm,sizeof(prm));
  }
}


static void calibrateTouchButton() {
  if (33 < 0) return;
  uint32_t sum = 0;
  const int samples = 16;
  for (int i = 0; i < samples; i++) {
    sum += (uint16_t)touchRead(33);
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
    {255, 0, 0},
    {255, 48, 0},
    {255, 96, 0},
    {255, 160, 0},
    {255, 220, 0},
    {180, 255, 0},
    {100, 255, 0},
    { 0, 255, 0},
    { 0, 255, 120},
    { 0, 255, 220},
    { 0, 180, 255},
    { 0, 110, 255},
    { 0, 40, 255},
    { 90, 0, 255},
    {160, 0, 255},
    {230, 0, 255},
    {255, 0, 190},
    {255, 0, 120},
    {255, 255, 255},
    {255, 190, 120},
    {255, 120, 20},
    {255, 80, 160}
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
      writeAlarmPin(!0x1 /*How to switch ON alarm buzzer*/);
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
  if (33 < 0) return;
  if (touchThreshold == 0) {
    calibrateTouchButton();
  }

  uint32_t nowMs = millis();
  uint16_t raw = (uint16_t)touchRead(33);
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

  static bool neopixelsForcedOff = false;



  if (tubesPowerState || alarmON) {
    neopixelsForcedOff = false;
    doAnimationMakuna();
  } else {
    if (!neopixelsForcedOff) {
      darkenNeopixels();
      neopixelsForcedOff = true;
    }
  }



  if (tubesPowerState || alarmON) {
    doAnimationPWM();
  }
  alarmSound();
  checkTubePowerOnOff();

  processTouchButton();

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
      wifi_mode_t wm = WiFi.getMode();
      bool apActive = (wm == WIFI_MODE_AP) || (wm == WIFI_MODE_APSTA);
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
  DebugOut.println("Restart Clock...");




  WiFi.disconnect();
  delay(1000);
  ESP.restart();
}



  void writeDisplay2(void){} //not interrupt driven display handler
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\DHT_temp.ino"
# 91 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\DHT_temp.ino"
void setupDHTemp() {}
void getDHTemp() {}
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\HV5122.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\HV5122new.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
//BME280, and AHT20+BMP280 sensor drivers


//#define USE_BME280
//#define USE_BMP280
//#define USE_AHTX0
//#define USE_SHT21
//#define USE_BH1750

# 11 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino" 2
# 12 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino" 2
# 25 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
# 26 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino" 2
  Adafruit_BMP280 bmp; // use I2C interface   address = 0x76 or 0x77
  Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
  Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
  byte BMP280tempPtr;
  byte BMP280humidPtr;
  byte BMP280pressPtr;



# 36 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino" 2
  Adafruit_AHTX0 aht;
  Adafruit_Sensor *aht_humidity; //, *aht_temp;
  //byte AHTX0tempPtr;
  byte AHTX0humidPtr;
# 51 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
# 52 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino" 2
  BH1750 lightMeter(0x23);


void setupI2Csensors() {
  DebugOut.println("Starting I2Csensors sensors...");

  pinMode(21 /* you can set the used SDA and SCL pins*/,0x05); regPin(21 /* you can set the used SDA and SCL pins*/,"PIN_SDA");
  pinMode(22 /* if it is not default value       */,0x05); regPin(22 /* if it is not default value       */,"PIN_SCL");
  Wire.begin(21 /* you can set the used SDA and SCL pins*/,22 /* if it is not default value       */);
  delay(100);
  I2Cscanner();


  DebugOut.println("Starting RTC Clock...");
  while(millis()<2000) {yield();} //waiting for 2 sec from startup
  //I2C_ClearBus();
  delay(100);

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

    if (error68 == 0) { //DS3231 at 0x68
      RTCexist = true;
      RTCisPCF8563 = false;
      DebugOut.println("RTC found on 0x68.");
    }
    else if (error51 == 0) {
      RTCexist = true;
      RTCisPCF8563 = true;
      DebugOut.println("RTC found on 0x51 (PCF8563).");
    }
    else {
      RTCexist = false;
      RTCisPCF8563 = false;
      DebugOut.println("!!!No RTC found on 0x68!!!");
      byte error57 = probeI2C(0x57);
      if (error57 == 0) {
        DebugOut.println("I2C 0x57 is present (AT24C32 EEPROM on many DS3231 modules). Check RTC wiring/power and SDA/SCL pin mapping.");
      }
      else if (error51 == 0) {
        DebugOut.println("I2C 0x51 is present (likely PCF8563 RTC). This firmware RTC driver expects DS3231 at 0x68.");
      }
      else {
        DebugOut.println("No response on 0x68 or 0x57. Check VCC/GND and configured PIN_SDA/PIN_SCL for this clock profile.");
      }
    }
# 136 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
  if (!bmp.begin(0x77)) {
    DebugOut.println("Could not find a valid BMP280 sensor on address 0x77, check wiring!");
  }
  else {
    BMP280exist = true;
    /* BMP280 default settings from datasheet. */
    DebugOut.println("BMP280 Found!");
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, /* Operating Mode. */
    Adafruit_BMP280::SAMPLING_X2, /* Temp. oversampling */
    Adafruit_BMP280::SAMPLING_X16, /* Pressure oversampling */
    Adafruit_BMP280::FILTER_X16, /* Filtering. */
    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    bmp_temp->printSensorDetails();
    BMP280tempPtr = useTemp; //remember my ID-s
    BMP280pressPtr = usePress; //remember my ID-s
    useTemp++; //increase the number of sensors
    usePress++; //increase the number of sensors
  }



  if (!aht.begin()) {
    DebugOut.println("Could not find a valid AHT10/AHT20 sensor, check wiring!");
  }
  else {
    AHTX0exist = true;
    DebugOut.println("AHT10/AHT20 Found!");
    //aht_temp = aht.getTemperatureSensor();
    //aht_temp->printSensorDetails();
    aht_humidity = aht.getHumiditySensor();
    aht_humidity->printSensorDetails();
    //AHTX0tempPtr = useTemp;  //remember my ID-s
    AHTX0humidPtr = useHumid;
    //useTemp++;   //increase the number of sensors
    useHumid++;
  }

  DebugOut.println(" ");
# 195 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    DebugOut.println("BH1750 luxmeter sensor found");
    BH1750exist = true;
    useLux++;
  }
  else {
    DebugOut.println("Error initialising BH1750 on address 0x23");
  }


  DebugOut.println(" ");
} //end of SetupI2Csensors()


void getBME280() {
# 227 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
}


void getBMP280() {

  temperature[BMP280tempPtr] = round1(bmp.readTemperature());
  pressur[BMP280pressPtr] = round1(bmp.readPressure()/100);
  DebugOut.print("BMP280: ");
  DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>(("Temperature = ")))));
  DebugOut.print(temperature[BMP280tempPtr]);
  DebugOut.print(" *C");

  DebugOut.print(((reinterpret_cast<const __FlashStringHelper *>(("  Pressure = ")))));
  DebugOut.print(pressur[BMP280pressPtr]);
  DebugOut.println(" hPa");

}

void getAHTX0() {

  //  /* Get a new normalized sensor event */
  sensors_event_t humidity;
  //sensors_event_t temp;
  aht_humidity->getEvent(&humidity);
  //aht_temp->getEvent(&temp);
  //temperature[AHTX0tempPtr] = round1(temp.temperature);
  DebugOut.print("AHT sensor:  ");
  //DPRINT("  Temperature ");
  //DPRINT(temperature[AHTX0tempPtr]);
  //DPRINT(" deg C");

  DebugOut.print("  Humidity: "); /* Display the results (humidity is measured in % relative humidity (% rH) */
  humid[AHTX0humidPtr] = round1(humidity.relative_humidity);
  DebugOut.print(humid[AHTX0humidPtr]);
  DebugOut.println(" % rH");

}

void getSHT21() {
# 278 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
}



void getI2Csensors() {
  static unsigned long lastRun = 0;

  if (((millis()-lastRun)<30000) && (lastRun !=0)) return; // || (second()!=TEMP_START)
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
  DebugOut.println(((reinterpret_cast<const __FlashStringHelper *>(("________________________________")))));
  DebugOut.println("Scanning I2C devices...");

  for(address = 1; address < 127; address++ ) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      DebugOut.print("I2C device found at address 0x");
      if (address<16)
        DebugOut.print("0");
      DebugOut.print(address,16);
      DebugOut.println("  !");

      nDevices++;
    }
    else if (error==4) {
      DebugOut.print("Unknown error at address 0x");
      if (address<16)
        DebugOut.print("0");
      DebugOut.println(address,16);
    }
  } //end for

  if (nDevices == 0)
    DebugOut.println("No I2C devices found\n");
  DebugOut.println(((reinterpret_cast<const __FlashStringHelper *>(("_________________________________")))));
}
# 337 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\I2Csensors.ino"
int getBH1750() {
  static float oldLux = 100;

  if (lightMeter.measurementReady()) {
    float lux = lightMeter.readLightLevel();
    //DPRINT("BH1750 Light: "); DPRINT(lux); DPRINTLN(" lx");

    if (lux>100) lux = 100; //Limited //Limit lux value to maximum
    if (lux<0) lux = oldLux;
    oldLux = oldLux + (lux-oldLux)/10; //slow down Lux change
  }
  return (int)oldLux;
}
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MAX6921.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MAX6921_ESP32.ino"

//VFD driver driver for ESP32
  char tubeDriver[] = "MAX6921_ESP32";





int __attribute__((section(".dram1" "." "130"))) maxDig; //memory variable version of maxDigits
//------------------abcdefgDP----------------   definition of different characters
byte __attribute__((section(".dram1" "." "131"))) charDefinition[] = {
                   252, //0: abcdef
                   96, //1: bc 
                   218, //2: abdeg
                   242, //3: abcdg
                   102, //4: bcfg
                   182, //5: acdfg
                   190, //6: acdefg
                   224, //7: abc
                   254, //8: abcdefg
                   246, //9: abcdfg
                   0, // : BLANK (10)
                   2, //-: minus (11)
                   1, // decimal point (12)
                   238, // A  abcefg (13)
                   206, // P  abefg (14)
                   156, // C  adef (15)
                   198, //grad  (upper circle) abfg (16)
                   180, //%  acdf  (17)
                   58, //lower circle cdeg  (18)                  
                   96, //I  bc    (19)
                   142 //F  aefg  (20)
};

uint32_t __attribute__((section(".dram1" "." "132"))) animationMaskBits[5];



int maxDigits = sizeof(digitEnablePins);

uint32_t __attribute__((section(".dram1" "." "133"))) charTable[sizeof(charDefinition)]; //generated pin table from segmentDefinitions
uint32_t __attribute__((section(".dram1" "." "134"))) segmentEnableBits[sizeof(segmentEnablePins)]; //bitmaps, generated from EnablePins tables
uint32_t __attribute__((section(".dram1" "." "135"))) digitEnableBits[10];

int __attribute__((section(".dram1" "." "136"))) PWMrefresh=5500; ////msec, Multiplex time period. Greater value => slower multiplex frequency
int __attribute__((section(".dram1" "." "137"))) PWM_min = 500;
int __attribute__((section(".dram1" "." "138"))) PWM_max = 5000;
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
  DebugOut.println("VFD Clock - setup MAX6921 pins");
  pinMode(4 /* Shift Register Latch Enable*/, 0x03); regPin(4 /* Shift Register Latch Enable*/,"PIN_LE");
  pinMode(32 /* Shift Register Blank (1=display off     0=display on)*/, 0x03); regPin(32 /* Shift Register Blank (1=display off     0=display on)*/,"PIN_BL");
  digitalWrite(32 /* Shift Register Blank (1=display off     0=display on)*/,0x0); //brightness
  pinMode(16 /* Shift Register Data*/,0x03); regPin(16 /* Shift Register Data*/,"PIN_DATA");
  pinMode(17 /* Shift Register Clock*/, 0x03); regPin(17 /* Shift Register Clock*/,"PIN_CLK");

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

void __attribute__((section(".iram1" "." "139"))) writeDisplay(){ //void IRAM_ATTR  writeDisplay(){
  static __attribute__((section(".dram1" "." "140"))) int timer = PWMrefresh;
  static __attribute__((section(".dram1" "." "141"))) uint32_t val;
  static __attribute__((section(".dram1" "." "142"))) byte pos = 0;
  static __attribute__((section(".dram1" "." "143"))) boolean state=true;
  static __attribute__((section(".dram1" "." "144"))) int brightness;
  static __attribute__((section(".dram1" "." "145"))) int PWMtimeBrightness=PWM_min;

  if (EEPROMsaving) { //stop refresh, while EEPROM write is in progress!
      //digitalWrite(PIN_BL,HIGH);    //OFF
    return;
  }

  vPortEnterCritical(&timerMux);
  do { __extension__({ unsigned __tmp; __asm__ __volatile__( "rsil	%0, " "3 /* level masked by PS.EXCM */" "\n" : "=a" (__tmp) : : "memory" ); __tmp;}); ; } while (0);
  intCounter++;

  brightness = displayON ? prm.dayBright : prm.nightBright;
  if (brightness>100) brightness = 100; //only for safety

  if ((!autoBrightness) && (brightness==100)) state = true;

  if (state) { //ON state
    pos++; if (pos>maxDigits-1) { //go to the tube#0
      pos = 0;
      if (autoBrightness && displayON) { //change brightness only on the tube#0
        PWMtimeBrightness = max(PWM_min,PWM_max*lx/100);
        }
      else
        PWMtimeBrightness = max(PWM_min,PWM_max*brightness/100);
    }

    val = (digitEnableBits[pos] | charTable[digit[pos]]); //the full bitmap to send to MAX chip
    if (digitDP[pos]) val = val | charTable[12]; //Decimal Point

    timer = PWMtimeBrightness;
    //if (pos==2) timer = 3*timer;  //Weak IV11 tube#2 brightness compensation
    timerON = timer;
    timerOFF = PWMrefresh-PWMtimeBrightness;
  }
  else { //OFF state
    timer = PWMrefresh-PWMtimeBrightness;
  }

  if (timer<500) timer = 500; //safety only...

  if ( (brightness == 0) || (!state) || (!radarON)) { //OFF state, blank digit
    digitalWrite(32 /* Shift Register Blank (1=display off     0=display on)*/,0x1); //OFF
  }
  else { //ON state
    if (animMask[pos]>0) val &= animationMaskBits[animMask[pos]-1]; //animationMode 6, mask characters from up to down and back
    for (int i=0; i<20; i++) {
      if (val & uint32_t(1 << (19 - i)))
        {digitalWrite(16 /* Shift Register Data*/, 0x1); //asm volatile ("nop");
        }
      else
        {digitalWrite(16 /* Shift Register Data*/, 0x0); //asm volatile ("nop");
        }

      digitalWrite(17 /* Shift Register Clock*/,0x1); //asm volatile ("nop");  //delayMS(1);
      digitalWrite(17 /* Shift Register Clock*/,0x0); //asm volatile ("nop"); //delayMS(1);
      } //end for      

    digitalWrite(4 /* Shift Register Latch Enable*/,0x1 ); asm volatile ("nop");
    digitalWrite(4 /* Shift Register Latch Enable*/,0x0);
    digitalWrite(32 /* Shift Register Blank (1=display off     0=display on)*/,0x0 ); //ON
  } //end else

  vPortExitCritical(&timerMux);
  state = !state;
  //ESP32timer = timerBegin(0, PRESCALER, true);  //set prescaler, true = edge generated signal
  //timerAttachInterrupt(ESP32timer, &writeDisplay, true);   
  timerAlarmWrite(ESP32timer, timer, true);
  timerAlarmEnable(ESP32timer);
  do { ; __extension__({ unsigned __tmp; __asm__ __volatile__( "rsil	%0, " "0" "\n" : "=a" (__tmp) : : "memory" ); __tmp;}); } while (0);
}

void generateBitTable() {
uint32_t out;

  DebugOut.println("--- Generating segment pins bitmap ---");
  for (int i=0;i<sizeof(segmentEnablePins);i++) {
    segmentEnableBits[i] = uint32_t(1<<segmentEnablePins[i]);
    //DPRINT(i); DPRINT(": "); DPRINTLN(segmentEnableBits[i],BIN);
  }
  animationMaskBits[0] = uint32_t(1<<segmentEnablePins[0]); //a
  animationMaskBits[1] = animationMaskBits[0] | uint32_t(1<<segmentEnablePins[1]) | uint32_t(1<<segmentEnablePins[5]); //bf
  animationMaskBits[2] = animationMaskBits[1] | uint32_t(1<<segmentEnablePins[6]); //g
  animationMaskBits[3] = animationMaskBits[2] | uint32_t(1<<segmentEnablePins[4]) | uint32_t(1<<segmentEnablePins[2]); //ec
  animationMaskBits[4] = animationMaskBits[3] | uint32_t(1<<segmentEnablePins[3]); //d
  for (int i=0;i<5;i++) {
    animationMaskBits[i] = ~animationMaskBits[i]; //invert bits
    //DPRINTLN(animationMaskBits[i],HEX);
  }
  DebugOut.println("--- Generating digit pins bitmap ---");
  for (int i=0;i<maxDigits;i++) {
    digitEnableBits[i] = uint32_t(1 << digitEnablePins[i]);
    //DPRINT(i); DPRINT(": "); DPRINTLN( digitEnableBits[i],BIN);
  }

DebugOut.println("---- Generated Character / Pins table -----");
  for (int i=0;i<sizeof(charDefinition);i++) {
    out = 0;
    //DPRINT(i); DPRINT(":  ");
    //DPRINT(charDefinition[i],BIN);  //DPRINT(" = ");
    for (int j=0;j<=7;j++) //from a to g
      if ((charDefinition[i] & 1<<(7-j)) != 0) {
        out = out | segmentEnableBits[j]; //DPRINT("1"); 
        }
    //else        DPRINT("0");
    //DPRINT("  >> ");  

    charTable[i] = out;
    //DPRINTLN(charTable[i],BIN);
  } //end for
}


void clearTubes() {
  digitalWrite(32 /* Shift Register Blank (1=display off     0=display on)*/,0x1);
}

void writeDisplaySingle() {}
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MAX7219CNG.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\MM5450.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\Numitron_4511N.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\PT6311.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\PT6355.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\SN75512.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\SN75518.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\VQC10.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\ZM1500.ino"
// ZM1500 Clock by UNFI
//  4x 74HC595N shift register + optocouplers
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dallas_temp.ino"
# 140 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dallas_temp.ino"
void setupDallasTemp() {}
void requestDallasTemp(boolean force) {}
void getTemp() {}
void resetSensors() {}
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\dummy.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\gesture_apds9960.ino"
# 79 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\gesture_apds9960.ino"
void setupGestureSensor() {}
void processGestureSensor() {}
bool isGestureSensorPresent() { return false; }
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\gps.ino"
# 91 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\gps.ino"
void setupGPS() {GPSexist = false;}
boolean getGPS() {return(false);}
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"

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

# 29 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino" 2
//https://pubsubclient.knolleary.net/     https://github.com/knolleary/pubsubclient





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

    localRadarEnabled = (34 >= 0);

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

    DebugOut.print("New message on topic: "); DebugOut.print(topic);
    // Generic external device support
    for (int i = 0; i < 4; ++i) {
      if (strlen(externalDevices[i].topic) > 0 && mqttTopicMatches(externalDevices[i].topic, topic)) {
        float val = parseNumericPayload((const char*)payload);
        externalDevices[i].lastValue = val;
        updateExternalDeviceSensor(externalDevices[i].sensorSlot, val);
        DebugOut.print("External device updated: "); DebugOut.print(externalDevices[i].name); DebugOut.print(" value: "); DebugOut.println(val);
        return;
      }
    }

    const uint16_t copyLen = (length < (sizeof(tmp) - 1)) ? length : (sizeof(tmp) - 1);
    memcpy(tmp, payload, copyLen);
    tmp[copyLen] = '\0';
    DebugOut.print("  Data: "); DebugOut.println(tmp);
    if (mqttMasterTempEnabled && mqttTopicMatches(masterTemperatureTopic, topic)) {
      parseTemperaturePayload(tmp, tempTMP);
      temperature[MASTERtempPtr] = tempTMP;
      mqttLastInboundMs = millis();
      DebugOut.print("MASTER clock's temperature:"); DebugOut.println(tempTMP);
    }
    if (mqttMasterHumidEnabled && mqttTopicMatches(masterHumidityTopic, topic)) {
      parseHumidityPayload(tmp, humidTMP);
      DebugOut.print("MASTER clock's humidity:"); DebugOut.println(humidTMP);
      humid[MASTERhumidPtr] = humidTMP;
      mqttLastInboundMs = millis();
    }
    if (mqttMasterRadarEnabled && mqttTopicMatches(masterRadarTopic, topic)) {
       DebugOut.print("MASTER radar:"); DebugOut.println(tmp[0]);
       if (tmp[0] == '0') {
         mqttRadarON = false;
         radarON = mqttRadarON;
       }
       else {
         mqttRadarON = true;
         radarON = mqttRadarON;
         radarLastOn = millis();
       }
       mqttLastInboundMs = millis();
       DebugOut.print("mqttRadarON:"); DebugOut.println(mqttRadarON);

    }
    if (mqttMasterPressureEnabled && mqttTopicMatches(masterPressureTopic, topic)) {
      parsePressurePayload(tmp, pressureTMP);
      pressur[0] = pressureTMP;
      mqttLastInboundMs = millis();
      DebugOut.print("MASTER pressure:"); DebugOut.println(pressureTMP);
    }
    if (mqttMasterLuxEnabled && mqttTopicMatches(masterLuxTopic, topic)) {
      parseLuxPayload(tmp, luxTMP);
      lx = luxTMP;
      mqttLastInboundMs = millis();
      DebugOut.print("MASTER lux:"); DebugOut.println(luxTMP);
    }
    //mqtt.publish("myPublishTopic", "hello");
}

void onMqttConnected() {
    DebugOut.println("Connected to the broker!");
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
    device.setUniqueId(mac, sizeof(mac)); //set device MAC, as unique ID
    sprintf(mactmp,"%02x%02x%02x%02x%02x%02x",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    masterTemperatureTopic[0] = '\0';
    masterHumidityTopic[0] = '\0';
    masterRadarTopic[0] = '\0';
    masterLuxTopic[0] = '\0';
    masterPressureTopic[0] = '\0';
# 323 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mqtt.ino"
    syncMasterTopicsFromConfig();
    DebugOut.print("Starting MQTT CLient: "); DebugOut.print("clockforgeos"); DebugOut.print("/sensor/"); DebugOut.println(mactmp);

    mqtt.setDiscoveryPrefix("clockforgeos");
    device.setName("fw42v1" /*firmware name*/);
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

      mqttRadar.setUnitOfMeasurement("on/off");
      mqttRadar.setDeviceClass("sensor");
      mqttRadar.setName("Radar sensor");


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

    DebugOut.print("MQTT broker: "); DebugOut.print(mqttHost); DebugOut.print(":"); DebugOut.println(brokerPort);
    DebugOut.print("MQTT user: "); DebugOut.print(mqttUser); DebugOut.print(", pass len: "); DebugOut.println(strlen(mqttPass));

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
      DebugOut.print("MASTER_TEMPERATURE_TOPIC:"); DebugOut.println(masterTemperatureTopic);
      MASTERtempPtr = (useTemp < 6) ? useTemp : 5;
      if (useTemp < 6) useTemp++;
    }
    if (mqttMasterHumidEnabled) {
      DebugOut.print("MASTER_HUMIDITY_TOPIC:"); DebugOut.println(masterHumidityTopic);
      MASTERhumidPtr = (useHumid < 6) ? useHumid : 5;
      if (useHumid < 6) useHumid++;
    }
    if (mqttMasterRadarEnabled) {
      DebugOut.print("MASTER_RADAR_TOPIC:"); DebugOut.println(masterRadarTopic);
    }
    if (mqttMasterLuxEnabled) {
      DebugOut.print("MASTER_LUX_TOPIC:"); DebugOut.println(masterLuxTopic);
      if (useLux == 0) useLux = 1;
    }
    if (mqttMasterPressureEnabled) {
      DebugOut.print("MASTER_PRESSURE_TOPIC:"); DebugOut.println(masterPressureTopic);
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
  if (mqtt.isConnected()) DebugOut.print("MQTT Connected.");
  else {
    DebugOut.println("MQTT Failed to connect.");
    return;
  }
  DebugOut.print("MQTT send:");
  if (useTemp>0) {mqttTemp.setValue(temperature[0]+ prm.corrT0); DebugOut.print(temperature[0]+ prm.corrT0); DebugOut.print(", ");}
  if (useHumid>0) {mqttHumid.setValue(humid[0]+ prm.corrH0); DebugOut.print(humid[0]+ prm.corrH0); DebugOut.print(", ");}
  if (usePress>0) {mqttPress.setValue(pressur[0]); DebugOut.print(pressur[0]); DebugOut.print(", ");}
  if (useLux>0) {mqttLux.setValue(lx); DebugOut.print(lx); DebugOut.print(", ");}

    mqttRadar.setValue(radarON); DebugOut.print(radarON);

  DebugOut.println(" ");
}
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\mulev2.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\multiplex74141.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\multiplex74141_ESP32.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\no_multiplex74141.ino"
// 4digit Nixie Clock 
//  2x 74HC595N shift register + 4x 74141
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\no_multiplex_ESP32.ino"
// 4digit Nixie Clock 
//  2x 74HC595N shift register + 4x 74141
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\pcf_multiplex74141.ino"
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\pwmleds.ino"
# 106 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\pwmleds.ino"
void doAnimationPWM() {}
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"

//DS3231 realtime clock driver
//with manual clock setup buttons
# 5 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino" 2
RtcDS3231<TwoWire> Rtc(Wire);
RtcDateTime rtcNow;
RtcDateTime compiled = RtcDateTime("Feb 17 2026", "09:05:07");


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
    DebugOut.println("PCF8563: voltage low flag set (time may be invalid)");
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
# 81 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
boolean pushedFldButtonValue = 0x0; //wich state means pushed button
boolean pushedSetButtonValue = 0x0; //wich state means pushed button

//------- I2C bus definition   Any pins are usable  Please, SET in clocks.h --------
//#define PIN_SDA 4           //D2   original general setup for 8266
//#define PIN_SCL 5           //D1



int monthdays[13] = {0,31,29,31,30,31,30,31,31,30,31,30,31};

struct Menu {
   short lowlimit; // alsó határ
   short uplimit; // felső határ
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



int mvar[5 +1];

enum switchStates {IS_OPEN, IS_OPENING, IS_PUSHED, IS_LONGPRESSING, IS_LONGPRESSED, IS_PUSHING, IS_LONGOPENING};

int fld = 0;
unsigned long LastModify = 0;

void setupRTC() {
# 134 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
  if (RTCexist) {
    if (RTCisPCF8563) {
      DebugOut.println("Connecting to RTC clock (PCF8563)...");
      int yy, mo, dd, hh, mm, ss;
      if (!pcf8563ReadDateTime(yy, mo, dd, hh, mm, ss)) {
        DebugOut.println("PCF8563 time invalid, setting compile time...");
        pcf8563WriteDateTime(compiled.Year(), compiled.Month(), compiled.Day(), compiled.Hour(), compiled.Minute(), compiled.Second());
      }
      return;
    }

    DebugOut.println("Connecting to RTC clock...");
    Rtc.Begin();
    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) {
            DebugOut.print("RTC communications error = "); DebugOut.println(Rtc.LastError());
        }
        else {
            DebugOut.println("RTC lost confidence in the DateTime!");
            Rtc.SetDateTime(compiled); //Set RTC time to firmware compile time
        }
    }

    if (!Rtc.GetIsRunning()) {
        DebugOut.println("RTC was not actively running, starting now");
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
# 173 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
}


void editor() {
  static unsigned long lastDisplay = 0;
  if (RTCexist && !RTCisPCF8563) {rtcNow = Rtc.GetDateTime();}
  LastModify = 0;
# 202 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
  if (LastModify != 0) saveRTC();
}

void showValue() {
  memset(digit,10,sizeof(digit)); //clear array
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

    //character set not available yet
# 230 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\rtc.ino"
}

//------------------------------------- RTC Clock functions --------------------------------------------------------------------
void updateRTC() {
  DebugOut.println("Update RTC?");
  if (!RTCexist) return;
  if (year()<2020) return; //invalid system date??

  if (RTCisPCF8563) {
    int yy, mo, dd, hh, mm, ss;
    if (!pcf8563ReadDateTime(yy, mo, dd, hh, mm, ss)) {
      DebugOut.println("PCF8563 read failed in updateRTC()");
      return;
    }
    if (abs(ss - second())>2 || (mm != minute()) || (hh != hour()) || (dd != day()) || (mo != month()) || (yy != year())) {
      DebugOut.println("Updating PCF8563 from TimeServer...");
      pcf8563WriteDateTime(year(), month(), day(), hour(), minute(), second());
    }
    return;
  }

  rtcNow = Rtc.GetDateTime();
  if (abs(rtcNow.Second() - second())>2 || (rtcNow.Minute() != minute()) || (rtcNow.Hour() != hour())
      || (rtcNow.Day() != day()) || (rtcNow.Month() != month()) || (rtcNow.Year() != year()) ) {
      DebugOut.println("Updating RTC from TimeServer...");
      DebugOut.print("- DS3231  date:"); DebugOut.print(rtcNow.Year()); DebugOut.print("/"); DebugOut.print(rtcNow.Month()); DebugOut.print("/"); DebugOut.print(rtcNow.Day());
      DebugOut.print(" time:"); DebugOut.print(rtcNow.Hour()); DebugOut.print(":"); DebugOut.print(rtcNow.Minute()); DebugOut.print(":"); DebugOut.println(rtcNow.Second());
      DebugOut.print("- Tserver date:"); DebugOut.print(year()); DebugOut.print("/"); DebugOut.print(month()); DebugOut.print("/"); DebugOut.print(day());
      DebugOut.print(" time:"); DebugOut.print(hour()); DebugOut.print(":"); DebugOut.print(minute()); DebugOut.print(":"); DebugOut.println(second());
      Rtc.SetDateTime(now()-946684800l /*offset from 1970->2000 epoch start*/); //update RTC with UNIX epoch timestamp to 2000year epoch
      } //endif
}


void getRTC() {
  if (RTCexist) {
    if (RTCisPCF8563) {
      int yy, mo, dd, hh, mm, ss;
      if (pcf8563ReadDateTime(yy, mo, dd, hh, mm, ss)) {
        setTime(hh, mm, ss, dd, mo, yy);
      } else {
        DebugOut.println("PCF8563 read failed");
      }
      mvar[1] = year(); mvar[2] = month(); mvar[3] = day();
      mvar[4] = hour(); mvar[5] = minute();
      return;
    }

    if (!Rtc.IsDateTimeValid()) {
        if (Rtc.LastError() != 0) {
            DebugOut.print("RTC communications error = "); DebugOut.println(Rtc.LastError());
        }
        else {
            DebugOut.println("RTC lost confidence in the DateTime!");
        }
    }
    else {
      rtcNow = Rtc.GetDateTime();
      //DPRINT("\nGet DS3231 data&time: ");  DPRINT(rtcNow.Year()); DPRINT("/"); DPRINT(rtcNow.Month()); DPRINT("/"); DPRINT(rtcNow.Day()); 
      //DPRINT(" ");  DPRINT(rtcNow.Hour()); DPRINT(":"); DPRINT(rtcNow.Minute()); DPRINT(":"); DPRINTLN(rtcNow.Second());  
      setTime(rtcNow.Hour(),rtcNow.Minute(),rtcNow.Second(),rtcNow.Day(),rtcNow.Month(),rtcNow.Year()); //set the time (hr,min,sec,day,mnth,yr)
    }
  }
  mvar[1] = year(); mvar[2] = month(); mvar[3] = day();
  mvar[4] = hour(); mvar[5] = minute();
}

void saveRTC() {
  setTime(mvar[4],mvar[5],0,mvar[3],mvar[2],mvar[1]); //set the time (hr,min,sec,day,mnth,yr)

    if (RTCexist) {
      DebugOut.println("Updating RTC from Manual settings...");
      if (RTCisPCF8563) {
        pcf8563WriteDateTime(year(), month(), day(), hour(), minute(), second());
      } else {
        Rtc.SetDateTime(now()-946684800l /*offset from 1970->2000 epoch start*/);
      }
    }
    DebugOut.print("New Date & Time:"); DebugOut.print(year()); DebugOut.print("/"); DebugOut.print(month()); DebugOut.print("/"); DebugOut.print(day());
    DebugOut.print(" time:"); DebugOut.print(hour()); DebugOut.print(":"); DebugOut.print(minute()); DebugOut.print(":"); DebugOut.println(second());
}

//-------------------------- check buttons  ------------------------------------------------

void scanButFLD(unsigned long mill) {
static unsigned long lastRun = millis();
static unsigned long lastPush = millis();;
static switchStates butState;
byte sw;

  if (-1<0) return;
  if ((millis() - lastRun) < mill) return; //refresh rate
  lastRun = millis();

  sw = digitalRead(-1);
  if (pushedFldButtonValue == 0x1) sw = !sw; //inverted button logic

  switch (butState) {
    case IS_OPEN:
      if(sw == 0x0) butState = IS_PUSHING;
      break;
    case IS_PUSHING:
      butState = IS_PUSHED;
      lastPush = millis();
      DebugOut.println(" PUSHED");
      break;
    case IS_PUSHED:
      if(sw == 0x1) butState = IS_OPENING;
      if((sw == 0x0) && ((millis()-lastPush)>2000)) butState = IS_LONGPRESSING;
      break;
    case IS_LONGPRESSING:
      butState = IS_LONGPRESSED;
      DebugOut.println(" IS_LONGPRESSED");
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
      if (sw == 0x1) butState = IS_LONGOPENING;
      break;
    case IS_OPENING:
      butState = IS_OPEN;
      if ((millis()-lastPush)<500) {
           LastModify = millis();
          fld++; if (fld>5) fld = 1;
          DebugOut.print(m1[fld].name); DebugOut.println(fld);
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

  if (-1<0) return;
  if ((millis() - lastRun) < mill) return; //refresh rate
  lastRun = millis();

  sw = digitalRead(-1);
  if (pushedSetButtonValue == 0x1) sw = !sw; //inverted button logic  

  switch (butState) {
    case IS_OPEN:
      if(sw == 0x0) butState = IS_PUSHING;
      break;
    case IS_PUSHING:
      butState = IS_PUSHED;
      lastPush = millis();
      DebugOut.println(" PUSHED");
      break;
    case IS_PUSHED:
      if(sw == 0x1) butState = IS_OPENING;
      if((sw == 0x0) && ((millis()-lastPush)>2000)) butState = IS_LONGPRESSING;
      break;
    case IS_LONGPRESSING:
      butState = IS_LONGPRESSED;
      DebugOut.println(" IS_LONGPRESSED");
      //if (noZero !=0) noZero =0; else noZero = 1;
      //EEPROM.update(EEPROM_NOZERO, noZero);  
      break;
    case IS_LONGPRESSED:
      if (sw == 0x1) butState = IS_LONGOPENING;
       // if ((millis()-lastPush)>10000)  {factoryReset(); resetFunc();}   //reset to factory settings
       LastModify = millis();
       mvar[fld]+=1;
       if (mvar[fld] < m1[fld].lowlimit) mvar[fld] = m1[fld].uplimit;
       if (mvar[fld] > m1[fld].uplimit) mvar[fld] = m1[fld].lowlimit;
       DebugOut.print(fld); DebugOut.print(" : "); DebugOut.println(mvar[fld]);
       break;
    case IS_OPENING:
      butState = IS_OPEN;
      if ((millis()-lastPush)<500) {
          LastModify = millis();
          mvar[fld]++;
          if (mvar[fld] < m1[fld].lowlimit) mvar[fld] = m1[fld].uplimit;
          if (mvar[fld] > m1[fld].uplimit) mvar[fld] = m1[fld].lowlimit;
          //if (fld <= 2) mvar[2] = monthday_check(mvar[RTC_Ev], mvar[RTC_Ho], mvar[RTC_Nap]);    //szökőév ellenőrzés           
          DebugOut.print(fld); DebugOut.print(" : "); DebugOut.println(mvar[fld]);
          if (fld>5) fld = 0;
          //flash(fld);
       } //endif millis()
      break;
    case IS_LONGOPENING:
      butState = IS_OPEN;
      break;
  } //end switch
}



int I2C_ClearBus() {



  DebugOut.println("I2C_ClearBus() started.");
  pinMode(21 /* you can set the used SDA and SCL pins*/, 0x05); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(22 /* if it is not default value       */, 0x05);

  Fdelay(2500); // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(22 /* if it is not default value       */) == 0x0); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(21 /* you can set the used SDA and SCL pins*/) == 0x0); // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(22 /* if it is not default value       */, 0x01); // release SCL pullup so that when made output it will be LOW
    pinMode(22 /* if it is not default value       */, 0x03); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(22 /* if it is not default value       */, 0x01); // release SCL LOW
    pinMode(22 /* if it is not default value       */, 0x05); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(22 /* if it is not default value       */) == 0x0); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) { //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(22 /* if it is not default value       */) == 0x0);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(21 /* you can set the used SDA and SCL pins*/) == 0x0); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(21 /* you can set the used SDA and SCL pins*/, 0x01); // remove pullup.
  pinMode(21 /* you can set the used SDA and SCL pins*/, 0x03); // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(21 /* you can set the used SDA and SCL pins*/, 0x01); // remove output low
  pinMode(21 /* you can set the used SDA and SCL pins*/, 0x05); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(21 /* you can set the used SDA and SCL pins*/, 0x01); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(22 /* if it is not default value       */, 0x01);
  return 0; // all ok
}




//------------------------------------------------------------------------------------------
# 1 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
// NeoPixel Tube Backlight



# 6 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino" 2

long neoBrightness;
int colorStep = 1;
# 18 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
RgbColor red(255, 0, 0);
RgbColor red2(255/2, 0, 0);
RgbColor green(0, 255, 0);
RgbColor blue(0, 0, 255);
RgbColor purple(255, 0, 255);
RgbColor white(255/2,255/2,255/2);
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


  byte PixelPin = 25; //on ESP32 usable any pin below 32 
  NeoPixelBrightnessBus<NeoGrbFeature, NeoEsp32Rmt7Ws2812xMethod> strip(StripPixelCount,PixelPin);






NeoGamma<NeoGammaTableMethod> colorGamma;

volatile uint16_t neoAppliedCurrentmA = 0;
volatile uint8_t neoAppliedBrightness = 0;
static const bool NEO_DOUBLE_SHOW = false;

static inline void showStableFrame(bool forceDoubleFrame = false) {
  strip.Show();

  if (forceDoubleFrame || NEO_DOUBLE_SHOW) {
    delayMicroseconds(300);
    strip.Show();
  }

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
    DebugOut.println("Setup NeoPixel LEDS");
    regPin(PixelPin,"NEOPIXEL_PIN");
    DebugOut.print("Pixel count: "); DebugOut.println(PixelCount);
    DebugOut.print("Brightness:"); DebugOut.print(c_MinBrightness); DebugOut.print(" - "); DebugOut.println(c_MaxBrightness);
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

    if (WheelPos == 192) {
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
  } while (c == 192);
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
        if (index == 192) index++; // Skip white
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

# 280 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino" 2


# 281 "c:\\Users\\Dellus\\Documents\\Arduino\\ClockForgeOS\\z_neopixel.ino"
void effect1() { // Ultra-Smooth Color Dimmer with Stability Fix
    static float brightness = 0.0; // Current brightness (0.0 to 1.0)
    static float brightnessStep = 0.01; // Step size for brightness changes
    static float colorBlendRatio = 0.0; // Ratio for blending between two colors
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

void effect2() { //Color FlowChanger (smooth full-strip transition)
  static bool firstRun = true;
  static uint8_t currWheel = 0;
  static uint8_t nextWheel = 0;
  static uint8_t blend = 0; //0..255
  static unsigned long lastRun = 0;

  const uint8_t blendStep = (uint8_t)max(1, prm.rgbSpeed / 18); //speed-scaled transition step
  const unsigned long frameDelay = (unsigned long)max(8, 120 - (prm.rgbSpeed / 2));

  if ((millis() - lastRun) < frameDelay) return;
  lastRun = millis();

  if (firstRun) {
    firstRun = false;
    currWheel = randomWheelNoWhite();
    nextWheel = randomWheelNoWhite();
    while (colorDistance(currWheel, nextWheel) < 30 /*how far colors will get in random mode*/) {
      nextWheel = randomWheelNoWhite();
    }
    blend = 0;
  }

  if (blend >= 255) {
    currWheel = nextWheel;
    nextWheel = randomWheelNoWhite();
    while (colorDistance(currWheel, nextWheel) < 30 /*how far colors will get in random mode*/) {
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
  static const int cMax = sizeof(c) / sizeof(c[0]); //size of array
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

  if (newColor[i] == actColor[i]) { //newColor reached... 
    if (prm.rgbDir) i++; else i--; //goto next pixel

    if (i>=maxDigits) { i=0; changeColor = true;}
    else if (i<0) {i=maxDigits-1; changeColor = true;}

    if (eachPixelRandom) { //each pixel is random color
      changeColor = true;
    }

    if (changeColor) {
      changeColor = false;
      if (enableRandom) {
        counter = 0;
        while (true) { //random color
          newC = randomWheelNoWhite(); //get a new random color (exclude white)
          colorOK = true;
          if (true /*true or false: when generating new colors, the distance must be calculated from all pixels or only from the actual pixel's color */) {
            for (int j=0;j<maxDigits;j++) {
              if (colorDistance(newC,newColor[j]) < 30 /*how far colors will get in random mode*/)
                colorOK = false; //here the oldColor is just stored in the newColor... :)
            }
          }
          else {
            if (colorDistance(newC,newColor[i]) < 30 /*how far colors will get in random mode*/) //check random only from actual pixel
                colorOK = false; //here the oldColor is just 
          }
          counter++;
          if (colorOK || (counter>100 /*maximum how many times try to found a new color*/)) break;
        } //end while 
        //DPRINT("Pix:"); DPRINT(i); DPRINT("  old:"); DPRINT(newColor[i]); DPRINT("  / newC:"); DPRINT(newC); DPRINT("  No of tries:"); DPRINTLN(counter);
      }
      else { //new color from table
        idx++; if (idx>=cMax) idx = 0;
        newC = c[idx];
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

    actColor[i] = oldColor[i]; //starting color} 
    } //endif (newColor[i] == actColor[i])


  if (actColor[i] != newColor[i]) { //next rainbow color
    if (dir==1) actColor[i] = min(newColor[i],actColor[i]+step);
    else actColor[i] = max(newColor[i],actColor[i]-step);
   }

  for (int j=0;j<maxDigits;j++)
    setPixels(j, Wheel(actColor[j]));

  applyCurrentLimitAndShow(neoBrightness);
//DPRINT("Pix:"); DPRINT(i); DPRINT(" Old:"); DPRINT(oldColor[i]); DPRINT(" ActCol:"); DPRINT(actColor[i]); DPRINT(" New:"); DPRINT(newColor[i]); DPRINT("  Step:"); DPRINTLN(step);
}


void effect4() { //every pixel is random changer
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

  if (actColor[t] == newColor[t]) { //change color
    oldColor[t] = newColor[t];
    do {
      newColor[t] = randomWheelNoWhite(); //get a new random color (exclude white)
    } while (colorDistance(newColor[t],oldColor[t])<30 /*how far colors will get in random mode*/);

    actColor[t] = oldColor[t]; //starting color} 
    step[t] = 1; //max(1,colorDistance(newColor[t],oldColor[t])/20);
  } //endif changeColor

  //DPRINT(i); DPRINT("/"); DPRINTLN(j);    
  setPixels(t, Wheel(actColor[t]));

  if (newColor[t] != actColor[t]) { //next rainbow color
    if (oldColor[t] < newColor[t]) actColor[t] = min(newColor[t],actColor[t]+step[t]);
    else actColor[t] = max(newColor[t],actColor[t]-step[t]);

   }
     //DPRINT("Pix:"); DPRINT(i); DPRINT(" ActCol:"); DPRINTLN(actColor); 
  } //end for t
  applyCurrentLimitAndShow(neoBrightness);
}

void setPixels(byte tubeNo, RgbColor c) {
  for (volatile int i=0;i<PixelCount;i++) {
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
  static int dir = 1; //direction
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
    static float brightness = 0.0; // Current brightness (0.0 to 1.0)
    static float brightnessStep = 0.0; // Current step size for brightness changes
    static int phase = 0; // Phase of the heartbeat cycle
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
    brightness = ((brightness)<(0.0)?(0.0):((brightness)>(1.0)?(1.0):(brightness)));

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
    if ((prm.rgbEffect <=1) && ((millis()-lastRun)<1000)) return; //fix color
    if ((millis()-lastRun)<max(1000/60 /*frame per sec*/,258-prm.rgbSpeed)) return;
  }
  lastRun = millis();

  if ((prm.rgbEffect == 0) || !displayON || !radarON) { //switch RGB backlight OFF
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
    neoBrightness = (neoBrightness * lx) / long(100);
    if (neoBrightness< c_MinBrightness)
            neoBrightness = c_MinBrightness;
  }
  neoBrightness = min(neoBrightness,long(255 /*Neopixel leds maximum brightness*/)); //safety only
  strip.SetBrightness(neoBrightness);
  //DPRINTLN("  NeoBrightness:"); DPRINT(neoBrightness);

  if (prm.rgbEffect==1) fixColor();
  else if (prm.rgbEffect==2) rainbow(); //flow
  else if (prm.rgbEffect==3) rainbow2(); //stepper
  else if (prm.rgbEffect==4) effect1(); //color dimmer
  else if (prm.rgbEffect==5) effect2(); //color stepper
  else if (prm.rgbEffect==6) effect3(false,false); //color stepflow table
  else if (prm.rgbEffect==7) effect3(true,false); //color stepflow random
  else if (prm.rgbEffect==8) effect3(true,true); //color stepflow each pixel random
  else if (prm.rgbEffect==9) effect4(); //color stepflow random all pixels
  else if (prm.rgbEffect==10) kitt(); //Knight Rider's KITT car
  else if (prm.rgbEffect==11) heartbeat(); //HeartBeat - Dr. Pricopi
  else fixColor(); //darken leds, if any error happens
}
