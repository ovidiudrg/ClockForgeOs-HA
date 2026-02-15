#ifdef USE_MQTT
//Clock can receive data from a MASTER_CLOCK and send data to other clocks
//_______________________________________________________________________________________________________________________
//To use MQTT services the following settings should be defined in clocks.h
//#define USE_MQTT
//#define MQTT_PREFIX "UniClock32"
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
  #define MQTT_PREFIX "UniClock32"
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
    
    IPAddress tmp;
    tmp.fromString(String(prm.mqttBrokerAddr));
    if (strlen(prm.mqttBrokerUser)==0)
      mqtt.begin(tmp);
    else  
      mqtt.begin(tmp, prm.mqttBrokerUser, prm.mqttBrokerPsw);
      
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
