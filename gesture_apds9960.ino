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
