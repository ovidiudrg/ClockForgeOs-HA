#ifdef USE_PWMLEDS

#define FPS 60 //frame per sec
#define FPS_MSEC 1000/FPS
#define COLORSATURATION 255
#define WHITE_INDEX -2

extern bool tubesPowerState;
extern bool startupMuteRgb;
extern Settings settings;

long pwmBrightness;
int pwmColorStep = 1;
int pwmRed,pwmGreen,pwmBlue = 0;
static bool pwmDetached = false;

static inline void forcePwmPinsLowAndDetach() {
  if (pwmDetached) return;
  #if PWM1_PIN>=0
    ledcDetachPin(PWM1_PIN);
    pinMode(PWM1_PIN, OUTPUT);
    digitalWrite(PWM1_PIN, LOW);
  #endif
  #if PWM2_PIN>=0
    ledcDetachPin(PWM2_PIN);
    pinMode(PWM2_PIN, OUTPUT);
    digitalWrite(PWM2_PIN, LOW);
  #endif
  #if PWM3_PIN>=0
    ledcDetachPin(PWM3_PIN);
    pinMode(PWM3_PIN, OUTPUT);
    digitalWrite(PWM3_PIN, LOW);
  #endif
  pwmDetached = true;
}

static inline void reattachPwmPinsIfNeeded() {
  if (!pwmDetached) return;
  #ifdef PWM_FREQ
    const uint32_t pwmFreq = PWM_FREQ;
  #else
    const uint32_t pwmFreq = 200;
  #endif
  #if PWM1_PIN>=0
    ledcSetup(0, pwmFreq, 8);
    ledcAttachPin(PWM1_PIN, 0);
    ledcWrite(0, 0);
  #endif
  #if PWM2_PIN>=0
    ledcSetup(1, pwmFreq, 8);
    ledcAttachPin(PWM2_PIN, 1);
    ledcWrite(1, 0);
  #endif
  #if PWM3_PIN>=0
    ledcSetup(2, pwmFreq, 8);
    ledcAttachPin(PWM3_PIN, 2);
    ledcWrite(2, 0);
  #endif
  pwmDetached = false;
}

static inline void setPWMrgb(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  const uint16_t dr = ((uint16_t)r * brightness) / 255;
  const uint16_t dg = ((uint16_t)g * brightness) / 255;
  const uint16_t db = ((uint16_t)b * brightness) / 255;
  static uint16_t pr = 0, pg = 0, pb = 0;

  // Write channels that are decreasing first, then increasing channels.
  // This avoids short all-high transients that can appear as white flashes.
  #if PWM1_PIN>=0
    if (dr < pr) ledcWrite(0, dr);
  #endif
  #if PWM2_PIN>=0
    if (dg < pg) ledcWrite(1, dg);
  #endif
  #if PWM3_PIN>=0
    if (db < pb) ledcWrite(2, db);
  #endif

  #if PWM1_PIN>=0
    if (dr >= pr) ledcWrite(0, dr);
  #endif
  #if PWM2_PIN>=0
    if (dg >= pg) ledcWrite(1, dg);
  #endif
  #if PWM3_PIN>=0
    if (db >= pb) ledcWrite(2, db);
  #endif

  pr = dr; pg = dg; pb = db;
}

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

  setPWMrgb((uint8_t)pwmRed, (uint8_t)pwmGreen, (uint8_t)pwmBlue, brightness);
#endif  
}


void pwmRainbow() {
 static byte j=0;
   
  setPWMcolor(j,pwmBrightness);
  if (prm.rgbDir) { j++; }
  else { j--; }
}

void pwmRainbowStep() {
  static byte j = 0;
  byte step = (byte)max(2, pwmColorStep);
  setPWMcolor(j, (byte)pwmBrightness);
  if (prm.rgbDir) j = (byte)(j + step);
  else j = (byte)(j - step);
}

void pwmBreathFixed() {
  static int level = 0;
  static int dir = 1;
  level += dir * max(1, pwmColorStep / 2);
  if (level >= 255) { level = 255; dir = -1; }
  if (level <= 4)   { level = 4;   dir = 1;  }
  uint8_t eff = (uint8_t)(((uint16_t)pwmBrightness * (uint16_t)level) / 255U);
  setPWMrgb(settings.rgbFixR, settings.rgbFixG, settings.rgbFixB, eff);
}

void pwmStrobeFixed() {
  static bool on = false;
  on = !on;
  if (on) setPWMrgb(settings.rgbFixR, settings.rgbFixG, settings.rgbFixB, (uint8_t)pwmBrightness);
  else setPWMrgb(0, 0, 0, 0);
}

void pwmRandomFade() {
  static uint8_t cr = 255, cg = 120, cb = 20;
  static uint8_t tr = 255, tg = 120, tb = 20;
  static unsigned long nextTargetMs = 0;
  const unsigned long now = millis();
  if (now >= nextTargetMs) {
    tr = (uint8_t)random(0, 256);
    tg = (uint8_t)random(0, 256);
    tb = (uint8_t)random(0, 256);
    nextTargetMs = now + (unsigned long)max(300, 2200 - (int)prm.rgbSpeed * 6);
  }
  uint8_t step = (uint8_t)max(1, pwmColorStep / 2);
  if (cr < tr) cr = (uint8_t)min(255, cr + step); else if (cr > tr) cr = (uint8_t)max(0, cr - step);
  if (cg < tg) cg = (uint8_t)min(255, cg + step); else if (cg > tg) cg = (uint8_t)max(0, cg - step);
  if (cb < tb) cb = (uint8_t)min(255, cb + step); else if (cb > tb) cb = (uint8_t)max(0, cb - step);
  setPWMrgb(cr, cg, cb, (uint8_t)pwmBrightness);
}

void pwmKitt() {
  static uint8_t idx = 0;
  static int8_t dir = 1;
  uint8_t r = 0, g = 0, b = 0;
  if (idx == 0) r = 255;
  else if (idx == 1) g = 255;
  else b = 255;
  setPWMrgb(r, g, b, (uint8_t)pwmBrightness);
  idx = (uint8_t)(idx + dir);
  if (idx >= 2) dir = -1;
  if (idx == 0) dir = 1;
}

void pwmHeartbeat() {
  static unsigned long cycleStart = 0;
  const unsigned long now = millis();
  if (cycleStart == 0) cycleStart = now;
  const unsigned long t = (now - cycleStart) % 1400UL;
  uint8_t env = 0;
  if (t < 90) env = (uint8_t)((t * 255UL) / 90UL);
  else if (t < 180) env = (uint8_t)(((180UL - t) * 255UL) / 90UL);
  else if (t >= 260 && t < 350) env = (uint8_t)(((t - 260UL) * 255UL) / 90UL);
  else if (t >= 350 && t < 440) env = (uint8_t)(((440UL - t) * 255UL) / 90UL);
  uint8_t eff = (uint8_t)(((uint16_t)pwmBrightness * (uint16_t)env) / 255U);
  setPWMrgb(settings.rgbFixR, settings.rgbFixG, settings.rgbFixB, eff);
}

void pwmCandle() {
  uint8_t base = (uint8_t)max(20L, pwmBrightness);
  uint8_t flick = (uint8_t)random(0, max(4, pwmColorStep * 3));
  uint8_t br = (uint8_t)max(8, (int)base - (int)flick);
  setPWMrgb(255, 90, 15, br);
}

void pwmAutoCycle() {
  static uint8_t idx = 0;
  static const uint8_t p[][3] = {
    {255, 0, 0}, {255, 140, 0}, {255, 255, 0}, {0, 255, 0},
    {0, 255, 255}, {0, 0, 255}, {180, 0, 255}
  };
  setPWMrgb(p[idx][0], p[idx][1], p[idx][2], (uint8_t)pwmBrightness);
  if (prm.rgbDir) idx = (uint8_t)((idx + 1) % 7);
  else idx = (uint8_t)((idx + 6) % 7);
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
  if (startupMuteRgb) {
    pwmBrightness = 0;
    forcePwmPinsLowAndDetach();
    lastRun = millis();
    return;
  }
#ifdef CLOCK_54
  if ((prm.rgbEffect == 0) || !displayON || !tubesPowerState) {   //switch RGB backlight OFF
#else
  if ((prm.rgbEffect == 0) || !displayON || !radarON || !tubesPowerState) {   //switch RGB backlight OFF
#endif
    pwmBrightness = 0;
    forcePwmPinsLowAndDetach();
    lastRun = millis();
    return;
  }

  reattachPwmPinsIfNeeded();

  if (alarmON) {
    pwmAlarmLight();
    return;
  }
  
  if ((prm.rgbEffect <=1) && ((millis()-lastRun)<1000)) return;  //fix color
  if (prm.rgbEffect == 11 && ((millis()-lastRun)<400)) return;   //auto-cycle
  if (prm.rgbEffect == 5 && ((millis()-lastRun)<140)) return;    //strobe
  if ((millis()-lastRun)<max(FPS_MSEC,258-prm.rgbSpeed)) return;
  lastRun = millis();

  pwmColorStep = max(1,prm.rgbSpeed/5);
  pwmBrightness = prm.rgbBrightness;
  if (autoBrightness) {
    pwmBrightness = (pwmBrightness * lx) /(long) MAXIMUM_LUX/2;
    if (pwmBrightness< c_MinBrightness) 
            pwmBrightness = c_MinBrightness;
  }
  pwmBrightness = min(pwmBrightness,long(RGB_MAX_BRIGHTNESS));  //safety only

  //DPRINTLN("  NeoBrightness:"); DPRINT(neoBrightness);
  
  if (prm.rgbEffect == 1) setPWMrgb(settings.rgbFixR, settings.rgbFixG, settings.rgbFixB, (uint8_t)pwmBrightness); // fixed
  else if (prm.rgbEffect == 2) pwmRainbow();      // smooth rainbow
  else if (prm.rgbEffect == 3) pwmRainbowStep();  // stepped rainbow
  else if (prm.rgbEffect == 4) pwmBreathFixed();  // breathing fixed
  else if (prm.rgbEffect == 5) pwmStrobeFixed();  // strobe fixed
  else if (prm.rgbEffect == 6) pwmRandomFade();   // random fade
  else if (prm.rgbEffect == 7) pwmKitt();         // R/G/B scan
  else if (prm.rgbEffect == 8) pwmHeartbeat();    // heartbeat fixed
  else if (prm.rgbEffect == 9) pwmCandle();       // candle
  else if (prm.rgbEffect == 10) setPWMrgb(settings.rgbFixR, settings.rgbFixG, settings.rgbFixB, (uint8_t)pwmBrightness); // disabled by request
  else if (prm.rgbEffect == 11) pwmAutoCycle();   // palette cycle
  else pwmRainbow();
}

#else
void doAnimationPWM() {}
#endif
