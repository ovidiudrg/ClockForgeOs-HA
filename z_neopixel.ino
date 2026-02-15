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
