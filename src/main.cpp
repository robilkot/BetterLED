#define DECODE_NEC

#include <Arduino.h>
#include <IRremote.hpp>
#include <TimerMs.h>

#define INO
//#define DEBUG

#ifdef INO
const uint8_t redPin = 5;
const uint8_t greenPin = 6;
const uint8_t bluePin = 9;
const uint8_t irPin = 2;

const uint8_t micPin = A0; 
const uint8_t potentiometerPin = A1;
#else
const uint8_t redPin = 17;
const uint8_t greenPin = 18;
const uint8_t bluePin = 19;
const uint8_t irPin = 34;

const uint freq = 1000; // PWM frequency
const uint8_t resolution = 8; // DAC resolution
#endif

const float dK = 0.01; // step for transitions of dimming coefficients
const uint8_t dV = 2; // step for transitions of integer values like r,g,b components of light

const uint8_t channelStep = 20; // step for separate r,g,b channels manually (6 row) 
const float dimmingStep = 0.1; // step for dimming (0 row)

const uint16_t defaultUpdateInterval = 5;
const uint16_t automaticUpdateIntervalQuick = 20;
const uint16_t automaticUpdateIntervalSlow = 100;
const uint16_t musicUpdateInterval = 50;
const uint16_t commandInterval = 150;

const uint16_t AmpMax = 512;
const uint16_t MicSamples = 100;
// Sensitivity of microphone to amplify volume at processing stage. (0..)
float Sensitivity = 3.1f;
// Minimum brightness of LEDs to prevent fading to black. (0..255)
const uint8_t MinBrightness = 10;
// Value for brightness to fade per update (0..255)
const uint8_t BrightnessFading = 4;
// Minimum hue changing speed.
const uint8_t HueBaseDerivative = 0;
// Volume amount needed for additional incrementation of hue. (0..255)
const uint8_t VolumeToHueRatio = 15;


struct RGBcolor {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
  
  RGBcolor(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {};
  RGBcolor() {};

  RGBcolor operator*(float k) const {
    k = constrain(k, 0, 1);
    return { r * k, g * k, b * k };
  }
};

void Increment(uint8_t& value, uint8_t increment)
{
  if(255 - value >= increment)
  {
    value += increment;
  }
  else
  {
    value = 255;
  }
}

void Decrement(uint8_t& value, uint8_t increment)
{
  if(value >= increment)
  {
    value -= increment;
  }
  else
  {
    value = 0;
  }
}

struct ColorState {
  RGBcolor color;
  RGBcolor target;
};

struct DimmingState { 
  float coef = 1;
  float coefT = 1;

  bool turnedOnT = false;
  float turnedOn = 0;
};

RGBcolor HueToRGB(uint8_t hue) 
{
  RGBcolor result;

  hue <= 42 ?
    result = {255, 255 * hue / 42., 0} :
  hue > 42 && hue <= 85 ?
    result = {255 * (43 - (hue - 42)) / 43., 255, 0} :
  hue > 85 && hue <= 127 ?
    result = {0, 255, 255 * (hue - 85) / 42.} :
  hue > 127 && hue <= 170 ?
    result = {0, 255 * (43 - (hue - 127)) / 43., 255} :
  hue > 170 && hue <= 212 ?
    result = {255 * (hue - 170) / 42., 0, 255} :
  /*hue > 212 ?*/
    result = {255, 0, 255 * (43 - (hue - 212)) / 43.};

  return result;
}

int MeasureVolume()
{
  long soundVolRMS = 0;

	for (uint16_t i = 0; i < MicSamples; i++)
	{
		int k = analogRead(micPin);
		int amp = abs(k - AmpMax);
		soundVolRMS += ((long)amp*amp);
	}
	soundVolRMS /= MicSamples;
	float soundVolRMSflt = sqrt(soundVolRMS);

  // RMS to estimate peak (RMS is 0.7 of the peak in sin)
	soundVolRMS = 10 * soundVolRMSflt / 7;

  return soundVolRMS;
}

void writeRGB(const RGBcolor& color, const DimmingState& dimmingState)
{
  #ifdef INO
  analogWrite(redPin, color.r * dimmingState.coef * dimmingState.turnedOn);
  analogWrite(greenPin, color.g * dimmingState.coef * dimmingState.turnedOn);
  analogWrite(bluePin, color.b * dimmingState.coef * dimmingState.turnedOn);
  #else
  ledcWrite(0, r);
  ledcWrite(1, g);
  ledcWrite(2, b);
  #endif
}

void updateDimmingStateNormal(DimmingState& dimmingState)
{
  float dimmingDiff = dimmingState.coef - dimmingState.coefT;

  if(dimmingDiff >= dK) dimmingState.coef -= dK;
  else if(dimmingDiff <= dK && dimmingDiff > 0) dimmingState.coef = dimmingState.coefT;
  else if(dimmingDiff <= -dK) dimmingState.coef += dK;
  else if(dimmingDiff >= -dK && dimmingDiff < 0) dimmingState.coef = dimmingState.coefT;

  float turnedOnDimmingDiff = dimmingState.turnedOn - dimmingState.turnedOnT;

  if(turnedOnDimmingDiff >= dK) dimmingState.turnedOn -= dK;
  else if(turnedOnDimmingDiff <= dK && turnedOnDimmingDiff > 0) dimmingState.turnedOn = dimmingState.turnedOnT;
  else if(turnedOnDimmingDiff <= -dK) dimmingState.turnedOn += dK;
  else if(turnedOnDimmingDiff >= -dK && turnedOnDimmingDiff < 0) dimmingState.turnedOn = dimmingState.turnedOnT;
}

void updateColorStateNormal(ColorState& colorState)
{
  int colorDiff = colorState.color.r - colorState.target.r;
  if(colorDiff > 0)
    Decrement(colorState.color.r, dV);
  else if(colorDiff < 0)
    Increment(colorState.color.r, dV);

  colorDiff = colorState.color.g - colorState.target.g;
  if(colorDiff > 0)
    Decrement(colorState.color.g, dV);
  else if(colorDiff < 0)
    Increment(colorState.color.g, dV);

  colorDiff = colorState.color.b - colorState.target.b;
  if(colorDiff > 0)
    Decrement(colorState.color.b, dV);
  else if(colorDiff < 0)
    Increment(colorState.color.b, dV);
}

void updateColorStateRainbow(ColorState& colorState)
{
  static uint8_t hue = 0;
  colorState.color = HueToRGB(hue++);
}

void updateColorStateMusic(ColorState& colorState)
{
  // static HSVcolor hsvColor = {0, 0, 0};
  static int volume = 0;
  static uint8_t hue = 0;
  static uint8_t value = MinBrightness;
  
  Sensitivity = analogRead(potentiometerPin) / 256;

  int newVolume = constrain(MeasureVolume() * Sensitivity, 0, 255);

  // Protects against fluctuations in volume measurement
  if(abs(newVolume - volume) > 4)
  {
    volume = newVolume;
    
    uint8_t newValue = (255 - MinBrightness) / 255. * volume + MinBrightness;
    value = max(newValue, value);
  }

  uint8_t dHue = constrain(HueBaseDerivative + volume / VolumeToHueRatio, 0, 255);
  hue += dHue;
  
  if(value >= BrightnessFading && value > (MinBrightness + BrightnessFading))
    value -= BrightnessFading;

  // hsvColor.s = 255 - hsvColor.v / 2;

  // RGBcolor converted = HSVToRGB(hsvColor);

  RGBcolor newColor = HueToRGB(hue) * (value / 255.);
  colorState.color = newColor;
}

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    IrReceiver.begin(irPin, 13);

#ifdef INO
    pinMode(redPin, 1);
    pinMode(greenPin, 1);
    pinMode(bluePin, 1);

    pinMode(micPin, 0);
    pinMode(potentiometerPin, 0);

    analogReference(EXTERNAL); // 3.3V to AREF
#else
    ledcSetup(0, freq, resolution);
    ledcSetup(1, freq, resolution);
    ledcSetup(2, freq, resolution);

    ledcAttachPin(redPin, 0);
    ledcAttachPin(greenPin, 1);
    ledcAttachPin(bluePin, 2);
#endif
}

void loop() {
  static TimerMs colorStateUpdateTimer(defaultUpdateInterval, 1, 0);
  static TimerMs ledStripUpdateTimer(defaultUpdateInterval, 1, 0);

  static unsigned long previousCommandTime = millis();

  static ColorState colorState = {{ 0, 0, 0 }, { 255, 140, 190 }};
  static DimmingState dimmingState;

  static void (*colorStateUpdater)(ColorState&) = &updateColorStateNormal; 
  static void (*dimmingStateUpdater)(DimmingState&) = &updateDimmingStateNormal; 

  if(colorStateUpdateTimer.tick())
  {
    (*colorStateUpdater)(colorState);
  }
  if(ledStripUpdateTimer.tick())
  {
    (*dimmingStateUpdater)(dimmingState);
    writeRGB(colorState.color, dimmingState);
  }

  if (IrReceiver.decode()) {
    //IrReceiver.printIRResultShort(&Serial);
    IrReceiver.resume();

    switch(IrReceiver.decodedIRData.command)
    {
      // 0 ROW
      case 0x5C: { // Up Value
        long currentTime = millis();
        if(currentTime - previousCommandTime > commandInterval) {
          if(dimmingState.coefT <= (1 - dimmingStep)) {
          dimmingState.coefT += dimmingStep;
          }
          previousCommandTime = currentTime;
        }
        break;
      }
      case 0x5D: { // Down Value
        long currentTime = millis();
        if(currentTime - previousCommandTime > commandInterval) {
          if(dimmingState.coefT >= 2 * dimmingStep) {
          dimmingState.coefT -= dimmingStep;
          }
          previousCommandTime = currentTime;
        }
        break;
      }
      case 0x41: { // Pause
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          if(colorStateUpdater == updateColorStateMusic) {
            colorStateUpdater = updateColorStateNormal;
            colorStateUpdateTimer.setTime(defaultUpdateInterval);
          } else {
            colorStateUpdater = updateColorStateMusic;
            colorStateUpdateTimer.setTime(musicUpdateInterval);
          }
        }
        
        break;
      }
      case 0x40: { // OFF ON
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          dimmingState.turnedOnT = !dimmingState.turnedOnT;
          previousCommandTime = currentTime;
        }
        
        break;
      }

      // 1 ROW
      case 0x58: { // R
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);
        
        colorState.target.r = 255;
        colorState.target.g = 0;
        colorState.target.b = 0;
        break;
      }
      case 0x59: { // G
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 0;
        colorState.target.g = 255;
        colorState.target.b = 0;
        break;
      }
      case 0x45: { // B
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 0;
        colorState.target.g = 0;
        colorState.target.b = 255;
        break;
      }
        case 0x44: { // W
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 255;
        colorState.target.g = 240;
        colorState.target.b = 190;
        break;
      }

      // 2 ROW
      case 0x54: { // Orange
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 230;
        colorState.target.g = 20;
        colorState.target.b = 0;
        break;
      }
      case 0x55: { // G
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 0;
        colorState.target.g = 220;
        colorState.target.b = 20;
        break;
      }
      case 0x49: { // B
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 15;
        colorState.target.g = 15;
        colorState.target.b = 200;
        break;
      }
      case 0x48: { // pink
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 255;
        colorState.target.g = 80;
        colorState.target.b = 110;
        break;
      }

      // 3 ROW
      case 0x50: { // Orange
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 210;
        colorState.target.g = 45;
        colorState.target.b = 0;
        break;
      }
      case 0x51: { // Cyan1
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 0;
        colorState.target.g = 200;
        colorState.target.b = 170;
        break;
      }
      case 0x4D: { // Purple
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 200;
        colorState.target.g = 0;
        colorState.target.b = 200;
        break;
      }
      case 0x4C: { // pink
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 255;
        colorState.target.g = 80;
        colorState.target.b = 110;
        break;
      }

      // 4 ROW
      case 0x1D: { // Cyan2
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 0;
        colorState.target.g = 180;
        colorState.target.b = 170;
        break;
      }

      // 5 ROW
        case 0x19: { // Cyan3
        colorStateUpdater = updateColorStateNormal;
        colorStateUpdateTimer.setTime(defaultUpdateInterval);

        colorState.target.r = 0;
        colorState.target.g = 190;
        colorState.target.b = 130;
        break;
      }

      // 6 ROW
      case 0x14: { // Red up
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          Increment(colorState.target.r, channelStep);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x15: { // Green up
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          Increment(colorState.target.g, channelStep);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x16: { // Blue up
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          Increment(colorState.target.b, channelStep);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x17: { // "Quick"
        colorStateUpdater = updateColorStateRainbow;
        colorStateUpdateTimer.setTime(automaticUpdateIntervalQuick);
        break;
      }

      // 7 ROW
      case 0x10: { // Red down
        long currentTime = millis();
        
        if(currentTime - previousCommandTime > commandInterval) {
          Decrement(colorState.target.r, channelStep);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x11: { // Green down
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          Decrement(colorState.target.g, channelStep);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x12: { // Blue down
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          Decrement(colorState.target.b, channelStep);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x13: { // "Slow"
        colorStateUpdater = updateColorStateRainbow;
        colorStateUpdateTimer.setTime(automaticUpdateIntervalSlow);
        break;
      }
    }
  }
}