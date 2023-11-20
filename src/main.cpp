#include <Arduino.h>
#define INO

#define DECODE_NEC
//#define DEBUG               // Activate this for lots of lovely debug output from the decoders.

#include <IRremote.hpp>
#include <TimerMs.h>

#ifdef INO
const uint8_t redPin = 5;
const uint8_t greenPin = 6;
const uint8_t bluePin = 9;
const uint8_t irPin = 2;
#else
const uint8_t redPin = 17;
const uint8_t greenPin = 18;
const uint8_t bluePin = 19;
const uint8_t irPin = 34;
#endif

#ifndef INO
const uint freq = 1000;
const uint8_t resolution = 8;
#endif

const float dK = 0.01; // step for transitions of dimming coefficients
const uint8_t dV = 2; // step for transitions of integer values like r,g,b components of light

const uint8_t channelStep = 20; // step for separate r,g,b channels manually (6 row) 
const float dimmingStep = 0.1; // step for dimming (0 row)
const uint8_t minimalDimming = 2; // step for transitions of integer values like r,g,b components of light

const uint16_t updateInterval = 5;
const uint16_t commandInterval = 150;

void writeRGB(uint8_t r, uint8_t g, uint8_t b) {
  #ifdef INO
  analogWrite(redPin, r);
  analogWrite(greenPin, g);
  analogWrite(bluePin, b);
  #else
  ledcWrite(0, r);
  ledcWrite(1, g);
  ledcWrite(2, b);
  #endif

  // #ifdef DEBUG
  // Serial.print(r);
  // Serial.print(g);
  // Serial.println(b);
  // #endif
}

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    IrReceiver.begin(irPin, 13);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);

#ifdef INO
    pinMode(redPin, 1);
    pinMode(greenPin, 1);
    pinMode(bluePin, 1);
#else
    ledcSetup(0, freq, resolution);
    ledcSetup(1, freq, resolution);
    ledcSetup(2, freq, resolution);

    ledcAttachPin(redPin, 0);
    ledcAttachPin(greenPin, 1);
    ledcAttachPin(bluePin, 2);
#endif
    writeRGB(255,240,190);
}

void loop() {
  static TimerMs update(updateInterval, 1, 0);

  static int redValue = 120;
  static int greenValue = 120;
  static int blueValue = 120;
  
  static int redValueT = 120;
  static int greenValueT = 120;
  static int blueValueT = 120;

  static float dimmingK = 1;
  static float targetDimmingK = 1;

  static bool turnedOn = true;
  static float turnedOnDimmingK = 1;

  static long previousCommandTime = millis();

  if(update.tick())
  {
    float dimmingDiff = dimmingK - targetDimmingK;

    if(dimmingDiff >= dK) dimmingK -= dK;
    else if(dimmingDiff <= dK && dimmingDiff > 0) dimmingK = targetDimmingK;
    else if(dimmingDiff <= -dK) dimmingK += dK;
    else if(dimmingDiff >= -dK && dimmingDiff < 0) dimmingK = targetDimmingK;


    float turnedOnDimmingDiff = turnedOnDimmingK - turnedOn;

    if(turnedOnDimmingDiff >= dK) turnedOnDimmingK -= dK;
    else if(turnedOnDimmingDiff <= dK && turnedOnDimmingDiff > 0) turnedOnDimmingK = turnedOn;
    else if(turnedOnDimmingDiff <= -dK) turnedOnDimmingK += dK;
    else if(turnedOnDimmingDiff >= -dK && turnedOnDimmingDiff < 0) turnedOnDimmingK = turnedOn;


    int colorDiff = redValue - redValueT;
    if(colorDiff >= dV) redValue -= dV;
    else if(colorDiff <= dV && colorDiff > 0) redValue = redValueT;
    else if(colorDiff <= -dV) redValue += dV;
    else if(colorDiff >= -dV && colorDiff < 0) redValue = redValueT;

    colorDiff = greenValue - greenValueT;
    if(colorDiff >= dV) greenValue -= dV;
    else if(colorDiff <= dV && colorDiff > 0) greenValue = greenValueT;
    else if(colorDiff <= -dV) greenValue += dV;
    else if(colorDiff >= -dV && colorDiff < 0) greenValue = greenValueT;

    colorDiff = blueValue - blueValueT;
    if(colorDiff >= dV) blueValue -= dV;
    else if(colorDiff <= dV && colorDiff > 0) blueValue = blueValueT;
    else if(colorDiff <= -dV) blueValue += dV;
    else if(colorDiff >= -dV && colorDiff < 0) blueValue = blueValueT;

    // Serial.print(redValue);
    // Serial.print("\t");
    // Serial.print(greenValue);
    // Serial.print("\t");
    // Serial.print(blueValue);
    // Serial.print("\t");
    
    // Serial.print(redValueT);
    // Serial.print("\t");
    // Serial.print(greenValueT);
    // Serial.print("\t");
    // Serial.print(blueValueT);
    // Serial.print("\n");

    writeRGB(
      redValue * dimmingK * turnedOnDimmingK,
      greenValue * dimmingK * turnedOnDimmingK,
      blueValue * dimmingK * turnedOnDimmingK
    );
  }

  if (IrReceiver.decode()) {
    //IrReceiver.printIRResultShort(&Serial);
    //IrReceiver.printIRSendUsage(&Serial);
    IrReceiver.resume(); // Enable receiving of the next value

    switch(IrReceiver.decodedIRData.command) {

      // 0 ROW
      case 0x5C: { // Up Value
        long currentTime = millis();
        if(currentTime - previousCommandTime > commandInterval) {
          if(targetDimmingK <= (1 - dimmingStep)) {
          targetDimmingK += dimmingStep;
          }
          previousCommandTime = currentTime;
        }
        break;
      }
      case 0x5D: { // Down Value
        long currentTime = millis();
        if(currentTime - previousCommandTime > commandInterval) {
          if(targetDimmingK >= 2 * dimmingStep) {
          targetDimmingK -= dimmingStep;
          }
          previousCommandTime = currentTime;
        }
        break;
      }
      case 0x41: { // Pause
        
        break;
      }
      case 0x40: { // OFF ON
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          turnedOn = !turnedOn;
          previousCommandTime = currentTime;
        }
        
        break;
      }

      // 1 ROW
      case 0x58: { // R
        redValueT = 255;
        greenValueT = 0;
        blueValueT = 0;
        break;
      }
      case 0x59: { // G
        redValueT = 0;
        greenValueT = 255;
        blueValueT = 0;
        break;
      }
      case 0x45: { // B
        redValueT = 0;
        greenValueT = 0;
        blueValueT = 255;
        break;
      }
      case 0x44: { // W
        redValueT = 255;
        greenValueT = 240;
        blueValueT = 190;
        break;
      }

      // 2 ROW
      case 0x54: { // Orange
        redValueT = 230;
        greenValueT = 20;
        blueValueT = 0;
        break;
      }
      case 0x55: { // G
        redValueT = 0;
        greenValueT = 220;
        blueValueT = 20;
        break;
      }
      case 0x49: { // B
        redValueT = 15;
        greenValueT = 15;
        blueValueT = 200;
        break;
      }
      case 0x48: { // pink
        redValueT = 255;
        greenValueT = 80;
        blueValueT = 110;
        break;
      }

      // 3 ROW
      case 0x50: { // Orange
        redValueT = 210;
        greenValueT = 45;
        blueValueT = 0;
        break;
      }
      case 0x51: { // Cyan1
        redValueT = 0;
        greenValueT = 200;
        blueValueT = 170;
        break;
      }
      case 0x4D: { // Purple
        redValueT = 200;
        greenValueT = 0;
        blueValueT = 200;
        break;
      }
      case 0x4C: { // pink
        redValueT = 255;
        greenValueT = 80;
        blueValueT = 110;
        break;
      }


      // 6 ROW
      case 0x14: { // Red up
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          redValueT += channelStep;
          redValueT = constrain(redValueT, 0, 255);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x15: { // Green up
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          greenValueT += channelStep;
          greenValueT = constrain(greenValueT, 0, 255);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x16: { // Blue up
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          blueValueT += channelStep;
          blueValueT = constrain(blueValueT, 0, 255);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x17: { // "Quick"

        break;
      }

      // 7 ROW
      case 0x10: { // Red down
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          redValueT -= channelStep;
          redValueT = constrain(redValueT, 0, 255);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x11: { // Green down
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          greenValueT -= channelStep;
          greenValueT = constrain(greenValueT, 0, 255);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x12: { // Blue down
        long currentTime = millis();

        if(currentTime - previousCommandTime > commandInterval) {
          blueValueT -= channelStep;
          blueValueT = constrain(blueValueT, 0, 255);
          previousCommandTime = currentTime;
        }
        
        break;
      }
      case 0x13: { // "Slow"

        break;
      }
    }
  }
}