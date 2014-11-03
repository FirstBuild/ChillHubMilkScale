#include <EEPROM.h>
#include "chillhub.h"

#define FULL_WEIGHT 60000
#define EMPTY_WEIGHT 0
#define DIFF_THRESHOLD 1200  // 2%

boolean doorWasOpen = false;
unsigned int LO_MEAS[3] = { 0 };
unsigned int HI_MEAS[3] = { 0 };

/* 
 * we find the full weight on each sensor by solving the statics problem:
 *
 * W1 + W2 + W3 = W
 * 2.374*W1 + 1.837*W2 - 2.374*W3 = 0  (sum Mx = 0)
 * 2.285*W1 - 2.323*W2 - 0.495*W3 = 0  (sum My = 0)
 *
 * This results in full W1 is 31.51%, W2 is 20.85%, and W3 is 47.64%
 * If we arbitrarily say we want the full weight to be 60000 (near limit of U16), then:
 */
unsigned int W_MAX[3] = {18908, 12507, 28585};
 
void setup() {
  // register the name (type) of this device with the chillhub
  ChillHub.setup("milkscale", 9);
  
  // load FSR limits here
  for (int j = 0; j < 3; j++) {
    int iStart = j*4;
    LO_MEAS[j] = (EEPROM.read(iStart+0) << 8) | EEPROM.read(iStart+1);
    HI_MEAS[j] = (EEPROM.read(iStart+2) << 8) | EEPROM.read(iStart+3);
  }
  
  // subscribe to door messages to trigger 
  ChillHub.subscribe(doorStatusMsgType, (chillhubCallbackFunction)readMilkWeight);
  
  // setup factory calibration listener
  ChillHub.addCloudListener(0x94, (chillhubCallbackFunction)factoryCalibrate);
}

void loop() {
  ChillHub.loop();
}

void readMilkWeight(unsigned char doorStatus) {
  boolean doorNowOpen = (doorStatus & 0x01);
  
  unsigned int sensorReadings[3], sensorWeights[3];
  unsigned int weight = 0;

  if (doorWasOpen && !doorNowOpen) {
    readFromSensors(sensorReadings);    
    applyFsrCurve(sensorWeights, sensorReadings);
    
    unsigned int weight = sensorWeights[0] + 
      sensorWeights[1] + 
      sensorWeights[2];
  
    if (weight >= (FULL_WEIGHT - DIFF_THRESHOLD)) {
      for (int j = 0; j < 3; j++)
        HI_MEAS[j] = sensorReadings[j];
      storeLimits();
    }
    else if (weight < (EMPTY_WEIGHT + DIFF_THRESHOLD)) {
      for (int j = 0; j < 3; j++)
        LO_MEAS[j] = sensorReadings[j];
      storeLimits();
    }
    
    ChillHub.sendU16Msg(0x51, weight/600);
  }
  doorWasOpen = doorNowOpen;
}

void applyFsrCurve(unsigned int scaledVal[3], unsigned int rawValue[3]) {
  for (int j = 0; j < 3; j++) {
    scaledVal[j] = (rawValue[j] - LO_MEAS[j]) * 1.0 * W_MAX[j] / (HI_MEAS[j] - LO_MEAS[j]);
  }
}

void readFromSensors(unsigned int meas[3]) {
  meas[0] = doSensorRead(A0);
  meas[1] = doSensorRead(A1);
  meas[2] = doSensorRead(A2);
}

unsigned int doSensorRead(unsigned char pinNumber) {
    unsigned int newReading = 0;
    unsigned int anaVal;
    // oversample to get the max amplitude
    for (int j = 0; j < 5000; j++) {
      anaVal = analogRead(pinNumber);
      newReading = max(newReading,anaVal);
    }
    return newReading;
}

void storeLimits() {
  for (int j = 0; j < 3; j++) {
    int iStart = j*4;
    EEPROM.write(iStart, (LO_MEAS[j] >> 8) & 0xff);
    EEPROM.write(iStart+1, LO_MEAS[j] & 0xff);
    EEPROM.write(iStart+2, (HI_MEAS[j] >> 8) & 0xff);
    EEPROM.write(iStart+3, HI_MEAS[j] & 0xff);
  }
}

void factoryCalibrate(boolean full) {
  unsigned int sensorReadings[3];
  readFromSensors(sensorReadings);
  
  if (full) {
    for (int j = 0; j < 3; j++)
      HI_MEAS[j] = sensorReadings[j];
  }
  else {
    for (int j = 0; j < 3; j++)
      LO_MEAS[j] = sensorReadings[j];
  }
  
  storeLimits();
}
