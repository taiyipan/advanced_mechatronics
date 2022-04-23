#include "SevSeg.h"
#include "SerialTransfer.h"

SevSeg sevseg;
SerialTransfer myTransfer;
int num;

void setup(){
  Serial.begin(115200);
  myTransfer.begin(Serial);

  byte numDigits = 1;
  byte digitPins[] = {};
  byte segmentPins[] = {6, 5, 2, 3, 4, 7, 8, 9};
  bool resistorsOnSegments = true;

  byte hardwareConfig = COMMON_CATHODE;
  sevseg.begin(hardwareConfig, numDigits, digitPins, segmentPins, resistorsOnSegments);
  sevseg.setBrightness(90);

  num = 0;
}

void loop(){
  sevseg.setNumber(num);
  sevseg.refreshDisplay();

  if (myTransfer.available()) {
    myTransfer.rxObj(num);
  }
}
