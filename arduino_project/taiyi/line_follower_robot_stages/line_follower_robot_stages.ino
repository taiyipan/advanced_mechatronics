#include <Servo.h>
#include <QTRSensors.h>

/*
 * This program controls a line follower robot.
 * The robot follow a track defined by a black line.
 * Able to dynamically traverse the track, and intelligently deal with intersections, deadends, and object obstacles.
*/

Servo servoLeft, servoRight, servoFront;

QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

//global constants
const int LEFT_SERVO_BASE_VALUE = 1480;
const int RIGHT_SERVO_BASE_VALUE = 1490;
const int FRONT_SERVO_BASE_VALUE = 1285;
const int piezoSpeakerPin = 4;
const int ledPin = 7;
const int pingPin = 10;
const int blackThreshold = 600; //define minimum threshold value to qualify as black line (high value means less sensitivity)[min: 0; max: 1000]
const int pingThreshold = 7; 

//stage protocols
int stage = 0;
// int objectMap[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// bool end = false;

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication:
  Serial.begin(9600);

  //configure piezospeaker and LED
  pinMode(piezoSpeakerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  beep();

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  beep();

  //configure servos
  servoLeft.attach(13);
  servoRight.attach(12);
  servoFront.attach(11);

  servoLeft.writeMicroseconds(LEFT_SERVO_BASE_VALUE);
  servoRight.writeMicroseconds(RIGHT_SERVO_BASE_VALUE);
  servoFront.writeMicroseconds(FRONT_SERVO_BASE_VALUE);
}

void loop() {
  // put your main code here, to run repeatedly:
  //default state: micro adjust and move forward a bit
  pingObject();
  senseLine();
  microAdjust();
  maneuver(50, 50, 50);
}

/*
 * Ping front area for object distance value
 */
long ping() {
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  // convert the time into a distance
  pinMode(pingPin, INPUT);
  long distance = microsecondsToCentimeters(pulseIn(pingPin, HIGH));
  Serial.println(distance);
  return distance;
}

/*
 * Ping front area for object distance value and execute action
 */
void pingObject() {
  //detecting close object (<10cm)
  if (ping() < pingThreshold) {
    alert();
    // if (stage > 0) end = true;
    turnLeft();
  }
  // else if (pingRemote()) alert();
}

/*
 * Ping remote area for object presence verification
 */
//bool pingRemote() {
//  if (ping() <= 20) return true;
//  else return false;
//}

/*
 * Read forward sensor array values
 */
void senseLine() {
  //define boolean flags for state detection
  bool allBlack = true;
  bool allWhite = true;

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
   for (uint8_t i = 0; i < SensorCount; i++)
   {
     Serial.print(sensorValues[i]);
     Serial.print('\t');
     if (sensorValues[i] > blackThreshold) {
       allWhite = false;
     }
     else allBlack = false;
   }
   Serial.println(position);

   //detecting deadend
   if (allWhite) {
     // if (stage > 0) end = true;
     turnLeft();
   }

   //detecting intersection
   else if (allBlack) {
     alert();
     maneuver(50, 50, 1000); //go forward a little
     //differentiate between X and Y intersections
     // if (blackLine() && !end) intersectionScan(); //X intersection before mapping completion
//     else if (blackLine() && end) {}; //X intersection after mapping completion
     if (blackLine()) intersectionScan(); //X intersection
     else turnLeft(); //Y intersection
     // turnLeft();
   }
}

void intersectionScan() {
  //full stop
  maneuver(0, 0, 50);
  //stage 0: first time encountering a X intersection, skip
  if (stage == 0) {
    stage++;
    return;
  }

  // //check front remote
  // if (pingRemote()) alert();
  //
  // //check left remote
  // turnLeft();
  // if (pingRemote()) {
  //   objectMap[stage * 2 - 2] = 1;
  //   alert();
  // }
  //
  // //check right remote
  // turnRight();
  // turnRight();
  // if (pingRemote()) {
  //   objectMap[stage * 2 - 1] = 1;
  // }

  //face forward
  turnLeft();
  stage++;
}

/*
 * Read forward sensor array values to detech presence of black line
 */
bool blackLine() {
  //define function return boolean (whether black line is seen or not)
  bool seen = false;

  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
   for (uint8_t i = 0; i < SensorCount; i++)
   {
     if (sensorValues[i] > blackThreshold) {
       seen = true;
     }
   }
   return seen;
}

/*
 * Given forward sensor array readings, perform small servo actions so to stay on the line
 */
void microAdjust() {
   //if right most sensor reads black, turn slightly to the right to compensate
  if (sensorValues[0] > 500) maneuver(50, 0, 50);
  //if left most sensor reads black, turn slightly to the left to compensate
  else if (sensorValues[5] > 500) maneuver(0, 50, 50);
}

/*
 * Turn left until the black line is re-acquired
 * (Scenario 1: turn left at a cross intersection)
 * (Scenario 2: make a U-turn at a deadend or after an object detection)
 */
void turnLeft() {
  //turn left in place to escape current black line
  while (blackLine()) {
    maneuver(-50, 50, 50);
    delay(100);
  }

  //turn left in place to re-acquire black line
  while (!blackLine()) {
    maneuver(-50, 50, 50);
    delay(100);
  }
}

/*
 * Reverse of turnLeft() function
 */
void turnRight() {
  //turn right in place to escape current black line
  while (blackLine()) {
    maneuver(50, -50, 50);
    delay(100);
  }

  //turn right in place to re-acquire black line
  while (!blackLine()) {
    maneuver(50, -50, 50);
    delay(100);
  }
}

/*
 *  Beep piezospeaker for 1 second
 */
void beep() {
  tone(piezoSpeakerPin, 3000, 1000);
}

void alert() {
  beep();
  digitalWrite(ledPin, HIGH);
  maneuver(0, 0, 1000);
  digitalWrite(ledPin, LOW);
}

/*
 * Execute servo pair maneuver
 */
void maneuver(int speedLeft, int speedRight, int msTime) {
  //speedLeft, speedRight ranges: Backward  Linear  Stop  Linear  Forward
  //                                -200     -100    0      100     200
  servoLeft.writeMicroseconds(LEFT_SERVO_BASE_VALUE + speedLeft); //left servo speed
  servoRight.writeMicroseconds(RIGHT_SERVO_BASE_VALUE - speedRight); //right servo speed
  if (msTime == -1) { //servo kill signals
    servoLeft.detach();
    servoRight.detach();
  }
  delay(msTime); //maneuver execution duration
}

/*
 * Output PING readings as centimeters
 */
long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}
