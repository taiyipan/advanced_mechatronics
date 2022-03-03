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
const int pingPin = 10;

void setup() {
  // put your setup code here, to run once:
  // initialize serial communication:
  Serial.begin(9600);

  //configure piezospeaker
  pinMode(piezoSpeakerPin, OUTPUT);
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
  pingObject();
  senseLine();
  microAdjust();

  //default state: move forward a bit
  maneuver(50, 50, 50);
}

/*
 * Ping front area for object distance value
 */
void pingObject() {
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

  //detecting close object (<10cm)
  if (distance < 10) {
    beep();
    maneuver(0, 0, 1000);
    turnLeft();
  }
}

/*
 * Read forward sensor array values and execute actions accordingly
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
     if (sensorValues[i] > 500) {
       allWhite = false;
     }
     else allBlack = false;
   }
   Serial.println(position);

   //detecting deadend
   if (allWhite) {
     beep();
     maneuver(0, 0, 1000);
     turnLeft();
   }

   //detecting intersection
   else if (allBlack) {
     beep();
     maneuver(0, 0, 1000);
     maneuver(50, 50, 1000); //go forward a little before turning left
     turnLeft();
   }
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
     if (sensorValues[i] > 500) {
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
 *  Beep piezospeaker for 1 second
 */
void beep() {
  tone(piezoSpeakerPin, 3000, 1000);
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
