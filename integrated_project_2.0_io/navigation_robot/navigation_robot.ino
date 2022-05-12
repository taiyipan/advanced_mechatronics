#include <Servo.h>
#include <QTRSensors.h>

//instantiate objects
Servo servoLeft, servoRight, servoTop;
QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

//global constants
const int LEFT_SERVO_BASE_VALUE = 1470; //stop motion value
const int RIGHT_SERVO_BASE_VALUE = 1470; //stop motion value
const int leftServoPin = 12;
const int rightServoPin = 13;
const int topServoPin = 9;
const int piezoSpeakerPin = 2;
const int redLedPin = 6;
const int greenLedPin = 10;
const int pingPin = 11;
const int blackThreshold = 600; //define minimum threshold value to qualify as black line (high value means less sensitivity)[min: 0; max: 1000]
const int pingThreshold = 7; //near ping distance to cause alert
const int baseSpeed = 100; //set base DC motor speed for going forward (impacts total completion runtime)
const int rotationSpeed = 50; //set DC motor speed when rotating the robot
const int arduinoResetSignal = -1;
const int friendStatePin = 5;
const int enemyStatePin = 7;
const int buttonPin = 4;

//3 scenarios for navigation routes
int obstacleI2[] = {0, 2, 2, 0, 0, 2, 2, 0, 0, 0, 3, 1, 2, 0, 0, 2, 0, 2, 0, 0, 0};
int obstacleI3[] = {0, 0, 0, 2, 2, 0, 0, 2, 2, 0, 3, 1, 2, 0, 0, 2, 0, 2, 0, 0, 0};
int obstacleI5[] = {0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 2, 2, 0, 0, 2, 0, 2, 0, 0, 0};

//global variables for control algorithm
int *route;
bool obstacleSeen;
int intersectionCount;
int scenario;

void setup() {
  // put your setup code here, to run once:
  //configure piezospeaker, LED, button, and input pins
  pinMode(friendStatePin, INPUT_PULLUP);
  pinMode(enemyStatePin, INPUT_PULLUP);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(piezoSpeakerPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);

  //wait for button press to proceed to calibration
  while (digitalRead(buttonPin) == LOW) {
    delay(500);
  }

  //begin calibration
  alert();

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

  delay(1000);

  //end calibration
  alert();

  //configure servos
  servoLeft.attach(leftServoPin);
  servoRight.attach(rightServoPin);
  servoTop.attach(topServoPin);

  servoLeft.writeMicroseconds(LEFT_SERVO_BASE_VALUE);
  servoRight.writeMicroseconds(RIGHT_SERVO_BASE_VALUE);
  servoTop.write(0);

  //initialize variables
  obstacleSeen = false;
  intersectionCount = -1;

  //define assumed route: change assumption based on ground truth detection
  route = obstacleI5;
  scenario = 5;
}

void loop() {
  // put your main code here, to run repeatedly:
  //receive io command, respond accordingly
  ioProtocol();

  //front ping: if obstacle detected, U-turn the robot
  pingProtocol();

  //navigation control
  navigationProtocol();

  //line follow protocol: forward pulse, variable speeds, center on the line
  lineFollowProtocol();
}

// Protocol functions -------------------------------------------------------------------
/*
 *  Listen for io commands
 */
void ioProtocol() {
  if (digitalRead(friendStatePin) == HIGH) {
    maneuver(baseSpeed, baseSpeed, 700);
    maneuver(0, 0, 500);
    greenAlert();
  } else if (digitalRead(enemyStatePin) == HIGH) {
    maneuver(baseSpeed, baseSpeed, 700);
    maneuver(0, 0, 500);
    redAlert();
    manipulator();
  }
}

/*
 *  Detect forward obstacle, and dynamically switch preprogrammed routes
 */
void pingProtocol() {
  //U-turn if obstacle detected in close proximity in front
  if (pingToggle() && ping() < pingThreshold) {
    //U-turn
    turnRight();
    //switch routes dynamically
    if (!obstacleSeen) {
      if (intersectionCount < 2) {
        route = obstacleI2;
        scenario = 2;
      }
      else if (intersectionCount < 3) {
        route = obstacleI3;
        scenario = 3;
      }
    }
    //update obstacleSeen
    obstacleSeen = true;
  }
}

/*
 * Navigation control algorithm:
 */
void navigationProtocol() {
  //meet deadend
  if (allWhiteSeen()) turnLeft();
  //meet intersection
  else if (allBlackSeen()) {
    //go through intersection for some distance
    maneuver(baseSpeed / 2, baseSpeed / 2, 600 - baseSpeed);
    //differentiate between Y and X intersections
    if (allWhiteSeen()) turnLeft(); //Y intersection
    else { //X intersection
      //end navigation trigger: last X intersection
      killServo();
      //choose a direction
      turn(*(route + intersectionCount));
      //increment
      intersectionCount++;
    }
  }
}

/*
 * Read QTR and set motor forward in a smooth curve
 */
void lineFollowProtocol() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  /// \return An estimate of the position of a black line under the sensors.
  ///
  /// The estimate is made using a weighted average of the sensor indices
  /// multiplied by 1000, so that a return value of 0 indicates that the line
  /// is directly below sensor 0, a return value of 1000 indicates that the
  /// line is directly below sensor 1, 2000 indicates that it's below sensor
  /// 2000, etc. Intermediate values indicate that the line is between two
  /// sensors. The formula is (where \f$v_0\f$ represents the value from the
  /// first sensor):
  ///
  /// \f[
  /// {(0 \times v_0) + (1000 \times v_1) + (2000 \times v_2) + \cdots
  /// \over
  /// v_0 + v_1 + v_2 + \cdots}
  /// \f]
  ///
  /// As long as your sensors aren’t spaced too far apart relative to the
  /// line, this returned value is designed to be monotonic, which makes it
  /// great for use in closed-loop PID control. Additionally, this method
  /// remembers where it last saw the line, so if you ever lose the line to
  /// the left or the right, its line position will continue to indicate the
  /// direction you need to go to reacquire the line. For example, if sensor
  /// 4 is your rightmost sensor and you end up completely off the line to
  /// the left, this function will continue to return 4000.
  ///
  /// This function is intended to detect a black (or dark-colored) line on a
  /// white (or light-colored) background. For a white line, see
  /// readLineWhite().
  uint16_t position = qtr.readLineBlack(sensorValues);
  //smooth forward pulse control: variable speed control for left and right servos
  float posFactor = (float)position / 5000.0;
  int leftServoSpeed = (int)(posFactor * baseSpeed);
  int rightServoSpeed = (int)((1 - posFactor) * baseSpeed);
  maneuver(leftServoSpeed, rightServoSpeed, 50);
}
// Utility functions --------------------------------------------------------------------
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
  return distance;
}

/*
 * Validate ping toggle: on or off, with aims to reduce debris in route false positives
 */
bool pingToggle() {
  if (!obstacleSeen) return true;
  else if (obstacleSeen && scenario == 2 && intersectionCount == 8) return true;
  else if (obstacleSeen && scenario == 3 && intersectionCount == 9) return true;
  else return false;
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
 * When conditions are met, send kill signals to servos
 */
void killServo() {
  //end navigation trigger: last X intersection; send servo kill signal
  int scenario2Length = sizeof(obstacleI2) / sizeof(obstacleI2[0]);
  int scenario3Length = sizeof(obstacleI3) / sizeof(obstacleI3[0]);
  int scenario5Length = sizeof(obstacleI5) / sizeof(obstacleI5[0]);
  if ((scenario == 2 && intersectionCount >= scenario2Length) || (scenario == 3 && intersectionCount >= scenario3Length) || (scenario == 5 && intersectionCount >= scenario5Length)) maneuver(0, 0, -1);
}

/*
 * Red-green alternate blinking: startup sequence
 */
void alert() {
  tone(piezoSpeakerPin, 3500, 1000);
  for (int i = 0; i < 10; i++) {
    if (i % 2 == 0) {
      digitalWrite(redLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
    } else {
      digitalWrite(redLedPin, LOW);
      digitalWrite(greenLedPin, HIGH);
    }
    delay(100);
  }
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, LOW);
}

/*
 * Red blinking: enemies
 */
void redAlert() {
  tone(piezoSpeakerPin, 3500, 1000);
  for (int i = 0; i < 10; i++) {
    if (i % 2 == 0) {
      digitalWrite(redLedPin, HIGH);
    } else {
      digitalWrite(redLedPin, LOW);
    }
    delay(100);
  }
  digitalWrite(redLedPin, LOW);
}

 /*
  * Green blinking: friends
  */
void greenAlert() {
  tone(piezoSpeakerPin, 3500, 1000);
  for (int i = 0; i < 10; i++) {
    if (i % 2 == 0) {
      digitalWrite(greenLedPin, HIGH);
    } else {
      digitalWrite(greenLedPin, LOW);
    }
    delay(100);
  }
  digitalWrite(greenLedPin, LOW);
}

/*
 * Read forward sensor array values to detect presence of black line
 */
bool blackLine() {
  //define function return boolean (whether black line is seen or not)
  bool seen = false;
  //read QTR
  qtr.readLineBlack(sensorValues);
  //validate black line presence
  for (uint8_t i = 0; i < SensorCount; i++) if (sensorValues[i] > blackThreshold) seen = true;
  return seen;
}

/*
 * Detect conditions for all sensors reading black
 */
bool allBlackSeen() {
  //define boolean flags for state detection
  bool allBlack = true;
  //read QTR
  qtr.readLineBlack(sensorValues);
  //validate all sensors reading black
  for (uint8_t i = 0; i < SensorCount; i++) if (sensorValues[i] <= blackThreshold) allBlack = false;
  return allBlack;
}

/*
 * Detect conditions for all sensors reading white
 */
bool allWhiteSeen() {
  //define boolean flags for state detection
  bool allWhite = true;
  //read QTR
  qtr.readLineBlack(sensorValues);
  //validate all sensors reading white
  for (uint8_t i = 0; i < SensorCount; i++) if (sensorValues[i] > blackThreshold) allWhite = false;
  return allWhite;
}

/*
 * Turn left until the black line is re-acquired
 * (Scenario 1: turn left at a cross intersection)
 * (Scenario 2: make a U-turn at a deadend or after an object detection)
 */
void turnLeft() {
  //turn left in place to escape current black line
  while (blackLine()) {
    maneuver(-rotationSpeed, rotationSpeed, 50);
    delay(100);
  }
  //turn left in place to re-acquire black line
  while (!blackLine()) {
    maneuver(-rotationSpeed, rotationSpeed, 50);
    delay(100);
  }
  //refresh QTR
  qtr.readLineBlack(sensorValues);
}

/*
 * Reverse of turnLeft() function
 */
void turnRight() {
  //turn right in place to escape current black line
  while (blackLine()) {
    maneuver(rotationSpeed, -rotationSpeed, 50);
    delay(100);
  }
  //turn right in place to re-acquire black line
  while (!blackLine()) {
    maneuver(rotationSpeed, -rotationSpeed, 50);
    delay(100);
  }
  //refresh QTR
  qtr.readLineBlack(sensorValues);
}

/*
 * At X intersection, turnLeft() twice to make 180 degrees U-turn
 */
void intersectionUTurn() {
  turnRight();
  turnRight();
}

/*
 * Interpret a turning direction by passing in an integer
 * 0: forward, 1: turn left, 2: turn right, 3: intersection U-turn
 */
void turn(int direction) {
  switch(direction) {
    case 0:
      break;
    case 1:
      turnLeft();
      break;
    case 2:
      turnRight();
      break;
    case 3:
      intersectionUTurn();
      break;
    default:
      return;
  }
}

/*
 * Execute manipulator arm routine, sweep 180 degrees on left side of robot
 */
 void manipulator() {
   servoTop.write(180);
   delay(1500);
   servoTop.write(0);
   delay(1500);
 }
