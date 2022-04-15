/*
  Driver program for propeller bot: navigate urban grid, non-stop
*/
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>
#include "simpletools.h"                      // Include simple tools
#include "ping.h"
#include "servo.h"

//function prototypes
void navigate(void *par);
void readLine(void *par);
void pingFront(void *par);
void pingLeft(void *par);
void intersectionDetected(void *par);
void targetDetected(void *par);
void blinkLED(int);
void piezo();
void ping_hcsr04(int, int);
void maneuver(int, int, int);
void compensate();
void goForward();
void turnLeft();
void turnRight();
bool blackLine();
bool allBlack();
bool allWhite();

//reserve stack space
unsigned int stackMotors[40 + 25];
unsigned int stackQTR[40 + 25];
unsigned int stackFrontPing[40 + 25];
unsigned int stackLeftPing[40 + 25];
unsigned int stackIntersectDetect[40 + 25];
unsigned int stackTargetDetect[40 + 25];

//shared global variables
serial *lcd;
const int ON  = 22;
const int CLR = 12;
const int frontPingPin = 10;
const int leftPingPin = 7;
const int trigPin = 8;
const int echoPin = 9;
const int led1 = 2;
const int led2 = 3;
const int piezoPin = 5;
const int lcdPin = 6;
const int servoLinearSpeed = 25;
const int servoStopSpeed = 0;
const int servoOffSet = -30;
const int rightServo = 12;
const int leftServo = 13;
const int pulseDuration = 200;
const int QTRThreshold = 800;  //above which value is defined as "black"
const int pingThreshold = 15; //cm
const int pulseMultiplier = 6;
static volatile int leftPingDist = INT_MAX, rightPingDist = INT_MAX, frontPingDist = INT_MAX;
static volatile int QTRreadings[4];
static volatile int intersectionCount = 0;
static volatile int map[] = {2, 2, 0, 0, 2, 0, 2, 0, 0, 2, 0, 2, 0, 0, 0}; //0: forward, 1: turn left, 2: turn right
static volatile int position = 0;
static volatile int targetCogID;
static volatile int firstPassCount = 0;
static volatile bool firstPass = true;
static volatile bool obstacleSeen = false;

/*
Cog 0: main function
*/
int main()
{
  //start cogs
  cogstart((void*) navigate, NULL, stackMotors, sizeof(stackMotors)); //cog 1
  cogstart((void*) readLine, NULL, stackQTR, sizeof(stackQTR)); //cog 2
  cogstart((void*) pingFront, NULL, stackFrontPing, sizeof(stackFrontPing)); //cog 3
  cogstart((void*) pingLeft, NULL, stackLeftPing, sizeof(stackLeftPing)); //cog 4
  cogstart((void*) intersectionDetected, NULL, stackIntersectDetect, sizeof(stackIntersectDetect)); //cog 5

  while(1)
  {
    //obtain PING readings and indicate target detection
    if (leftPingDist < pingThreshold || rightPingDist < pingThreshold || frontPingDist < pingThreshold) {
      //indicate target
      targetCogID = cogstart((void*) targetDetected, NULL, stackTargetDetect, sizeof(stackTargetDetect)); //cog 6
      piezo();
      //time buffer: prevent repeated alerts of the same target
      pause(3000);
    }
  }
}

/*
Cog 1: control twin DC motors to navigate the grid arena
*/
void navigate(void *par) {
  while (1) {
    //forward pulse
    goForward();
    //obstacle detection: front ultrasonic ping
    if (!obstacleSeen && frontPingDist < 7) {
      obstacleSeen = true;
      turnLeft(); //U-turn
    }
    //first pass mode toggle
    if (obstacleSeen && firstPassCount == 2) firstPass = false;
    //meet deadend
    if (allWhite()) turnLeft();
    //meet intersection
    else if (allBlack()) {
      //forward pulse multiplier: go through intersection for some distance
      for (int i = 0; i < pulseMultiplier; i++) goForward();
      //differentiate between Y and X intersections
      if (allWhite()) turnLeft(); //Y intersection
      else { //X intersection
        //indicate detection
        intersectionCount++;
        //if firstPass mode is on, contiune straight on central lane, detect obstacle, then return to original route
        if (firstPass) {
          if (!obstacleSeen) {
            firstPassCount++;
          } else {
            firstPassCount--;
          }
        } else { //regular navigation mapping routine
          //end navigation trigger: last X intersection
          if (position >= (sizeof(map) / sizeof(map[0]))) {
            maneuver(servoStopSpeed, servoStopSpeed, pulseDuration);
            break;
          }
          //left turn
          if (map[position] == 1) turnLeft();
          //right turn
          else if (map[position] == 2) turnRight();
          //increment current position
          position++;
        }
      }
    }
    //stay on track
    compensate();
  }
}

/*
Cog 2: IR sensor (QTR reflective array: 4 readings "OXOOXO" arrangement)
*/
void readLine(void *par) {
  //initialize
  adc_init(21, 20, 19, 18);                   // CS=21, SCL=20, DO=19, DI=18
  //update sensor readings
  while (1) {
    for (int i = 0; i < (sizeof(QTRreadings) / sizeof(QTRreadings[0])); i++) QTRreadings[i] = adc_in(i);
    pause(100);
  }
}

/*
Cog 3: front ultrasonic sensor (Parallax)
*/
void pingFront(void *par) {
  while (1) {
    frontPingDist = ping_cm(frontPingPin);
    pause(100);
  }
}

/*
Cog 4: left ultrasonic sensor (Parallax)
*/
void pingLeft(void *par) {
  while (1) {
    leftPingDist = ping_cm(leftPingPin);
    pause(100);
  }
}

/*
Cog 5: LCD display (indicate intersection detection)
*/
void intersectionDetected(void *par) {
  int localCount = 0;
  while (1) {
    lcd = serial_open(lcdPin, lcdPin, 0, 9600);
    writeChar(lcd, ON);
    writeChar(lcd, CLR);
    pause(5);
    dprint(lcd, "%d\n" , intersectionCount);
    if (localCount != intersectionCount) {
      localCount = intersectionCount;
      blinkLED(1);
      for (int i = 0; i < 5; i++) {
        high(26);
        pause(100);
        low(26);
        high(27);
        pause(100);
        low(27);
      }
    }
    pause(1000);
  }
}

/*
Cog 6: Piezospeaker and LED (indicate target detection)
*/
void targetDetected(void *par) {
  blinkLED(10);
  cogstop(targetCogID);
}

/*
One blink cycle
*/
void blinkLED(int cycle) {
  for (int i = 0; i < cycle; i++) {
    high(led1);
    pause(100);
    low(led1);
    high(led2);
    pause(100);
    low(led2);
  }
}

/*
One Piezospeaker cycle: 2000ms total
*/
void piezo() {
  freqout(piezoPin, 2000, 3000);
}

/*
 * Execute servo pair maneuver
 */
void maneuver(int speedLeft, int speedRight, int msTime) {
  //speedLeft, speedRight ranges: Backward  Linear  Stop  Linear  Forward
  //                                -200     -100    0      100     200
  servo_speed(rightServo, servoOffSet - speedRight); //right servo speed
  servo_speed(leftServo, servoOffSet + speedLeft); //left servo speed
  pause(msTime); //maneuver execution duration
}

/*
Read QTRreadings. If robot is off course, compensate so it goes back to its track (also includes deadend and u-turn)
*/
void compensate() {
  //if right most sensor reads black, turn slightly to the right to compensate
   if (QTRreadings[0] > QTRThreshold) maneuver(servoLinearSpeed, -servoLinearSpeed, pulseDuration - 50);
   //if left most sensor reads black, turn slightly to the left to compensate
   else if (QTRreadings[(sizeof(QTRreadings) / sizeof(QTRreadings[0])) - 1] > QTRThreshold) maneuver(-servoLinearSpeed, servoLinearSpeed, pulseDuration - 50);
}

/*
Forward pulse: 50ms
*/
void goForward() {
  maneuver(servoLinearSpeed, servoLinearSpeed, pulseDuration);
}

/*
Rotate in place until black line is lost and then re-acquired: counterclockwise
*/
void turnLeft() {
  while (blackLine()) maneuver(-servoLinearSpeed, servoLinearSpeed, pulseDuration);
  while (!blackLine()) maneuver(-servoLinearSpeed, servoLinearSpeed, pulseDuration);
  maneuver(-servoLinearSpeed, servoLinearSpeed, pulseDuration);
}

/*
Rotate in place until black line is lost and then re-acquired: clockwise
*/
void turnRight() {
  while (blackLine()) maneuver(servoLinearSpeed, -servoLinearSpeed, pulseDuration);
  while (!blackLine()) maneuver(servoLinearSpeed, -servoLinearSpeed, pulseDuration);
  maneuver(servoLinearSpeed, -servoLinearSpeed, pulseDuration);
}

/*
Validate black line presence
*/
bool blackLine() {
  bool seen = false;
  for (int i = 0; i < (sizeof(QTRreadings) / sizeof(QTRreadings[0])); i++)
    if (QTRreadings[i] > QTRThreshold) seen = true;
  return seen;
}

/*
Validate all readings black
*/
bool allBlack() {
  bool allBlack = true;
  for (int i = 0; i < (sizeof(QTRreadings) / sizeof(QTRreadings[0])); i++)
    if (QTRreadings[i] <= QTRThreshold) allBlack = false;
  return allBlack;
}

/*
Validate all readings white
*/
bool allWhite() {
  bool allWhite = true;
  for (int i = 0; i < (sizeof(QTRreadings) / sizeof(QTRreadings[0])); i++)
    if (QTRreadings[i] > QTRThreshold) allWhite = false;
  return allWhite;
}
