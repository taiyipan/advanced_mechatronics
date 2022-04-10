/*
  Driver program for propeller bot: navigate urban grid, non-stop
*/
#include "simpletools.h"                      // Include simple tools
#include "ping.h"
#include "servo.h"

//function prototypes
void nagivate(void *par);
void readLine(void *par);
void pingFront(void *par);
void pingLeft(void *par);
void pingRight(void *par);
void intersectionDetected(void *par);
void targetDetected(void *par);
void blinkLED();
void piezo();
void ping_hcsr04(int, int);
void maneuver(int, int, int);
void compensate();
void goForward();
void turnLeft();
void turnRight();
void blackLine();
void allBlack();
void allWhite();

//reserve stack space
unsigned int stackMotors[40 + 25];
unsigned int stackQTR[40 + 25];
unsigned int stackFrontPing[40 + 25];
unsigned int stackLeftPing[40 + 25];
unsigned int stackRightPing[40 + 25];
unsigned int stackIntersectDetect[40 + 25];
unsigned int stackTargetDetect[40 + 25];

//shared global variables
serial *lcd;
const int ON  = 22;
const int CLR = 12;
const int frontPingPin = 0;
const int leftPingPin = 10;
const int trigPin = 8;
const int echoPin = 9;
const int led1 = 26;
const int led2 = 27;
const int piezoPing = 0;
const int servoLinearSpeed = 50;
const int servoStopSpeed = 0;
const int servoOffSet = -30;
const int rightServo = 16;
const int leftServo = 17;
const int QTRThreshold = 500;
const int QTRSensorCount = 4;
const int pingThreshold = 15; //cm
static volatile int leftPingDist, rightPingDist, frontPingDist;
static volatile int[4] QTRreadings;
static volatile int intersectionCount = 0;
static volatile int[] map = {0, 1, 2, 0, 0, 2, 0, 2, 0, 0, 2, 0, 2, 0, 0, 0}; //0: forward, 1: turn left, 2: turn right
static volatile int position = 0;
static volatile int intersectCogID, targetCogID;

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
  cogstart((void*) pingRight, NULL, stackRightPing, sizeof(stackRightPing)); //cog 5

  while(1)
  {
    //obtain PING readings and indicate target detection
    if (leftPingDist < pingThreshold || rightPingDist < pingThreshold || frontPingDist < pingThreshold) {
      //indicate target
      targetCogID = cogstart((void*) targetDetected, NULL, stackTargetDetect, sizeof(stackTargetDetect));
      //time buffer
      pause(2000);
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
    //stay on track
    compensate();
    //meet deadend
    if (allWhite()) turnLeft();
    //meet intersection
    else if (allBlack()) {
      //indicate detection
      intersectCogID = cogstart((void*) intersectionDetected, NULL, stackIntersectDetect, sizeof(stackIntersectDetect));
      //forward pulse x10
      for (int i = 0; i < 10; i++) goForward();
      //differentiate between Y and X intersections
      if (allWhite()) turnLeft(); //Y intersection
      else { //X intersection
        //end navigation trigger: last X intersection
        if (position >= (sizeof(map) / sizeof(map[0]))) break;
        //left turn
        if (map[position] == 1) turnLeft();
        //right turn
        else if (map[position] == 2) turnRight();
        //increment current position
        position++;
      }
    }
  }
}

/*
Cog 2: IR sensor (QTR reflective array: 4 readings "OXOOXO" arrangement)
*/
void readLine(void *par) {

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
Cog 5: right ultrasonic sensor (non-Parallax)
*/
void pingRight(void *par) {
  while (1) {
    rightPingDist = ping_hcsr04(trigPin, echoPin);
    pause(100);
  }
}

/*
Cog 6: LCD display (indicate intersection detection)
*/
void intersectionDetected(void *par) {
  lcd = serial_open(15, 15, 0, 9600);

  writeChar(lcd, ON);
  writeChar(lcd, CLR);
  pause(5);

  dprint(lcd, "Intersection: %d\n" , intersectionCount);

  cogstop(intersectCogID);
}

/*
Cog 7: Piezospeaker and LED (indicate target detection)
*/
void targetDetected(void *par) {
  piezo();
  blinkLED();
  cogstop(targetCogID);
}

/*
One blink cycle: pin 26, 27 (200ms total)
*/
void blinkLED() {
  high(led1);
  pause(100);
  low(led1);
  high(led2);
  pause(100);
  low(led2);
}

/*
One Piezospeaker cycle: 1000ms total
*/
void piezo() {
  freqout(piezoPing, 1000, 3000);
}

/*
Custom function to obtain right ultrasonic ping reading (non-Parallax)
*/
int ping_hcsr04(int trig, int echo)
{
    set_directions(trig,echo,0b10); //set up pins

    low(trig);                      //set trig to low before pulse_out
    pulse_out(trig,10);             //send 10us pulse

    long duration = pulse_in(echo,1);

    int hcsr04_cmDist = duration*0.034/2; //calculate distance
    //int hcsr04_cmDist = duration/58;
    //printf("echo = %d\n",duration);
    if(hcsr04_cmDist>=200)
    {
      hcsr04_cmDist=200;
    }
    return hcsr04_cmDist;
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
   if (QTRreadings[0] > QTRThreshold) maneuver(servoLinearSpeed, servoStopSpeed, 50);
   //if left most sensor reads black, turn slightly to the left to compensate
   else if (QTRreadings[QTRSensorCount - 1] > QTRThreshold) maneuver(servoStopSpeed, servoLinearSpeed, 50);
}

/*
Forward pulse: 50ms
*/
void goForward() {
  maneuver(servoLinearSpeed, servoLinearSpeed, 50);
}

/*
Rotate in place until black line is lost and then re-acquired: counterclockwise
*/
void turnLeft() {
  while (blackLine()) {
    maneuver(-servoLinearSpeed, servoLinearSpeed, 50);
    pause(100);
  }
  while (!blackLine()) {
    maneuver(-servoLinearSpeed, servoLinearSpeed, 50);
    pause(100);
  }
}

/*
Rotate in place until black line is lost and then re-acquired: clockwise
*/
void turnRight() {
  while (blackLine()) {
    maneuver(servoLinearSpeed, -servoLinearSpeed, 50);
    pause(100);
  }
  while (!blackLine()) {
    maneuver(servoLinearSpeed, -servoLinearSpeed, 50);
    pause(100);
  }
}

/*
Validate black line presence
*/
bool blackLine() {
  bool seen = false;
  for (int i = 0; i < QTRSensorCount; i++)
    if (QTRreadings[i] > QTRThreshold) seen = true;
  return seen;
}

/*
Validate all readings black
*/
bool allBlack() {
  bool allBlack = true;
  for (int i = 0; i < QTRSensorCount; i++)
    if (QTRreadings[i] <= QTRThreshold) allBlack = false;
  return allBlack;
}

/*
Validate all readings white
*/
bool allWhite() {
  bool allWhite = true;
  for (int i = 0; i < QTRSensorCount; i++)
    if (QTRreadings[i] > QTRThreshold) allWhite = false;
  return allWhite;
}
