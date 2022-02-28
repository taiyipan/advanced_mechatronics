//////////////
/////////////////////////////////////////////////////////////////////start and final position not coded yet, other functions work well///////////////////////////////////////
#include <QTRSensors.h>
#include <Servo.h>
#include "SR04.h"
#define TRIG_PIN 3
#define ECHO_PIN 2




// A0~A5 left to right


Servo leftservo;
Servo rightservo;
QTRSensors qtr;



SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long dist;
int buzzer = 12;//the pin of the active buzzer
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
uint16_t count = 0; // count intersection
bool go_left=0 , go_right = 0; // indicator
int turn_90 = 600; // time needed to turn 90 degrees
int turn_180 = 1200; // time needed to turn 180 degrees

void setup() 
{
// configure the sensors

  pinMode(buzzer,OUTPUT);//initialize the buzzer pin as an output
  //buzzer_n_times(5); //n for the buzzer times
  
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  leftservo.attach(10);//connect pin 10 with the control line(the middle line of Servo) 
  rightservo.attach(9);//connect pin 9 with the control line(the middle line of Servo) 
  servo_stop();

  Serial.begin(9600);//Initialization of Serial Port
  delay(1000);

  Serial.begin(9600);
}

void loop() {
   //ultrasonic sensor 
//   dist=sr04.Distance();
//   Serial.print(dist);
//   Serial.println("cm");
//   Serial.print('\t');


  // read ir sensor data
  //qtr.read(sensorValues);

  // print the sensor values as numbers from 0 to 1023, where 0 means maximum
  // reflectance and 1023 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();


  //
  
  linefollow(); //basic line following

  // meet starting intersection, do not stop, count=1
  if(sensorValues[5]>100 && sensorValues[0]>100 && sensorValues[2]>100 && sensorValues[3]>100 && count==0)
  {
    count++;
    Serial.print("11111111111111111111111111111111111111111");
    Serial.print('\t');
    
    servo_stop();
    buzzer_n_times(1);
    delay(1000);
    
    servo_forward();
    delay(500);
    return;
  }
  
  // meet intersection, count>0, get state
  if(sensorValues[5]>100 && sensorValues[0]>100 && sensorValues[2]>100 && sensorValues[3]>100 && count>0)
  {
    Serial.print("2222222222222222222222222222222222222");
    Serial.print('\t');
    servo_stop();
    buzzer_n_times(1);
    delay(1000);

    servo_forward();
    delay(400);
    
    servo_left();
    delay(turn_90);

    servo_stop();
    delay(500);

    dist=sr04.Distance();
    if(dist<30) // check left
    {
      go_left=1;
      buzzer_n_times(2);
    }
    else
    {
    go_left=0;
    }
    Serial.print("dist = ");
    Serial.println(dist);
    Serial.print("go_left = ");
    Serial.println(go_left);
    
    servo_right();
    delay(turn_180);
   
    servo_stop();
    delay(500);

    dist=sr04.Distance();
    if(dist<30) // check right
    {
      go_right=1;
      buzzer_n_times(2);
    }
      else
      { 
      go_right=0;
      }
      
    servo_left(); 
    delay(turn_90);

 
  }
  
  
  if(go_left==1 && go_right==0) //object detected at left
  {
    servo_left(); // turn left
    delay(turn_90);

    do{ ////////////////////////////////sometimes zero!!!!!!!!!!!!!!!!!!!!!!!!
      dist=sr04.Distance();
      Serial.print("dist = ");
      Serial.println(dist);
      linefollow();
    }while(dist>6 || dist==0);
    Serial.println("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL");

    servo_stop();
    delay(1000);
    buzzer_n_times(3);

    servo_right();
    delay(turn_180);

    servo_stop();
    delay(100);

    do{
      linefollow();
    }while(sensorValues[5]<100 || sensorValues[0]<100);


    servo_stop();
    delay(500);

    servo_forward();
    delay(400);

    servo_left();
    delay(turn_90);

    go_left=0; // reset indicator

    return;
  }

  if(go_left==0 && go_right==1) //object detected at right
  {
    servo_right(); // turn right
    delay(turn_90);

    do{
      dist=sr04.Distance();
      Serial.print("dist = ");
      Serial.println(dist);
      linefollow();
      
    }while(dist>6 || dist==0);
    Serial.println("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR");

    servo_stop(); // stop and beep
    delay(1000);
    buzzer_n_times(3);

    servo_left(); // turn 180
    delay(turn_180);

    do{
      linefollow();
    }while(sensorValues[5]<100 || sensorValues[0]<100);
 
    servo_stop();
    delay(500);

    servo_forward();
    delay(400);

    servo_right();
    delay(turn_90);

    go_right=0; // reset indicator

    return;
  }

  if(go_left==1 && go_right==1) //object detected at both direction
  {
    servo_left(); // turn left first
    delay(turn_90);

    do{
      dist=sr04.Distance();
      Serial.print("dist = ");
      Serial.println(dist);
      linefollow();
    }while(dist>6 || dist==0);
    Serial.println("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");

    servo_stop();
    delay(1000);
    buzzer_n_times(2);

    servo_right();
    delay(turn_180);

    do{
      linefollow();
    }while(sensorValues[5]<100 || sensorValues[0]<100);
 
    servo_stop();
    delay(500);

    servo_forward(); //get across the intersection
    delay(400);

    do{
      dist=sr04.Distance();
      Serial.print("dist = ");
      Serial.println(dist);
      linefollow();
    }while(dist>6 || dist==0);
    Serial.println("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWW");

    servo_stop();
    delay(1000);
    buzzer_n_times(3);
    
    servo_left(); 
    delay(turn_180);

    do{
      linefollow();
    }while(sensorValues[5]<100 || sensorValues[0]<100);

    servo_stop();
    delay(500);

    servo_forward();
    delay(400);

    servo_right();
    delay(turn_90);


    go_left = go_right = 0; // reset indicator

    return;
  } 
}




/*-----( Declare User-written Functions )-----*/

//////servo control function ///////////////////////
void servo_right()
{
  leftservo.write(120);
  rightservo.write(120);
}  
void servo_left()
{
  leftservo.write(60);
  rightservo.write(60);
}  
void servo_stop()
{
  leftservo.write(90);
  rightservo.write(90);
} 
void servo_forward() 
{
  leftservo.write(120);
  rightservo.write(60);
}  
void servo_backward()
{
  leftservo.write(60);
  rightservo.write(120);
}


//////basic line following ///////////////////////
void linefollow()
{
  qtr.read(sensorValues);

  if(sensorValues[5]<100 && sensorValues[0]<100 && sensorValues[2]<100 && sensorValues[3]<100) //no line detected, turn 180 degrees
  {
    servo_right();
    delay(100);
  }
  if(sensorValues[5]<100 && sensorValues[0]<100 && sensorValues[2]>100 && sensorValues[3]>100) // forward
  {
    servo_forward();
  }
  if(sensorValues[5]<100 && sensorValues[0]>100) // right side out, turn left
  {
    servo_left();
  }
  if(sensorValues[5]>100 && sensorValues[0]<100) // left side out, turn right
  {
    servo_right();
  }
}

//////buzzer ///////////////////////

void buzzer_n_times(int n) // n for multiple times
{
  unsigned char i; 
  for(i=0;i<n;i++)
  {
    tone(buzzer,200);
    delay(1000);
    noTone(buzzer);
    delay(500);
  }
}
