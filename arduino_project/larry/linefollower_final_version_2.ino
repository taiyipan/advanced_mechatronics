//////////////
///////////////////////////////////////////////////////////////////// C1 position not yet tested, other functions works well///////////////////////////////////////
#include <QTRSensors.h>
#include <Servo.h>
#include "SR04.h"
#define TRIG_PIN 3
#define ECHO_PIN 2




// A0~A5 left to right


Servo leftservo;
Servo rightservo;
QTRSensors qtr;


const uint8_t light_threshold = 100;
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long dist; // receive ultrasonic sensor values
int mytime, new_time; // debounce 
int buzzer = 12; //the pin of the active buzzer
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
uint16_t count = 0; // count intersection
uint16_t obj_count = 0; // count object
bool go_left=0 , go_right=0, go_forward=0; // object indicator
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
  delay(1111);

  Serial.begin(9600);
}

void loop() {
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();



  /* *********************************************************************************************************************** */
  
  linefollow(); //basic line following


  ////////////////////////meet first intersection////////////////////////////////
  if(sensorValues[5]>light_threshold && sensorValues[0]>light_threshold && sensorValues[2]>light_threshold && sensorValues[3]>light_threshold && count==0)
  {
    count++;
    mytime=millis();
    Serial.print("0000000000000000000000000000000");
    Serial.print('\t');
 
    servo_forward();
    delay(150);
    return;
  }


  /////////////////////meet starting intersection, do not stop, count=1////////////////////////////
  if(sensorValues[5]>light_threshold && sensorValues[4]>light_threshold && sensorValues[2]>light_threshold && sensorValues[1]>light_threshold && sensorValues[0]>light_threshold && count==1)
  {
    Serial.println("debounce");
    new_time=millis();
    if (new_time-mytime<3000)
    {
      servo_forward();
      delay(50);
      return;
    }

    count++;
    Serial.print(count);
    Serial.print("Start!");
    Serial.print('\t');
    
    servo_stop();
    buzzer_n_times(1);
    delay(1111);
    
    servo_forward();
    delay(500);
    return;
  }
  
  ///////////////// meet intersection 1~3, start detecting object//////////////////
  if(sensorValues[5]>light_threshold && sensorValues[0]>light_threshold && sensorValues[2]>light_threshold && sensorValues[3]>light_threshold && count>1 && count<5)
  {
    count++;
    Serial.print(count);
    Serial.print("Intersection Detected!");
    Serial.print('\t');
    servo_stop();
    buzzer_n_times(1);
    delay(1111);

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

    return;

 
  }

///////////////// meet intersection 4, start detecting object//////////////////
  if(sensorValues[5]>light_threshold && sensorValues[0]>light_threshold && sensorValues[2]>light_threshold && sensorValues[3]>light_threshold && count==5)
  {
    count++;
    Serial.print("Intersection Detected!");
    Serial.print(count);
    Serial.print('\t');
    servo_stop();
    buzzer_n_times(1);
    delay(1111);

    servo_forward();
    delay(400);
    
    servo_left();
    delay(turn_90);

    servo_stop();
    delay(500);

    dist=sr04.Distance();
    if(dist<55) // check left
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
    if(dist<55) // check right
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

    return;

 
  }


////////////meet the last intersection////////////////////////////
if(sensorValues[5]>light_threshold && sensorValues[0]>light_threshold && sensorValues[2]>light_threshold && sensorValues[3]>light_threshold && count==6) 
  {
    count++;
    Serial.print(count);
    Serial.print("Last Intersection Detected!");
    Serial.print('\t');
    servo_stop();
    buzzer_n_times(1);
    delay(1111);

    servo_forward();
    delay(400);
    
    servo_left(); // turn left
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
    
    servo_right(); // face front
    delay(turn_90);
   
    servo_stop();
    delay(500);

    dist=sr04.Distance();
    if(dist<30) // check front
    {
      go_forward=1;
      buzzer_n_times(2);
    }
      else
      { 
      go_forward=0;
      }
      
    servo_right(); //turn right
    delay(turn_90);

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

    servo_left(); // face front
    delay(turn_90);

    return;
  }


  
///////////////////object detected at left//////////////////////
  if(go_left==1 && go_right==0) 
  {
    servo_left(); // turn left
    delay(turn_90);

    do{ ////////////////////////////////sometimes zero!!!!!!!!!!!!!!!!!!!!!!!!
      dist=sr04.Distance();
//      Serial.print("dist = ");
//      Serial.println(dist);
      linefollow();
    }while(dist>6 || dist==0);
    Serial.println("Reach Object BBBBBB");

    servo_stop();
    delay(1111);
    buzzer_n_times(3);
    obj_count++;

    servo_right();
    delay(turn_180);

    servo_stop();
    delay(101);

    do{
      linefollow();
    }while(sensorValues[5]<light_threshold || sensorValues[0]<light_threshold);


    servo_stop();
    delay(500);

    servo_forward();
    delay(400);

    servo_left();
    delay(turn_90);

    go_left=0; // reset indicator

    return;
  }


//////////////object detected at right///////////////
  if(go_left==0 && go_right==1) 
  {
    servo_right(); // turn right
    delay(turn_90);

    do{
      dist=sr04.Distance();
//      Serial.print("dist = ");
//      Serial.println(dist);
      linefollow();
      
    }while(dist>6 || dist==0);
    Serial.println("Reach Object AAAAAA");

    servo_stop(); // stop and beep
    delay(1111);
    buzzer_n_times(3);
    obj_count++;

    servo_left(); // turn 180
    delay(turn_180);

    do{
      linefollow();
    }while(sensorValues[5]<light_threshold || sensorValues[0]<light_threshold);
 
    servo_stop();
    delay(500);

    servo_forward();
    delay(400);

    servo_right();
    delay(turn_90);

    go_right=0; // reset indicator

    return;
  }


////////////object detected at both direction///////////////
  if(go_left==1 && go_right==1) 
  {
    servo_left(); // turn left first
    delay(turn_90);

    do{
      dist=sr04.Distance();
//      Serial.print("dist = ");
//      Serial.println(dist);
      linefollow();
    }while(dist>6 || dist==0);
    Serial.println("Reach Object BBBBBBB");

    servo_stop();
    delay(1111);
    buzzer_n_times(3);
    obj_count++;

    servo_right();
    delay(turn_180);

    do{
      linefollow();
    }while(sensorValues[5]<light_threshold || sensorValues[0]<light_threshold);
 
    servo_stop();
    delay(500);

    servo_forward(); //get across the intersection
    delay(400);

    do{
      dist=sr04.Distance();
//      Serial.print("dist = ");
//      Serial.println(dist);
      linefollow();
    }while(dist>6 || dist==0);
    Serial.println("Reach Object AAAAAAA");

    servo_stop();
    delay(1111);
    buzzer_n_times(3);
    obj_count++;
    
    servo_left(); 
    delay(turn_180);

    do{
      linefollow();
    }while(sensorValues[5]<light_threshold || sensorValues[0]<light_threshold);

    servo_stop();
    delay(500);

    servo_forward();
    delay(400);

    servo_right();
    delay(turn_90);

    go_left = go_right = 0; // reset indicator

    return;
  } 

//////////////object detected in front///////////////
  if(go_forward==1 && go_left==0 && go_right==0) 
  {
    do{
      dist=sr04.Distance();
      linefollow();
      }while(dist>6 || dist==0);
      
    Serial.println("Reach Object CCCCCCCCCC");
    servo_stop(); // stop and beep
    delay(1111);
    buzzer_n_times(3);
    obj_count++;

    Serial.print("Number of object = ");
    Serial.println(obj_count);

    go_forward=0; // reset indicator
    exit(0);
  }

  
}//void loop









/*-----( Declare User-written Functions )-----*/

/////////////////////////servo control function ///////////////////////
void servo_right()
{
  leftservo.write(120);
  rightservo.write(120);
}  
void servo_right_slow()
{
  leftservo.write(92);
  rightservo.write(92);
}  
void servo_left()
{
  leftservo.write(60);
  rightservo.write(60);
}  
void servo_left_slow()
{
  leftservo.write(88);
  rightservo.write(88);
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
void servo_forward_slow() 
{
  leftservo.write(101);
  rightservo.write(79);
}  

void servo_backward()
{
  leftservo.write(60);
  rightservo.write(120);
}


///////////////////////basic line following ///////////////////////
void linefollow()
{
  qtr.read(sensorValues);

  if(sensorValues[5]<light_threshold && sensorValues[0]<light_threshold && sensorValues[2]<light_threshold && sensorValues[3]<light_threshold) //no line detected, turn 180 degrees
  {
    servo_right();
    delay(101);
  }
  if(sensorValues[5]<light_threshold && sensorValues[0]<light_threshold && sensorValues[2]>light_threshold && sensorValues[3]>light_threshold) // forward
  {
    //servo_forward();
    servo_forward_slow();
  }
  if(sensorValues[5]<light_threshold && sensorValues[0]>light_threshold) // right side out, turn left
  {
    servo_left_slow();
  }
  if(sensorValues[5]>light_threshold && sensorValues[0]<light_threshold) // left side out, turn right
  {
    servo_right_slow();
  }
  
}

//////////////////////buzzer ///////////////////////

void buzzer_n_times(int n) // n for multiple times
{
  unsigned char i; 
  for(i=0;i<n;i++)
  {
    tone(buzzer,200);
    delay(1111);
    noTone(buzzer);
    delay(500);
  }
}
