#include <QTRSensors.h>
#include <Servo.h>



// A0~A5 left to right


Servo leftservo;
Servo rightservo;
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];
uint16_t count = 0; // count intersection
bool action = 1; // when to stop
int red_led = 4; // indication of intersection
int yellow_led = 5;// indication of detecting object
int green_led = 6; // indication of reaching object
int turn_90 = 600; // time needed to turn 90 degrees
int turn_180 = 1200; // time needed to turn 180 degrees

void setup() 
{
// configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  leftservo.attach(10);//connect pin 10 with the control line(the middle line of Servo) 
  rightservo.attach(9);//connect pin 9 with the control line(the middle line of Servo) 
  servo_stop();
  pinMode(red_led,OUTPUT);
  pinMode(yellow_led,OUTPUT);
  pinMode(green_led,OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // read raw sensor values
  qtr.read(sensorValues);

  // print the sensor values as numbers from 0 to 1023, where 0 means maximum
  // reflectance and 1023 means minimum reflectance
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  //delay(100);q

followline();

  // meet 1st intersection, do not stop, count=1
  if(sensorValues[5]>100 && sensorValues[0]>100 && count==0)
  {
    count++;
    Serial.print("11111111111111111111111111111111111111111");
    Serial.print('\t');
    servo_stop();
    delay(1000);
//    serial.write(led,HIGH); // turn on LED 1s to indicate intersection
//    delay(1000);
//    serial.write(led,LOW); 
    servo_forward();
    delay(500);
    return;
  }
    
  // meet 2nd intersection, count=2, turn right
  if(sensorValues[5]>100 && sensorValues[0]>100 && (count==1 || count==4||count==7||count==10))
  {
    count++;
    Serial.print("2222222222222222222222222222222222222");
    Serial.print('\t');
    servo_stop();
    delay(1000);
    servo_forward();
    delay(400);
    
//    serial.write(red_led,HIGH); // turn on red LED 1s to indicate intersection
//    delay(1000);
//    serial.write(red_led,LOW);
    servo_right(); // turn right first
    delay(turn_90); 
    servo_forward(); 
    delay(400);
    return;

  }

  // meet 2nd intersection, count=3, go straight
  if(sensorValues[5]>100 && sensorValues[0]>100 && (count==2 || count==5||count==8||count==11))
  {
    count++;
    Serial.print("33333333333333333333333333333333333333");
    Serial.print('\t');
//    servo_stop();
//    delay(1000);
    servo_forward();
    delay(400);
    return;
    
//    serial.write(red_led,HIGH); // turn on red LED 1s to indicate intersection
//    delay(1000);
//    serial.write(red_led,LOW);
    
  }
  
  // meet 2nd intersection, count=3, turn left
  if(sensorValues[5]>100 && sensorValues[0]>100 && (count==3 || count==6||count==9||count==12))
  {
    count++;
    Serial.print("444444444444444444444444444444444444");
    Serial.print('\t');
    servo_stop();
    delay(1000);
    servo_forward();
    delay(400);
    servo_left();
    delay(turn_90);
    
//    serial.write(red_led,HIGH); // turn on red LED 1s to indicate intersection
//    delay(1000);
//    serial.write(red_led,LOW);
   servo_forward();
   delay(500); 
   return;
  }


// ultrasonic
  //    if(distance<30)
  //    {
  //      servo_stop();
  //      serial.write(green_led,HIGH);
  //      delay(1000);
  //      serial.write(green_led,LOW);
  //      servo_right(turn_180);
  //    }
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
void followline()
{
  if(sensorValues[5]<100 && sensorValues[0]<100 && sensorValues[2]<100 && sensorValues[3]<100) //no line detected, turn 180 degrees
  {
    servo_right();
    delay(turn_180);
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
