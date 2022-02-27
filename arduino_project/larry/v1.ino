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
int turn_90 = 500; // time needed to turn 90 degrees
int turn_180 = 1000; // time needed to turn 180 degrees

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
  //delay(250);


  // basic line following
  if(sensorValues[5]<100 && sensorValues[0]<100 && sensorValues[2]<100 && sensorValues[3]<100) //no line detected
  {
    servo_right();
    delay(100);
    return;
  }
  if(sensorValues[5]<100 && sensorValues[0]<100 && sensorValues[2]>100 && sensorValues[3]>100) // forward
  {
    servo_forward();
    delay(100);
    return;
  }
  if(sensorValues[5]<100 && sensorValues[0]>100) // right side out, turn left
  {
    servo_left();
    delay(100);
    return;
  }
  if(sensorValues[5]>100 && sensorValues[0]<100) // left side out, turn right
  {
    servo_right();
    delay(100);
    return;
  }

  // meet 1st intersection, do not stop, count=1
  if(sensorValues[5]>100 && sensorValues[0]>100 && count==0)
  {
    delay(1000);
    count=1;
//    serial.write(led,HIGH); // turn on LED 1s to indicate intersection
//    delay(1000);
//    serial.write(led,LOW); 
    servo_forward();
    return;
  }
    
  // meet 2nd intersection, count=2
  if(sensorValues[5]>100 && sensorValues[0]>100 && count==1)
  {
    delay(1000);
    count=2;
//    serial.write(red_led,HIGH); // turn on red LED 1s to indicate intersection
//    delay(1000);
//    serial.write(red_led,LOW);
    servo_right(); // turn right first
    delay(turn_90); // to be determined
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
