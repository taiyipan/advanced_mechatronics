#include <Servo.h>
/* After including the corresponding libraries,
   we can use the "class" like "Servo" created by the developer for us.
   We can use the functions and variables created in the libraries by creating 
   objects like the following "myservo" to refer to the members in ".".*/

Servo myservo;
//it created an object called myservo.
/*  you can see Servo as a complex date type(Including functions and various data types)
    and see myservo as variables.               */

void setup(){
  /*"attach" and "write" are both functions,
     and they are members contained in the complex structure of "Servo". 
     We can only use them if we create the object "myservo" for the complex structure of "Servo".
  */
  myservo.attach(9);//connect pin 9 with the control line(the middle line of Servo) 
  myservo.write(90);// move servos to center position -> 90°
  
  Serial.begin(9600);  
  Serial.println("--- Start controlling the arm ---");
  Serial.println(" Type in Box above to control the arm(1 for left, 2 for right,3 for both side )");
  Serial.println();
} 
void loop(){

if (Serial.available()) { // Check if at least one character available
  char ch = Serial.read();
  switch(ch) {
    case '1':
      Serial.println("You entered 1,  arm turn left"); 
      arm_left();
      break;
    case '2':
      Serial.println("You entered 2,  arm turn right "); 
      arm_right();
      break;
    case '3':
      Serial.println("You entered 3, arm turn both side"); 
      arm_left();
//      arm_middle();
      arm_right();
      break;
    default :
      Serial.print(ch);
      Serial.println(" was received but not expected");
      break;
    }
    arm_middle();
   }
  
//  myservo.write(0);// move servos to center position -> 0° //angle: the value to write to the servo, from 0 to 180
//  delay(1000);
//  myservo.write(90);// move servos to center position -> 90°
//  delay(1000);
//  myservo.write(180);// move servos to center position -> 180°
//  delay(1000);
//  myservo.write(90);// move servos to center position -> 90°
//  delay(1000);

}

void arm_right()
{
  myservo.write(0);// move servos to center position -> 0°
  delay(1000);
}  
void arm_left()
{
  myservo.write(180);// move servos to center position -> 180°
  delay(1000);
}  

void arm_middle()
{
  myservo.write(90);// move servos to center position -> 90°
  delay(1000);
} 
