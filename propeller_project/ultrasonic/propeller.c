/*
for Parallax Ultrasonic sensor, connect a 2K resistor between SIG and pin
Pin 7 Right Ultrasonic
Pin 8 trigger pin Front Ultrasonic
Pin 9 echo pin Front Ultrasonic
Pin 10 Left Ultrasonic 

*/

#include "ping.h"
#include "ping_hcsr04.h"                      // non-parallax ultrasonic
#include "simpletools.h"                      // Include simple tools

const int right_ultrasonic_pin = 7;
const int trigPin = 8;
const int echoPin = 9;
const int left_ultrasonic_pin = 10;

int main()                                    // Main function
{
  while(1)
  {
   int cmDist = ping_cm(left_ultrasonic_pin);
   printf("cmDist = %d\n",cmDist);
   //pause(100);
   int front_dist = ping_hcsr04(trigPin, echoPin);
   printf("front_dist = %d\n",front_dist);
   pause(200);
   
   
  }  
}
