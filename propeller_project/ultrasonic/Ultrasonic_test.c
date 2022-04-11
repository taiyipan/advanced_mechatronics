/*
Pin 7 Right Ultrasonic
Pin 8 trigger pin Front Ultrasonic
Pin 9 echo pin Front Ultrasonic
Pin 10 Left Ultrasonic 
*/

#include "ping.h"
#include "ping_hcsr04.h"                      // non-parallax ultrasonic
#include "simpletools.h"                      // Include simple tools


const int left_ultrasonic_pin = 7;
const int trigPin = 8;
const int echoPin = 9;
const int front_ultrasonic_pin = 10;

int main()                                    // Main function
{
  while(1)
  {
   int left_dist = ping_cm(left_ultrasonic_pin);
   printf("left_dist = %d\n",left_dist);

   int right_dist = ping_hcsr04(trigPin, echoPin);
   printf("right_dist = %d\n",right_dist);

   pause(200);
   
  }  
}
