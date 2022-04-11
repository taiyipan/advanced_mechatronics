/*
pin 12 for right servo
pin 13 for left servo

servo_speed(pin,speed)        still     counter clockwise full speed           clockwise full speed
                       speed    0                  100                                -100 
speed=-30 is still, don't know why
*/

#include "simpletools.h"                      // Include simple tools
#include "servo.h"

void servo_forward(int speed)
{
  servo_speed(12,-speed-30);
  servo_speed(13,speed-30);
}  

void servo_backward(int speed)
{
  servo_speed(12,speed-30);
  servo_speed(13,-speed-30);
}  

void servo_left(int speed)
{
  servo_speed(12,-speed-30);
  servo_speed(13,-speed-30);
}  

void servo_right(int speed)
{
  servo_speed(12,speed-30);
  servo_speed(13,speed-30);
}  

void servo_still()
{
  servo_speed(12,-30);
  servo_speed(13,-30);
}  