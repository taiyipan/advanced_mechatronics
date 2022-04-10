/*
Add TAB "servo_control.c"

servo_forward(int speed)
servo_backward(int speed)
servo_left(int speed)
servo_right(int speed)
servo_stop()

speed:0->still, 100->full speed, 200->twice full speed, -100-> clockwise full speed
                       
 */
#include "simpletools.h"                      // Include simple tools
#include "servo.h"                            // Parallax servo lib

int turn_90 = 600;                            // varies in voltage

int main()                                    // Main function
{
  while(1)
  {
    servo_forward(100);
    pause(1000);
    
    servo_still();
    pause(1000);
    
    servo_left(100);
    pause(turn_90);
    
    servo_still();
    pause(1000);
    
    servo_right(100);
    pause(turn_90);
    
    servo_still();
    pause(1000);
    
    servo_backward(100);
    pause(1000);
    
    servo_still();
    pause(1000);
    
  }  
}
