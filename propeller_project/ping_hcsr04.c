#include "simpletools.h"

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
