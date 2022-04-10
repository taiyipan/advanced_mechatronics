
#include "simpletools.h"                      // Include simpletools
#include "adcDCpropab.h"                      // Include adcDCpropab


//int sensor_value[3];


int main()                                    // Main function
{
  pause(1000);                                // Wait 1 s for Terminal app
  adc_init(21, 20, 19, 18);                   // CS=21, SCL=20, DO=19, DI=18

  int i = 0;                                  // Index variable
  while(1)                                    // Loop repeats indefinitely
  {
    if(i == 4)                                // After index = 3
    {
      i = 0;                                  // Reset to zero
      print("%c", HOME);                      // Cursor home
    }  
    print("adc[%d] = %d%c\n", i, adc_in(i), CLREOL);              // Display raw ADC
             
    i++;                                      // Add 1 to index
    
    
/*    for(int i=0;i<=3;i++)       //get sensor values
    {
      sensor_value[i] = adc_in(i);
    }
    printf("value 1 = %d\n", sensor_value[0]); 
    printf("value 2 = %d\n", sensor_value[1]);
    printf("value 3 = %d\n", sensor_value[2]);
    printf("value 4 = %d\n", sensor_value[3]);
    print(" \n");
*/        
    pause(100);                               // Wait 1/10 s
  }  
}
