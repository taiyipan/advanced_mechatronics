/*

Pin 13 left servo
Pin 12 right servo
Pin 10 middle ultrasonic
Pin 9 right ultrasoinc echo
Pin 8 right ultrasonic trigger
Pin 7 left ultrasonic
Pin 6 LCD 
Pin 5 Piezo

Pin 3 LED Green 
Pin 2 LED Red

*/


#include "simpletools.h"                      // Include simple tools
#include "servo.h"                            // servo library
#include "ping.h"
#include "adcDCpropab.h"                      // ADC library for QTR


void servo_core(void *par);
void piezo_core(void *par);
void lcd_core(void *par);
void left_core(void *par);
void right_core(void *par);

unsigned int stack_servo[40+25];
unsigned int stack_piezo[40+25];
unsigned int stack_lcd[40+25];
unsigned int stack_left[40+25];
unsigned int stack_right[40+25];


unsigned int beep = 0;                        //indicator of piezo

const int left_ultrasonic_pin = 7;            // ultrasonic pin
const int trigPin = 8;
const int echoPin = 9;
const int front_ultrasonic_pin = 10;
int left_dist;
int right_dist;

int lcd_msg = 0;                            //lcd
serial *lcd;
const int ON  = 22;
const int CLR = 12;

/*
Cog 1: main, reading QTR values
*/

int main()                                    // Main function
{
  cogstart(&servo_core, NULL, stack_servo, sizeof(stack_servo));
  cogstart(&piezo_core, NULL, stack_piezo, sizeof(stack_piezo));
  cogstart(&lcd_core, NULL, stack_lcd, sizeof(stack_lcd));
  cogstart(&left_core, NULL, stack_left, sizeof(stack_left));
  cogstart(&right_core, NULL, stack_right, sizeof(stack_right));
  
  


  pause(1000);                                // Wait 1 s for Terminal app
  lcd_msg = 123;
  beep = 1;
  
  
  adc_init(21, 20, 19, 18);                   // CS=21, SCL=20, DO=19, DI=18

  int i = 0;                                  // Index variable
  while(1)                                    // Loop repeats indefinitely
  {
//    if(i == 4)                                // After index = 3
//    {
//      i = 0;                                  // Reset to zero
//      print("%c", HOME);                      // Cursor home
//    }  
//    print("adc[%d] = %d%c\n", i, adc_in(i), CLREOL);              // Display raw ADC             
//    i++;                                      // Add 1 to index   
//    pause(100);
      
    printf("left_dist = %d\n",left_dist);
    
    printf("right_dist = %d\n",right_dist);
    
    pause(200);
    //high(2);
    //pause(500);
    //low(2);
    //pause(500);
    
  }  
}


/*
Cog 1: servo core
*/
void servo_core(void *par)
{
 while(1)
  {
    servo_forward(100);
    pause(1000);
    
    servo_still();
    pause(1000);
    
    servo_left(100);
    pause(600);
    
    servo_still();
    pause(1000);
    
    servo_right(100);
    pause(600);
    
    servo_still();
    pause(1000);
    
    servo_backward(100);
    pause(1000);
    
    servo_still();
    pause(1000);
    
  }
}

/*
Cog 2: piezo core
*/
void piezo_core(void *par)
{
  if(beep == 1)
  {
   freqout(5,1000,3000);                      //freqout(Pin,Duration,Freq) 
   beep = 0; 
  }   
}


/*
Cog 3: LCD core
*/
void lcd_core(void *par)
{
  pause(1000);
  lcd = serial_open(6, 6, 0, 9600);  //serial_open(pin, pin, 0, boad_rate)
  
  writeChar(lcd, ON);
  writeChar(lcd, CLR);
  pause(5);
  
  dprint(lcd, "message: %d\n" , lcd_msg);
}


/*
Cog 4: Left core
*/
void left_core(void *par)
{
  while(1)
  {
   left_dist = ping_cm(left_ultrasonic_pin);

   
  }
}  


/*
Cog 5: Right core
*/
void right_core(void *par)
{
  while(1)
  {
   right_dist = ping_hcsr04(trigPin, echoPin);
   
   
  }
}  
