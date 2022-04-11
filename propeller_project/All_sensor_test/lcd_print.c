
#include "simpletools.h"

serial *lcd;

const int ON  = 22;
const int CLR = 12;

void lcd_print(int lcd_msg)
{
  lcd = serial_open(6, 6, 0, 9600);  //serial_open(pin, pin, 0, boad_rate)
  
  writeChar(lcd, ON);
  writeChar(lcd, CLR);
  pause(5);
  
  dprint(lcd, "lcd message: %d\n" , lcd_msg);
}
