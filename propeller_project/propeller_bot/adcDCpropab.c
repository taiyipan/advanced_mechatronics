
#include "simpletools.h"
#include "adcDCpropab.h"

static int dout, din, scl, cs;

int adc124S021dc(int channel);

void adc_init(int csPin, int sclPin, int doPin, int diPin)
{
  dout = doPin;
  din = diPin;
  scl = sclPin;
  cs = csPin;  
}

int adc_in(int channel)
{
  adc124S021dc(channel);
  int val = adc124S021dc(channel);
  return val;
}

int adc124S021dc(int channel)
{
  channel = (channel & 3) << 12;
  high(cs);
  high(scl);
  low(din);
  input(dout);
  low(cs);
  int val = 0;
  for(int i = 15; i >= 0; i--)
  {
    val = val << 1;
    low(scl);
    high(scl);
    set_output(din, (channel >> i) & 1);
    val = val + (get_state(dout) & 1);
  }
  high(cs);
  return val;
}  


