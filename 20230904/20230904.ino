#include <MsTimer2.h>

unsigned long time1;
unsigned long time2;

void flash()
{
  static boolean output = HIGH;

  digitalWrite(18,output);
  output = !output;
}

void setup()
{
  pinMode(18, OUTPUT);
  Serial.begin(115200);
  
  MsTimer2::set(1, flash); 
  MsTimer2::start();
}

void loop()
{
  
}
