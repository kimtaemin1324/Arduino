#include <MsTimer2.h>

int _100ms_FG = LOW;
int _100ms_CNT = 0;
void TimerISR()
{
  _100ms_CNT++;
  if(_100ms_CNT>=10)
  {
    _100ms_CNT = 0;
    _100ms_FG = HIGH;
    
  }
}
void setup()
{
  pinMode(13, OUTPUT);
  MsTimer2::set(10, TimerISR);
  MsTimer2::start();
}
void loop()
{
  static boolean output = HIGH;

  if(_100ms_FG == HIGH)
  {
    _100ms_FG = LOW;
    digitalWrite(13, output);
    output = !output;
  }
  
}
