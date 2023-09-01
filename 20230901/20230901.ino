#include <MsTimer2.h>
unsigned long time1;
unsigned long time2;

void flash() {
  static boolean output = HIGH;

  digitalWrite(3, output);
  output = !output;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(3,OUTPUT) ;
  Serial.begin(115200);
  MsTimer2::set(20, flash);// flash함수를 500ms마다 호출한다
  MsTimer2::start();
  

}

void loop() {
  // put your main code here, to run repeatedly:

  /*time1 = micros();
  digitalWrite(3, HIGH);
  time2 = micros();
  Serial.println(time2-time1);
  digitalWrite(3, LOW);
  time1= micros();
  digitalWrite(3, HIGH);
  while( (micros() - time1)<=19 )
  {
    
  }
  digitalWrite(3, LOW);*/

}
