#include <NewPing.h>

#define SONAR_NUM 1 // 센서 1  
#define MAX_DISTANCE 150

#define Front 0

#define TRIG 2  //  초음파 센서 Trig 핀 
#define ECHO 3  //  초음파 센서 Echo 핀

NewPing sonar(TRIG, ECHO, MAX_DISTANCE);


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);// 통신 속도 115200으로 변경
}

long sonar_front(void) {
  
  long duration, distance;

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  
  duration = pulseIn(ECHO, HIGH);
  distance = ((float)(340 * duration) / 1000) / 2;
  
  
  
  return distance;
}

void loop() {
  float front_sonar = 0.0;  

  front_sonar = sonar.ping_cm()*10;  
  if(front_sonar == 0.0)  front_sonar = MAX_DISTANCE;
   
 
  Serial.print("Distance: ");
  Serial.print(front_sonar);  
  Serial.println("mm");

  if((front_sonar > 0) && (front_sonar <= 220.0)){  // 220mm보다 가까우면 LED 켜짐, 멀어지면 LED 꺼짐
    digitalWrite(LED_BUILTIN, HIGH);   // LED 켜짐
    delay(1000);  
  }
  
  else{
    digitalWrite(LED_BUILTIN, LOW);    // LED 꺼짐
    delay(1000);   
 
  }
 

}
