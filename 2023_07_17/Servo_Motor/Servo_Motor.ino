#include <Servo.h>

Servo servo;  

void setup() {
  servo.attach(9);  // 서보 모터 신호 핀을 9번에 연결 
}

void loop() {
  servo.write(90);  // 서보 모터를 90도로 조정
  delay(1000);     

 
  while (true) {
    continue;
  }
}
