//주석
/*
    2023.03.07 첫 아두이노 프로그램입니다.
 */


void setup() 
{
  // put your setup code here, to run once:
 pinMode(3,OUTPUT); // 3번 핀을 출력으로 설정
 pinMode(4,INPUT);  // 4번 핀을 출력으로 설정
}

void loop()
{
  // put your main code here, to run repeatedly:
  digitalWrite(3,HIGH);  // 3번 핀 출력을 HIGH로
  delay(1000);           // 1000 msec지연
  digitalWrite(3,LOW);   // 3번 핀 출력을 LOW로
  delay(1000);           // 1000mesc 지연   
}
