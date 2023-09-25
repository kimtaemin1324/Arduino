#include <MsTimer2.h>

#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

#define wheel_track  0.13       //m 단위로 구할 것, 0.1 = 10cm
#define RAD2DEG(x)   (x*180.0/3.14159)
#define DEG2RAD(x)   (x*3.14159/180.0)

const byte outPin = 13; // Output pin: digital pin 13(D13)
const byte interruptPin1 = 2; // Interrupt pin: D2
const byte interruptPin2 = 3; // Interrupt pin: D2
const int encoder1_A = 18;
const int encoder1_B = 19;
const int encoder2_A = 20;
const int encoder2_B = 21;
const byte resetPin = 5;
volatile byte state = 0;

long cnt1 = 0; // 추가
long cnt2 = 0; // 추가
long cnt1_old = 0;
long cnt2_old = 0;

double pulse_to_distance_left  = 0.2/511;
double pulse_to_distance_right = 0.2/514;

const double odom_left  = 0;
const double odom_right = 0;
double yaw = 0.0;
double yaw_degree = 0.0;

struct Pose2D
{
  double x;
  double y;
  double theta; //theta = radian
} my_odom;

double heading(double x, double y)
{
  double head = atan2(y, x); // stope Y, stope X
  return head;
}

void msTimer2_ISR()
{
  char msg[100] = {0x00};
  double odom_left_delta     = 0.0;
  double odom_right_delta    = 0.0;
  double odom_center_delta   = 0.0;
  double theta_delta         = 0.0;
  double delta_encoder_right = 0.0;
  double delta_encoder_left  = 0.0;
  double theta_delta_degree  = 0.0;

  delta_encoder_right = cnt1 - cnt1_old;
  delta_encoder_left  = cnt2 - cnt2_old;
  
  //sprintf(msg, "encorder delta : {%3d  %3d}", delta_encoder_left, delta_encoder_right);
  //Serial.println(msg);
  Serial.print("delta_encoder_right:  "); Serial.print(delta_encoder_right); Serial.print("  ");
  Serial.print("delta_encoder_left:  ");  Serial.print(delta_encoder_left); Serial.println("  ");
  Serial.println();
  
  odom_right_delta = (delta_encoder_right) * pulse_to_distance_right;
  odom_left_delta  = (delta_encoder_left)  * pulse_to_distance_left;
  odom_center_delta = (odom_right_delta + odom_left_delta) * 0.5;
  
  Serial.print("odom_right_delta:  "); Serial.print(odom_right_delta); Serial.print("     ");
  Serial.print("odom_left_delta:  ");  Serial.print(odom_left_delta); Serial.println("    ");
  
  theta_delta = heading(wheel_track, (odom_right_delta - odom_left_delta));
  Serial.print("theta delta radian:  "); Serial.print(theta_delta); Serial.println(" ");

  yaw += theta_delta;
  theta_delta_degree = RAD2DEG(theta_delta);
  Serial.print("yaw radian:  "); Serial.print(yaw); Serial.println(" ");
  Serial.print("theta delta degree:  "); Serial.print(theta_delta_degree); Serial.println(" ");
  yaw_degree += theta_delta_degree;
  Serial.print("yaw degree:  "); Serial.print(yaw_degree); Serial.println(" ");

  my_odom.x     += odom_center_delta * sin(theta_delta);
  my_odom.y     += odom_center_delta * cos(theta_delta);
  my_odom.theta += theta_delta;
  Serial.print("odom: "); 
  Serial.print(my_odom.x); Serial.print(" ");
  Serial.print(my_odom.y); Serial.print(" ");
  Serial.print(my_odom.theta); Serial.println(" ");
 
  cnt1_old = cnt1;
  cnt2_old = cnt2;
}

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  pinMode(outPin, OUTPUT); // Output mode
  pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
  pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt

  my_odom.x = 0;   my_odom.y = 0;   my_odom.theta = 0;
  
  Serial.begin(115200);
  MsTimer2::set(100, msTimer2_ISR);
  MsTimer2::start();
}

void intfunc1() // Interrupt function
{
  if (digitalRead(encoder1_B) == HIGH) // If D4 output is low
  {
    cnt1++;
  }
  else
  {
    cnt1--;
  }
}

void intfunc2() // Interrupt function
{
  if (digitalRead(encoder2_B) == LOW) // If D4 output is low
  {
    cnt2++;
  }
  else
  {
    cnt2--;
  }
}

void motor_R_control(int direction_r, int motor_speed_r) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if(direction_r == HIGH)
  {
     digitalWrite(IN1, LOW); //모터의 방향 제어
     digitalWrite(IN2, HIGH);
     analogWrite(ENA,motor_speed_r); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA,motor_speed_r);
    
  }
}

void motor_L_control(int direction_l, int motor_speed_l) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if(direction_l == HIGH)
  {
     digitalWrite(IN3, HIGH); //모터의 방향 제어
     digitalWrite(IN4, LOW);
     analogWrite(ENB,motor_speed_l); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB,motor_speed_l);
    
  }
}

void loop()
{
  motor_R_control(HIGH, 100);
  motor_L_control(HIGH, 100);
  delay(1000);
 motor_R_control(HIGH, 0);
  motor_L_control(HIGH, 0);
  delay(2000);

  //motor_R_control(HIGH, 200);
  //motor_L_control(LOW, 200);
  //delay(4200);

  //motor_R_control(HIGH, 100);
  //motor_L_control(HIGH, 100);
  //delay(500);
}
