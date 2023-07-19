#include <NewPing.h>
#include <LSM303.h>
#include <Wire.h>
#include <math.h>


#define EncoderAPin 2
#define EncoderBPin 3
#define pulse2_m 0.000488

const unsigned long width = 100; 

volatile int counter = 0;

volatile int encoderB;

double angle = 0.0;

unsigned long current_time;
unsigned long time_old;
unsigned long time_different;

unsigned long previous_Millis = 0;

void Encoder()
{
  current_time = millis();
  time_different = current_time - time_old;

  encoderB = digitalRead(EncoderBPin);

  if (encoderB == LOW) 
  {
    counter++;
  }
  else 
  {                 
    counter--;
  }
  time_old = current_time;
}

void setup()
{
  Serial.begin(115200);
 
  pinMode(EncoderAPin, INPUT_PULLUP);
  pinMode(EncoderBPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(EncoderAPin), Encoder, RISING);
}

void loop()
{
  unsigned long current_Millis = millis();

  if (current_Millis - previous_Millis >= width)
  {
   

    previous_Millis = current_Millis;
  }

  Serial.print("Pulse_counter: "); 
  Serial.println(counter);
  Serial.print("Wheel_tick: "); 
  Serial.println((counter * pulse2_m));

}
