const int signalPin = 13;

void setup()
{
  pinMode(signalPin, OUTPUT);
}

void loop()
{
  digitalWrite(signalPin, HIGH);
  delay(10);
  digitalWrite(signalPin, LOW);
  delay(10);
}
