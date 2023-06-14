#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); 

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    int x = Serial.readString().toInt();
    if(x == 0)
    {
      // turn off the led
      digitalWrite(LED_PIN, LOW); 
    }
    else
    {
      // turn on the led
      digitalWrite(LED_PIN, HIGH); 
    }
  }
  delay(0.1);
}
