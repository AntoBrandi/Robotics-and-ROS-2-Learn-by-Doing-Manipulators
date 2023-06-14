int x = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  Serial.println(x);
  x++;
  delay(0.1);
}
