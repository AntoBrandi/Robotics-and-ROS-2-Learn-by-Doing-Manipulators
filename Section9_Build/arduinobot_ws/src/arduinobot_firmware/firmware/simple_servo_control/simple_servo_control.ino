#include <Servo.h>

// Register a servo motor
Servo motor;

void setup() {
  // Attach and Initialize each Servo to the Arduino pin where it is connected
  motor.attach(8); 

  // Set a known starting point for the motor
  motor.write(90);

  // Start the Serial communication with ROS
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    int angle = Serial.readString().toInt();
    motor.write(angle);
  }
  delay(0.1);
}
