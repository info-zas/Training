#include <Stepper.h>

const int stepsPerRevolution = 2048;

// Create Stepper object with pin sequence IN1-IN3-IN2-IN4
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 7);

void setup() {
myStepper.setSpeed(10); // Set speed in RPM
Serial.begin(9600);
Serial.println("Stepper Motor Test Start");
}

void loop() {
Serial.println("Clockwise rotation");
myStepper.step(stepsPerRevolution); // 1 full rotation clockwise
delay(1000);

Serial.println("Counterclockwise rotation");
myStepper.step(-stepsPerRevolution); // 1 full rotation counterclockwise
delay(1000);
}
