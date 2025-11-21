// =================== OLED Libraries ===================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =================== Pin Definitions ===================
#define ENCA_L 2
#define ENCB_L 4
#define PWM_L 6
#define IN2_L 8
#define IN1_L 7

#define ENCA_R 3
#define ENCB_R 5
#define PWM_R 11
#define IN2_R 10
#define IN1_R 12



Adafruit_SSD1306 display(128, 64, &Wire);

// =================== Low-Level Motor Control ===================

void motorLeft(int speed, bool forward) {
  analogWrite(PWM_L, speed);

  digitalWrite(IN1_L, forward ? HIGH : LOW);
  digitalWrite(IN2_L, forward ? LOW : HIGH);
}

void motorRight(int speed, bool forward) {
  analogWrite(PWM_R, speed);

  digitalWrite(IN1_R, forward ? HIGH : LOW);
  digitalWrite(IN2_R, forward ? LOW : HIGH);
}

void stopMotors() {
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
}

// =================== High-Level Movement Functions ===================

void moveForward(int speed) {
  motorLeft(speed, true);
  motorRight(speed, true);
  showMessage("Forward");
}

void moveBackward(int speed) {
  motorLeft(speed, false);
  motorRight(speed, false);
  showMessage("Backward");
}

void turnLeft(int speed) {
  motorLeft(speed, false);   // left backwards
  motorRight(speed, true);   // right forward
  showMessage("Left Turn");
}

void turnRight(int speed) {
  motorLeft(speed, true);
  motorRight(speed, false);
  showMessage("Right Turn");
}

// =================== OLED Display Function ===================

void showMessage(String msg) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.println(msg);
  display.display();
}

// =================== Setup ===================

void setup() {
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  showMessage("Ready");
  delay(2000);
}

// =================== Loop ===================

void loop() {

  moveForward(100);
  delay(5000);
  stopMotors();
  delay(1000);

  moveBackward(100);
  delay(5000);
  stopMotors();
  delay(1000);

  turnLeft(100);
  delay(1000);
  stopMotors();
  delay(1000);

  turnRight(100);
  delay(1000);
  stopMotors();
  delay(1000);
}
