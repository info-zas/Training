#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Right motor pins
#define ENCA_R 2
#define ENCB_R 4
#define PWM_R 6
#define IN1_R 7
#define IN2_R 8

// Left motor pins
#define ENCA_L 3
#define ENCB_L 5
#define PWM_L 11
#define IN1_L 12
#define IN2_L 10

volatile long ticksLeft = 0;
volatile long ticksRight = 0;

// PID variables
float error = 0;
float prevError = 0;
float integral = 0;
float derivative = 0;

float Kp = 0.9;
float Ki = -0.01;
float Kd = 0.1;

// Base PWM
int basePWM = 100;

unsigned long startTime;

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  // Encoder pins
  pinMode(ENCA_L, INPUT);
  pinMode(ENCB_L, INPUT);
  pinMode(ENCA_R, INPUT);
  pinMode(ENCB_R, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_L), isrLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_R), isrRight, RISING);

  // Start OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (true);
  }
  display.clearDisplay();
  display.display();

  startTime = millis();
  Serial.println("Error\tPWM_R");  // Header line
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - startTime > 10000) {
    stopMotors();
    //display.clearDisplay();
    display.setCursor(0, 30);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println("Stopped after 10 sec");
    display.display();
    while (true);
  }

  // Calculate PID
  error = ticksLeft - ticksRight;
  integral += error;
  derivative = error - prevError;

  float correction = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;

  int pwmLeft = basePWM;
  int pwmRight = basePWM + correction;

  // Constrain PWM
  pwmLeft = constrain(pwmLeft, 0, 255);
  pwmRight = constrain(pwmRight, 0, 255);

  setMotorSpeed(pwmLeft, pwmRight);

  // Display on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print("Ticks L: ");
  display.println(ticksLeft);
  display.print("Ticks R: ");
  display.println(ticksRight);
  display.print("Err: ");
  display.println(error);
  display.display();

  // Serial for plotting
  /*Serial.print("Ticks_L: ");
  Serial.print(ticksLeft);
  Serial.print(",Ticks_R: ");
  Serial.print(ticksRight);
  Serial.print(",Error: ");
  Serial.print(error);
  Serial.print(",PWM_R: ");
  Serial.println(pwmRight);*/

//Serial.print(ticksLeft);
//Serial.print("\t");
//Serial.print(ticksRight);
//Serial.print("\t");
Serial.print(error);
Serial.print("\t");
Serial.println(pwmRight);

  delay(100);
}

void setMotorSpeed(int pwmL, int pwmR) {
  // Left motor forward
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, LOW);
  analogWrite(PWM_L, pwmL);

  // Right motor forward
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  analogWrite(PWM_R, pwmR);
}

void stopMotors() {
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
}

void isrLeft() {
  ticksLeft++;
}

void isrRight() {
  ticksRight++;
}
