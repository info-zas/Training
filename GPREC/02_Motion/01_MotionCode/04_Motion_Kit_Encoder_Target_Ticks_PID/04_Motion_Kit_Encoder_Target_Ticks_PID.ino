#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------- OLED Configuration --------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- Motor Driver Pins --------------------
const int PWMA = 5;
const int AIN1 = 6;
const int AIN2 = 4;

// -------------------- Encoder Pin --------------------
const int ENCA = 3;

// -------------------- Encoder Tracking --------------------
volatile long encoderTicks = 0;

// -------------------- PID Settings --------------------
const long targetTicks = 5000;
float Kp = 0.1;
float Ki = 0.001;
float Kd = 0.1;

long previousError = 0;
float integral = 0;

const int maxPWM = 150;
const int minPWM = 30;

unsigned long lastTime = 0;
bool motorStopped = false;

void readEncoder() {
  encoderTicks++;
}

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(ENCA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // --- OLED setup ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);  // hang if OLED not found
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("PID Start");
  display.display();
  delay(1000);
}

void loop() {
  noInterrupts();
  long ticks = encoderTicks;
  interrupts();

  long error = targetTicks - ticks;
  long tolerance = targetTicks * 0.005;
  

  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0;  // seconds

  // Stop when within tolerance
  if (abs(error) <= tolerance) {
    if (!motorStopped) {
      analogWrite(PWMA, 0);
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      motorStopped = true;
    }
  } else {
    // PID calculations
    integral += error * deltaTime;
    float derivative = (error - previousError) / deltaTime;

    float output = Kp * error + Ki * integral + Kd * derivative;

    int pwm = abs(output);
    pwm = constrain(pwm, minPWM, maxPWM);

    // Set direction CW
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);

    previousError = error;
    lastTime = now;
    motorStopped = false;
  }

  // -------------------- OLED DISPLAY --------------------
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Ticks(5K):");
  display.setCursor(0, 20);
  display.println(ticks);

  display.setCursor(0, 45);
  display.print("Err:");
  display.println(error);

  display.display();

  delay(100);
}