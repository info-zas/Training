
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

MPU6050 mpu(Wire);

// NRF24L01
#define CE_PIN  A3
#define CSN_PIN A2
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t pipe = 0xE8E8F0F0E1LL;
char msg[20];

// Push buttons
int up_button = 7;
int down_button = 6;
int left_button = 4;
int right_button = 5;

// Mode selection
#define BUTTON_PIN 2
#define LED_PIN    10

int lastButtonState = HIGH;
int currentMode = 0;
const int maxModes = 3;
unsigned long lastDebounceTime = 0;

// Joystick
#define JOY_X A0
#define JOY_Y A1
#define JOY_THRESHOLD 200
#define JOY_CENTER 512

String status = "Idle";

void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Mode: ");
  if (currentMode == 0) display.println("Buttons");
  else if (currentMode == 1) display.println("MPU6050");
  else if (currentMode == 2) display.println("Joystick");

  display.setCursor(0, 20);
  display.print("Status: ");
  display.println(status);
  display.display();
}

void setup() {
  pinMode(up_button, INPUT);
  pinMode(down_button, INPUT);
  pinMode(left_button, INPUT);
  pinMode(right_button, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  updateDisplay();

  radio.begin();
  radio.openWritingPipe(pipe);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

void checkModeChange() {
  int buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);

    currentMode = (currentMode + 1) % maxModes;
    status = "Idle";
    updateDisplay();
  }
  lastButtonState = buttonState;
}

void buttonMode() {
  bool anyPressed = false;

  if (digitalRead(up_button) == LOW) {
    strcpy(msg, "up");
    status = "Up";
    radio.write(&msg, strlen(msg) + 1);
    Serial.println("Sent: up");
    updateDisplay();
    delay(300);
    anyPressed = true;
  }
  if (digitalRead(down_button) == LOW) {
    strcpy(msg, "down");
    status = "Down";
    radio.write(&msg, strlen(msg) + 1);
    Serial.println("Sent: down");
    updateDisplay();
    delay(300);
    anyPressed = true;
  }
  if (digitalRead(left_button) == LOW) {
    strcpy(msg, "left");
    status = "Left";
    radio.write(&msg, strlen(msg) + 1);
    Serial.println("Sent: left");
    updateDisplay();
    delay(300);
    anyPressed = true;
  }
  if (digitalRead(right_button) == LOW) {
    strcpy(msg, "right");
    status = "Right";
    radio.write(&msg, strlen(msg) + 1);
    Serial.println("Sent: right");
    updateDisplay();
    delay(300);
    anyPressed = true;
  }

  if (!anyPressed) {
    status = "Idle";
    updateDisplay();
  }
}

void mpuMode() {
  mpu.update();
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();

  if (ax > 0.3) {
    strcpy(msg, "left");
    status = "Left";
    radio.write(&msg, strlen(msg) + 1);
    updateDisplay();
    delay(300);
  } else if (ax < -0.3) {
    strcpy(msg, "right");
    status = "Right";
    radio.write(&msg, strlen(msg) + 1);
    updateDisplay();
    delay(300);
  }

  if (ay > 0.3) {
    strcpy(msg, "up");
    status = "Up";
    radio.write(&msg, strlen(msg) + 1);
    updateDisplay();
    delay(300);
  } else if (ay < -0.3) {
    strcpy(msg, "down");
    status = "Down";
    radio.write(&msg, strlen(msg) + 1);
    updateDisplay();
    delay(300);
  }

  if (abs(ax) < 0.3 && abs(ay) < 0.3) {
    status = "Idle";
    updateDisplay();
  }
}

void joystickMode() {
  int xVal = analogRead(JOY_X);
  int yVal = analogRead(JOY_Y);
  bool moved = false;

  if (yVal > JOY_CENTER + JOY_THRESHOLD) {
    strcpy(msg, "up");
    status = "Up";
    radio.write(&msg, strlen(msg) + 1);
    updateDisplay();
    delay(300);
    moved = true;
  } else if (yVal < JOY_CENTER - JOY_THRESHOLD) {
    strcpy(msg, "down");
    status = "Down";
    radio.write(&msg, strlen(msg) + 1);
    updateDisplay();
    delay(300);
    moved = true;
  }

  if (xVal > JOY_CENTER + JOY_THRESHOLD) {
    strcpy(msg, "right");
    status = "Right";
    radio.write(&msg, strlen(msg) + 1);
    updateDisplay();
    delay(300);
    moved = true;
  } else if (xVal < JOY_CENTER - JOY_THRESHOLD) {
    strcpy(msg, "left");
    status = "Left";
    radio.write(&msg, strlen(msg) + 1);
    updateDisplay();
    delay(300);
    moved = true;
  }

  if (!moved) {
    status = "Idle";
    updateDisplay();
  }
}

void loop() {
  checkModeChange();

  switch (currentMode) {
    case 0: buttonMode(); break;
    case 1: mpuMode(); break;
    case 2: joystickMode(); break;
  }
}
