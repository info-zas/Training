#include <Servo.h>
#include <Stepper.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//
// --- OLED SETUP ---
//
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//
// --- SERVO ---
//
Servo myServo;
const int servoPin = 11;

//
// --- DC MOTOR ---
//
const int pwmPin = 5;
const int dir1 = 6;
const int dir2 = 4;

//
// --- STEPPER MOTOR ---
//
const int stepsPerRevolution = 2048;
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 7);

void showOLED(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println(line1);
  display.setCursor(0, 30);
  display.println(line2);
  display.display();
}

void setup() {

  // DC Motor setup
  pinMode(pwmPin, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  // Servo setup
  myServo.attach(servoPin);

  // Stepper setup
  myStepper.setSpeed(10);

  Serial.begin(9600);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found");
    while (true);
  }

  showOLED("System Ready", "Motors Initializing");
  delay(1500);

  Serial.println("Running Each Motor One by One...");
}

void loop() {

  // ============================
  // 1️⃣ DC MOTOR SECTION
  // ============================
  showOLED("DC Motor", "Forward");
  Serial.println("DC Motor Forward");
  analogWrite(pwmPin, 255);
  digitalWrite(dir1, HIGH);
  digitalWrite(dir2, LOW);
  delay(400);

  showOLED("DC Motor", "Backward");
  Serial.println("DC Motor Backward");
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, HIGH);
  delay(400);

  analogWrite(pwmPin, 0);
  delay(200);

  // ============================
  // 2️⃣ SERVO MOTOR SECTION
  // ============================
  showOLED("Servo Motor", "Sweeping");
  Serial.println("Servo Sweep");

  for (int angle = 0; angle <= 180; angle += 5) {
    myServo.write(angle);
    delay(10);
  }

  for (int angle = 180; angle >= 0; angle -= 5) {
    myServo.write(angle);
    delay(10);
  }

  delay(200);

  // ============================
  // 3️⃣ STEPPER MOTOR SECTION
  // ============================
  showOLED("Stepper Motor", "Clockwise");
  Serial.println("Stepper Clockwise");
  myStepper.step(400);
  delay(200);

  showOLED("Stepper Motor", "Counter-Clockwise");
  Serial.println("Stepper Counterclockwise");
  myStepper.step(-400);
  delay(200);

  showOLED("Cycle Done", "Restarting...");
  Serial.println("Cycle Done\n");
  delay(800);
}
