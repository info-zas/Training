#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// === Right Motor Pins ===
#define IN1_R 12
#define IN2_R 10
#define PWM_R 11
#define ENCA_R 3
#define ENCB_R 5

// === Left Motor Pins ===
#define IN1_L 7
#define IN2_L 8
#define PWM_L 6
#define ENCA_L 2
#define ENCB_L 4

// === Encoder Counts ===
volatile long encoderCountR = 0;
volatile long encoderCountL = 0;

// === Encoder ISRs with Direction Detection ===
void readEncoderR() {
  if (digitalRead(ENCB_R) == LOW)
    encoderCountR--;
  else
    encoderCountR++;
}

void readEncoderL() {
  if (digitalRead(ENCB_L) == LOW)
    encoderCountL++;
  else
    encoderCountL--;
}

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT); pinMode(PWM_R, OUTPUT);
  pinMode(IN1_L, OUTPUT); pinMode(IN2_L, OUTPUT); pinMode(PWM_L, OUTPUT);

  // Encoder pins
  pinMode(ENCA_R, INPUT);
  pinMode(ENCB_R, INPUT);
  pinMode(ENCA_L, INPUT);
  pinMode(ENCB_L, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_R), readEncoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), readEncoderL, RISING);

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
    while (true);
  }
  display.clearDisplay();
  display.display();

  // Start both motors forward
  digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW);
  analogWrite(PWM_R, 180);

  digitalWrite(IN1_L, HIGH); digitalWrite(IN2_L, LOW);
  analogWrite(PWM_L, 180);

  Serial.println("Motors running. Encoders with direction...");
}

void loop() {
  // Serial monitor
  Serial.print("Right: "); Serial.print(encoderCountR);
  Serial.print(" | Left: "); Serial.println(encoderCountL);

  // OLED display
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("R Cnt: "); display.println(encoderCountR);
  display.print("L Cnt: "); display.println(encoderCountL);
  display.display();

  delay(300);
}
