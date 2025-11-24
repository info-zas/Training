#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------------------- OLED Configuration --------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- Pins --------------------
const int ENCA = 3;   // Encoder A (interrupt-capable)
const int DIR  = 12;  // Direction pin (external input or controlled)

// -------------------- Encoder Tracking --------------------
volatile long encoderTicks = 0;
volatile int lastDirection = 1;  // 1 = CW, -1 = CCW

void readEncoder() {
  if (digitalRead(DIR)) {
    encoderTicks++;
    lastDirection = 1;
  } else {
    encoderTicks--;
    lastDirection = -1;
  }
}

void setup() {
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(DIR, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  // --- OLED SETUP ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);  // Hang if OLED not found
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Ready...");
  display.display();
  delay(1000);
}

void loop() {
  // Copy volatile vars safely
  noInterrupts();
  long ticks = encoderTicks;
  int dir = lastDirection;
  interrupts();

  // Display values
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Ticks:");
  display.setCursor(0, 20);
  display.println(ticks);

  display.setCursor(0, 45);
  display.print("Dir: ");
  display.println(dir == 1 ? "CW" : "CCW");

  display.display();
  delay(200);
}
