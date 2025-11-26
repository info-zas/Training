#include <SPI.h>
#include <RF24.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------------------- Motor Pins (given) --------------------
#define IN1 7   // Left motor forward
#define IN2 8   // Left motor backward
#define ENA 6   // Left motor PWM

#define IN3 3   // Right motor forward
#define IN4 4   // Right motor backward
#define ENB 5   // Right motor PWM

// -------------------- NRF24L01 Setup --------------------
#define CE_PIN A3
#define CSN_PIN A2
RF24 radio(CE_PIN, CSN_PIN);

// Must match transmitter’s address
//const byte address[6] = "00001";
const uint64_t pipe = 0xE8E8F0F0E1LL;

// -------------------- Motor Functions --------------------
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}

void forward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
}

void backward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
}

void right() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);

   /// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

   display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("System Initialized");
  display.display();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotors();

  radio.begin();
  radio.openReadingPipe(0, pipe);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
}

// -------------------- Loop --------------------
void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));

    String cmd = String(text);
    cmd.toLowerCase();  // normalize

    Serial.print("Command: ");
    Serial.println(cmd);

    if (cmd.indexOf("up") == 0) {
      showText("forward");
      forward();
    } else if (cmd.indexOf("down") == 0) {
      showText("backward");
      backward();
    } else if (cmd.indexOf("left") == 0) {
      showText("left");
      left();
    } else if (cmd.indexOf("right") == 0) {
      showText("right");
      right();
    } else {
      showText("stop");
      stopMotors();
    }
  }
}

void showText(const char* text) {
    Serial.println(text);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Current Mode:");
    display.println(text);
    display.display();
  
}
