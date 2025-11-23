#include <SPI.h>
#include <RF24.h>

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
      forward();
    } else if (cmd.indexOf("down") == 0) {
      backward();
    } else if (cmd.indexOf("left") == 0) {
      left();
    } else if (cmd.indexOf("right") == 0) {
      right();
    } else {
      stopMotors();
    }
  }
}
