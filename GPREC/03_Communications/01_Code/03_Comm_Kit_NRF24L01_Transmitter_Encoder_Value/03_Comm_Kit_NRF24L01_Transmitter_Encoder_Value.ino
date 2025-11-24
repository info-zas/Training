#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// === NRF24L01 Setup ===
//RF24 radio(9, 10); // CE, CSN (Change if needed)

// --- NRF24L01 Setup ---
#define CE_PIN 4
#define CSN_PIN 5
RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001";

// === Encoder Pins ===
#define CLK 26
#define DT 27
#define SW 33

int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir = "";
unsigned long lastButtonPress = 0;

void setup() {
  Serial.begin(9600);

  // Encoder pins
  pinMode(CLK, INPUT);
  pinMode(DT, INPUT);
  pinMode(SW, INPUT_PULLUP);
  lastStateCLK = digitalRead(CLK);

  // NRF24L01
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setChannel(100);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(address);
  radio.stopListening();

  Serial.println("Encoder + NRF24L01 Transmitter Ready");
}

void loop() {
  // Read encoder
  currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1) {
    if (digitalRead(DT) != currentStateCLK) {
      counter--;
      currentDir = "CCW";
    } else {
      counter++;
      currentDir = "CW";
    }

    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);

    // Send over nRF24L01
    radio.write(&counter, sizeof(counter));
  }
  lastStateCLK = currentStateCLK;

  // Check button press (optional)
  int btnState = digitalRead(SW);
  if (btnState == LOW && millis() - lastButtonPress > 200) {
    Serial.println("Encoder button pressed");
    lastButtonPress = millis();
    // You could send a special value or reset the counter, etc.
  }

  delay(1);
}
