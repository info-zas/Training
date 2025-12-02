#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// QMC5883P I2C
const int QMC5883P_ADDR = 0x2C;


const int MODE_REG   = 0x0A;
const int CONFIG_REG = 0x0B;


const int X_LSB_REG = 0x01;
const int X_MSB_REG = 0x02;
const int Y_LSB_REG = 0x03;
const int Y_MSB_REG = 0x04;
const int Z_LSB_REG = 0x05;
const int Z_MSB_REG = 0x06;


void initQMC5883P() {
  Wire.begin();

  // Set continuous mode (200 Hz)
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(MODE_REG);
  Wire.write(0xCF);
  Wire.endTransmission();

  // Enable Set/Reset + ±8G
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(CONFIG_REG);
  Wire.write(0x08);
  Wire.endTransmission();
}


void readQMC5883PData(int16_t& x, int16_t& y, int16_t& z) {
  Wire.beginTransmission(QMC5883P_ADDR);
  Wire.write(X_LSB_REG);
  Wire.endTransmission(false);

  Wire.requestFrom(QMC5883P_ADDR, 6);

  if (Wire.available() == 6) {
    byte x_lsb = Wire.read();
    byte x_msb = Wire.read();
    byte y_lsb = Wire.read();
    byte y_msb = Wire.read();
    byte z_lsb = Wire.read();
    byte z_msb = Wire.read();

    x = (int16_t)((x_msb << 8) | x_lsb);
    y = (int16_t)((y_msb << 8) | y_lsb);
    z = (int16_t)((z_msb << 8) | z_lsb);
  }
}


float computeHeading(int16_t x, int16_t y) {
  float heading = atan2((float)y, (float)x);
  heading = heading * (180.0 / PI);
  if (heading < 0) heading += 360.0;
  return heading;
}

void setup() {
  Serial.begin(9600);
  initQMC5883P();

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    while (1);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("QMC5883P Compass");
  display.display();
  delay(1000);
}

void loop() {
  int16_t x, y, z;
  readQMC5883PData(x, y, z);

  float heading = computeHeading(x, y);

  // --- Serial Output (optional) ---
  Serial.print("X: "); Serial.print(x);
  Serial.print("\tY: "); Serial.print(y);
  Serial.print("\tZ: "); Serial.print(z);
  Serial.print("\tHeading: "); Serial.println(heading, 2);

  // --- OLED Display ---
  display.clearDisplay();
  display.setCursor(0, 0);

  display.setTextSize(1);
  display.print("X: "); display.print(x);
  display.print(" Y: "); display.print(y);
  display.print(" Z: "); display.println(z);

  display.setTextSize(2);
  display.setCursor(0, 30);
  display.print("HDG: ");
  display.println(heading, 1);

  display.display();

  delay(100);
}
