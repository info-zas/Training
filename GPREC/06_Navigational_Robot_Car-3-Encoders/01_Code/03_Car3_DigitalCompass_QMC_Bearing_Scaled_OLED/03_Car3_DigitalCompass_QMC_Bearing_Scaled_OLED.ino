#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Magnetometer ---
QMC5883LCompass compass;

// --- Calibration (runtime min/max) ---
int16_t xMin = 32767, xMax = -32768;
int16_t yMin = 32767, yMax = -32768;
int16_t zMin = 32767, zMax = -32768;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    // If this fails, reduce what you print or switch to U8g2 low-memory mode
    while (1);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("QMC5883L Calib"));
  display.println(F("Rotate in all axes"));
  display.display();

  compass.init();
  delay(800);
}

void loop() {
  compass.read();
  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();

  // --- Update min/max ---
  if (x < xMin) xMin = x; if (x > xMax) xMax = x;
  if (y < yMin) yMin = y; if (y > yMax) yMax = y;
  if (z < zMin) zMin = z; if (z > zMax) zMax = z;

  // --- Offsets (hard-iron) ---
  float xOff = (xMax + xMin) * 0.5f;
  float yOff = (yMax + yMin) * 0.5f;
  float zOff = (zMax + zMin) * 0.5f;

  // --- Scales (soft-iron) ---
  float xScale = (xMax - xMin) * 0.5f;
  float yScale = (yMax - yMin) * 0.5f;
  float zScale = (zMax - zMin) * 0.5f;

  // Use averaged scale to keep math light on Uno
  float avgScale = (xScale + yScale + zScale) / 3.0f;
  if (avgScale < 1.0f) avgScale = 1.0f;  // safety

  // --- Calibrated & normalized readings ---
  float xCal = (x - xOff) / avgScale;
  float yCal = (y - yOff) / avgScale;
  // zCal not needed for 2D heading on a level surface

  // --- Bearing from calibrated X/Y ---
  float bearing = atan2f(yCal, xCal) * 180.0f / PI;
  if (bearing < 0) bearing += 360.0f;

  // --- OLED output (compact) ---
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(F("Bear: ")); display.print(bearing, 1); display.println(F(" deg"));

  // Show calibrated values briefly (helps verify offsets)
  display.print(F("Xc:")); display.print(xCal, 2);
  display.print(F(" Yc:")); display.println(yCal, 2);

  display.print(F("Xmn:")); display.print(xMin);
  display.print(F(" Xmx:")); display.println(xMax);
  display.print(F("Ymn:")); display.print(yMin);
  display.print(F(" Ymx:")); display.println(yMax);
  display.print(F("Zmn:")); display.print(zMin);
  display.print(F(" Zmx:")); display.println(zMax);

  display.display();

  delay(250);
}
