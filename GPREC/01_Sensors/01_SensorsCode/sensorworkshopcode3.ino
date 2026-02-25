// === ZAS Robotics: Touch counter + NeoPixel pattern cycler ===
// Cycle order per touch: WHITE -> RAINBOW -> COLORWIPE -> THEATER -> CYCLON -> FADE -> OFF -> ...
// Board: Arduino UNO

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <DHT.h>

// ---------------- Pins ----------------
#define BUZZER_PIN    12
#define TRIG_PIN      A1
#define ECHO_PIN      A2
#define TOUCH_PIN     4        // TTP223 SIG
#define IR_PIN        2
#define NEOPIXEL_PIN  5
#define DHT_PIN       8        // optional (yours may be faulty)
#define DHT_TYPE      DHT11

// ------------- NeoPixel ----------------
#define NUMPIXELS     8        // your ring in the photo is 8 LEDs
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// --------------- OLED ------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --------------- DHT (optional) --------
DHT dht(DHT_PIN, DHT_TYPE);

// ------------- Behavior settings -------
const long  DIST_BEEP_CM = 20;
const bool  IR_ACTIVE_LOW = true;
const unsigned long UI_MS = 300;
const unsigned long BEEP_MS = 100;

const unsigned long LED_MS_RAINBOW = 40;
const unsigned long LED_MS_WIPE    = 60;
const unsigned long LED_MS_THEATER = 80;
const unsigned long LED_MS_CYCLON  = 60;
const unsigned long LED_MS_FADE    = 20;

const unsigned long DHT_INTERVAL_MS = 2500;

// ---- Touch polarity (set to 1 for active-HIGH boards like yours) ----
#define TOUCH_ACTIVE_HIGH 1
#if TOUCH_ACTIVE_HIGH
  #define TOUCH_PIN_MODE   INPUT
  #define TOUCH_PRESSED(raw) ((raw)==HIGH)
  const char* TOUCH_HINT = "Touch=HIGH (active-HIGH)";
#else
  #define TOUCH_PIN_MODE   INPUT_PULLUP
  #define TOUCH_PRESSED(raw) ((raw)==LOW)
  const char* TOUCH_HINT = "Touch=LOW  (active-LOW w/ pullup)";
#endif

// ------------- LED patterns -----------
enum LedMode {
  LED_WHITE = 0,
  LED_RAINBOW,
  LED_COLORWIPE,
  LED_THEATER,
  LED_CYCLON,
  LED_FADE,
  LED_OFF,
  LED_COUNT
};
LedMode ledMode = LED_WHITE;

const char* modeName(LedMode m) {
  switch (m) {
    case LED_WHITE:    return "WHITE";
    case LED_RAINBOW:  return "RAINBOW";
    case LED_COLORWIPE:return "COLORWIPE";
    case LED_THEATER:  return "THEATER";
    case LED_CYCLON:   return "CYCLON";
    case LED_FADE:     return "FADE";
    default:           return "OFF";
  }
}

// -------------- State / timers --------
unsigned long t_ui=0, t_beepOff=0, t_led=0, t_dht=0;
float lastTempC = NAN, lastHum = NAN;

unsigned long touchCount = 0;
const unsigned long DEBOUNCE_MS = 80;
bool lastPressed = false;
int  prevRaw = -1;
unsigned long lastEdgeMs = 0;

struct {
  uint8_t hue = 0;
  uint8_t wipeIdx = 0, wipePhase = 0;
  uint8_t chaseOfs = 0;
  int8_t  cyclonIdx = 0, cyclonDir = 1;
  int16_t fadeLevel = 0; int8_t fadeDir = 5;
} led;

int lastIrState = -1;

// ================= Helpers =================
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (dur == 0) return NAN;
  return dur * 0.0343f / 2.0f;
}

void beepNow(unsigned long ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  t_beepOff = millis() + ms;
}

void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (int i=0;i<NUMPIXELS;i++) pixels.setPixelColor(i, pixels.Color(r,g,b));
  pixels.show();
}

uint32_t wheel(uint8_t pos) {
  pos = 255 - pos;
  if (pos < 85)  return pixels.Color(255 - pos * 3, 0, pos * 3);
  if (pos < 170) { pos -= 85; return pixels.Color(0, pos * 3, 255 - pos * 3); }
  pos -= 170;     return pixels.Color(pos * 3, 255 - pos * 3, 0);
}

void resetPattern(LedMode m) {
  t_led = 0;
  led = {};
  pixels.clear(); pixels.show();
  if (m == LED_WHITE) setAllPixels(255,255,255);
}

void animatePixels(unsigned long now) {
  switch (ledMode) {
    case LED_OFF:   break;
    case LED_WHITE: break;

    case LED_RAINBOW:
      if (now - t_led >= LED_MS_RAINBOW) {
        for (int i=0;i<NUMPIXELS;i++) {
          uint8_t pos = (i * 256 / NUMPIXELS + led.hue) & 255;
          pixels.setPixelColor(i, wheel(pos));
        }
        pixels.show();
        led.hue = (led.hue + 3) & 255;
        t_led = now;
      }
      break;

    case LED_COLORWIPE:
      if (now - t_led >= LED_MS_WIPE) {
        uint32_t c = (led.wipePhase==0)?pixels.Color(255,0,0):
                     (led.wipePhase==1)?pixels.Color(0,255,0):pixels.Color(0,0,255);
        pixels.setPixelColor(led.wipeIdx, c);
        pixels.show();
        if (++led.wipeIdx >= NUMPIXELS) {
          led.wipeIdx = 0;
          led.wipePhase = (led.wipePhase + 1) % 3;
          pixels.clear();
        }
        t_led = now;
      }
      break;

    case LED_THEATER:
      if (now - t_led >= LED_MS_THEATER) {
        pixels.clear();
        for (int i = led.chaseOfs; i < NUMPIXELS; i += 3)
          pixels.setPixelColor(i, pixels.Color(255,255,255));
        pixels.show();
        led.chaseOfs = (led.chaseOfs + 1) % 3;
        t_led = now;
      }
      break;

    case LED_CYCLON:
      if (now - t_led >= LED_MS_CYCLON) {
        pixels.clear();
        pixels.setPixelColor(led.cyclonIdx, pixels.Color(255,0,0));
        if (led.cyclonIdx-1 >= 0) pixels.setPixelColor(led.cyclonIdx-1, pixels.Color(40,0,0));
        if (led.cyclonIdx+1 < NUMPIXELS) pixels.setPixelColor(led.cyclonIdx+1, pixels.Color(40,0,0));
        pixels.show();
        led.cyclonIdx += led.cyclonDir;
        if (led.cyclonIdx <= 0 || led.cyclonIdx >= NUMPIXELS-1) led.cyclonDir = -led.cyclonDir;
        t_led = now;
      }
      break;

    case LED_FADE:
      if (now - t_led >= LED_MS_FADE) {
        led.fadeLevel += led.fadeDir;
        if (led.fadeLevel >= 255) { led.fadeLevel = 255; led.fadeDir = -led.fadeDir; }
        if (led.fadeLevel <= 0)   { led.fadeLevel = 0;   led.fadeDir = -led.fadeDir; }
        setAllPixels(led.fadeLevel, led.fadeLevel, led.fadeLevel);
        t_led = now;
      }
      break;
  }
}

// ================= Setup =================
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== Touch counter + NeoPixel cycler ==="));
  Serial.println(TOUCH_HINT);

  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(TRIG_PIN, OUTPUT);   digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(TOUCH_PIN, TOUCH_PIN_MODE);

  pixels.begin();
  pixels.setBrightness(60);
  pixels.clear(); pixels.show();

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println(F("ZAS Robotics Demo"));
  display.println(F("Touch = next pattern"));
  display.display();

  pinMode(DHT_PIN, INPUT); // harmless even if sensor is bad
  dht.begin();

  resetPattern(ledMode);
}

// ================= Loop ==================
void loop() {
  unsigned long now = millis();

  // --- Distance & IR
  float dist = readDistanceCM();
  bool irRaw = digitalRead(IR_PIN);
  bool irDetect = IR_ACTIVE_LOW ? !irRaw : irRaw;

  // --- Touch handling (debounced)
  int raw = digitalRead(TOUCH_PIN);             // HIGH on touch if ACTIVE_HIGH
  if (raw != prevRaw) {
    Serial.print(F("[TOUCH] raw=")); Serial.println(raw);
    prevRaw = raw;
  }
  bool pressed = TOUCH_PRESSED(raw);
  if (pressed != lastPressed && (now - lastEdgeMs) > DEBOUNCE_MS) {
    lastPressed = pressed; lastEdgeMs = now;
    if (pressed) {
      touchCount++;
      ledMode = (LedMode)((ledMode + 1) % LED_COUNT);
      resetPattern(ledMode);
      Serial.print(F("[TOUCH] Count=")); Serial.print(touchCount);
      Serial.print(F("  Mode=")); Serial.println(modeName(ledMode));
    }
  }

  // --- Buzzer
  if ((dist == dist && dist <= DIST_BEEP_CM) || irDetect) beepNow(BEEP_MS);
  if (t_beepOff && now >= t_beepOff) { digitalWrite(BUZZER_PIN, LOW); t_beepOff = 0; }

  // --- NeoPixel animation
  animatePixels(now);

  // --- (optional) DHT read
  if (now - t_dht >= DHT_INTERVAL_MS) {
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t) && !isnan(h)) { lastTempC = t; lastHum = h; }
    t_dht = now;
  }

  // --- OLED UI
  if (now - t_ui >= UI_MS) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println(F("ZAS Robotics - Live"));

    display.setTextSize(2);
    display.print(F("D: "));
    if (dist == dist) { display.print(dist, 1); display.println(F("cm")); }
    else              { display.println(F("--")); }

    display.setTextSize(1);
    display.print(F("IR: "));
    display.println(irDetect ? F("OBSTACLE") : F("CLEAR"));

    display.print(F("Touch#: "));
    display.println((unsigned long)touchCount);

    display.print(F("LED: "));
    display.println(modeName(ledMode));

    display.print(F("Temp: "));
    if (!isnan(lastTempC)) { display.print(lastTempC, 1); display.println(F("C")); }
    else                   { display.println(F("--")); }

    display.print(F("Hum : "));
    if (!isnan(lastHum))   { display.print(lastHum, 0); display.println(F("%")); }
    else                   { display.println(F("--")); }

    display.display();

    // console snapshot
    Serial.print(F("[SNAP] touch#=")); Serial.print(touchCount);
    Serial.print(F(" mode=")); Serial.print(modeName(ledMode));
    Serial.print(F(" raw=")); Serial.print(raw);
    Serial.print(F(" dist="));
    if (dist == dist) Serial.print(dist,1); else Serial.print(F("--"));
    Serial.println(F("cm"));
    t_ui = now;
  }
}