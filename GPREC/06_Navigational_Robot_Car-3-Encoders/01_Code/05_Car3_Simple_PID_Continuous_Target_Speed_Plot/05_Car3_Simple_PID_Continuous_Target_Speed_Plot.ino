#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Right motor pins
#define ENCA_R 2
#define ENCB_R 4
#define PWM_R 6
#define IN1_R 7
#define IN2_R 8

// Left motor pins
#define ENCA_L 3
#define ENCB_L 5
#define PWM_L 11
#define IN1_L 12
#define IN2_L 10

// TB6612 STANDBY pin
#define STBY 9

// ------------------------------------
volatile long ticksR = 0;
volatile long ticksL = 0;

unsigned long lastTime = 0;

float speedL = 0;
float speedR = 0;

float targetSpeed = 250.0;   // <-- ticks per second

// PID constants
float Kp = 0.5;
float Ki = 0.003;
float Kd = 0.10;

float error = 0;
float prevError = 0;
float integral = 0;

int pwmBase = 100;
int pwmL, pwmR;

// ------------------------------------
void  readEncoderR() {
  int b = digitalRead(ENCB_R);
  if (b == HIGH) ticksR--;
  else ticksR++;
}

void  readEncoderL() {
  int b = digitalRead(ENCB_L);
  if (b == HIGH) ticksL++;
  else ticksL--;
}

// ------------------------------------
void setMotorL(int pwm) {
  if (pwm >= 0) {
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  } else {
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
    pwm = -pwm;
  }
  analogWrite(PWM_L, constrain(pwm,0,155));
}

void setMotorR(int pwm) {
  if (pwm >= 0) {
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
  } else {
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
    pwm = -pwm;
  }
  analogWrite(PWM_R, constrain(pwm,0,155));
}

// ------------------------------------
void setup() {

  Serial.begin(115200);

  // motor pins
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // encoder pins
  pinMode(ENCA_R, INPUT_PULLUP);
  pinMode(ENCB_R, INPUT_PULLUP);
  pinMode(ENCA_L, INPUT_PULLUP);
  pinMode(ENCB_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_R), readEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), readEncoderL, CHANGE);

  // OLED init
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();

  lastTime = millis();
}

// ------------------------------------
void loop() {

  // measure speed every 100 ms
  if (millis() - lastTime >= 100) {

    // ticks counted in 100 ms → convert to ticks/sec
    long r = ticksR;
    long l = ticksL;
    ticksR = 0;
    ticksL = 0;

    speedR = r * 10;   // because 100 ms × 10 = 1 sec
    speedL = l * 10;

    // PID for straight-line
    error = speedL - speedR;    // positive → right slower

    integral += error * 0.1;
    float derivative = (error - prevError) / 0.1;
    prevError = error;

    float correction = Kp * error + Ki * integral + Kd * derivative;

    // Both motors attempt to maintain base speed first
    pwmL = pwmBase - correction;
    pwmR = pwmBase + correction;

    // extra: adjust PWM to chase 250 ticks/sec
    pwmL += (targetSpeed - speedL) * 0.1;
    pwmR += (targetSpeed - speedR) * 0.1;

    setMotorL(pwmL);
    setMotorR(pwmR);

    // ---------------- Serial Plotter ----------------
    Serial.print("Error:");
    Serial.print(error);
     Serial.print(",Zero:0");
    Serial.print(",Max:");
    Serial.print(50);   // top of y-axis

    Serial.print(",Min:");
    Serial.println(-50);  // bottom of y-axis
    // ---------------- OLED display -------------------
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);

    display.print("L:");
    display.println(speedL);
    display.print("R:");
    display.println(speedR);
    display.print("Err:");
    display.println(error);
    display.print("PWM L:");
    display.println(pwmL);
    display.print("PWM R:");
    display.println(pwmR);

    display.display();

    lastTime = millis();
  }
}
