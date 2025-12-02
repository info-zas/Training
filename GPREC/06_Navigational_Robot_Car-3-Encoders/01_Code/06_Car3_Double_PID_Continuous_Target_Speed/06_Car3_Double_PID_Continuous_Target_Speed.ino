#include <U8x8lib.h>

// ---------------- OLED (Text-Only Mode) ----------------
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8;   // Use if SH1106
// If SSD1306 display, use this instead:
// U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8;

// ---------------- Motor Pins ---------------------------
#define ENCA_L 2
#define ENCB_L 4
#define PWM_L  6
#define IN1_L  7
#define IN2_L  8

#define ENCA_R 3
#define ENCB_R 5
#define PWM_R  11
#define IN1_R  12
#define IN2_R  10

#define STBY   9

// ---------------- Encoder Counters ---------------------
volatile long ticksR = 0;
volatile long ticksL = 0;

// ---------------- Timing -------------------------------
unsigned long lastTime = 0;

// ---------------- Speed Measurement --------------------
float speedL = 0;
float speedR = 0;

float filtL = 0;
float filtR = 0;

float baseSpeed = 250;  // ticks/sec target
float targetL = 250;
float targetR = 250;

// ---------------- Velocity PID (Inner Loop) ------------
float Kp_v = 0.45;
float Ki_v = 0.004;
float Kd_v = 0.06;

float errL_prev = 0, errR_prev = 0;
float intL = 0, intR = 0;

// ---------------- Straightness PID (Outer Loop) --------
float Kp_s = 0.7;
float Ki_s = 0.002;
float Kd_s = 0.17;

float err_s_prev = 0;
float err_s_int = 0;

int pwmL = 0;
int pwmR = 0;

// ---------------- Encoder Interrupts -------------------
void readEncoderR() {
  int b = digitalRead(ENCB_R);
  ticksR += (b ? 1 : -1);
}

void readEncoderL() {
  int b = digitalRead(ENCB_L);
  ticksL += (b ? 1 : -1);
}

// ---------------- Motor Control ------------------------
void setMotorL(int pwm) {
  if (pwm >= 0) {
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
  } else {
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
    pwm = -pwm;
  }
  analogWrite(PWM_L, constrain(pwm, 0, 155));
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
  analogWrite(PWM_R, constrain(pwm, 0, 155));
}

// =======================================================
void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // Encoder pins
  pinMode(ENCA_R, INPUT_PULLUP);
  pinMode(ENCB_R, INPUT_PULLUP);
  pinMode(ENCA_L, INPUT_PULLUP);
  pinMode(ENCB_L, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA_R), readEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_L), readEncoderL, CHANGE);


  // OLED
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  lastTime = millis();
}

// =======================================================
void loop() {

  if (millis() - lastTime >= 200) {   // 200 ms loop = 0.2 sec

    long r = ticksR;
    long l = ticksL;
    ticksR = 0;
    ticksL = 0;

    // Convert to ticks/sec
    speedL = l * 5;
    speedR = r * 5;

    // Exponential moving average filtering
    filtL = filtL * 0.7 + speedL * 0.3;
    filtR = filtR * 0.7 + speedR * 0.3;

    float leftSpeed  = filtL;
    float rightSpeed = filtR;

    // ----------------- OUTER PID (straight-line) -----------------
    float err_s = leftSpeed - rightSpeed;
    err_s_int += err_s * 0.2;
    float d_s = (err_s - err_s_prev) / 0.2;
    err_s_prev = err_s;

    float correction = Kp_s * err_s + Ki_s * err_s_int + Kd_s * d_s;

    targetL = baseSpeed - correction;
    targetR = baseSpeed + correction;

    // ----------------- INNER PID (Velocity Left) -----------------
    float errL = targetL - leftSpeed;
    intL += errL * 0.2;
    float dL = (errL - errL_prev) / 0.2;
    errL_prev = errL;

    pwmL = Kp_v * errL + Ki_v * intL + Kd_v * dL;

    // ----------------- INNER PID (Velocity Right) ----------------
    float errR = targetR - rightSpeed;
    intR += errR * 0.2;
    float dR = (errR - errR_prev) / 0.2;
    errR_prev = errR;

    pwmR = Kp_v * errR + Ki_v * intR + Kd_v * dR;

    setMotorL(pwmL);
    setMotorR(pwmR);

    // ----------------- Serial Plotter ----------------------------
    Serial.print("Err:");
    Serial.println(err_s);
    Serial.print(",Zero:0");
    Serial.print(",Max:");
    Serial.print(20);   // top of y-axis

    Serial.print(",Min:");
    Serial.println(-20);  // bottom of y-axis

    // ----------------- OLED Display ------------------------------
    u8x8.clearDisplay();
    u8x8.setCursor(0,0); u8x8.print("L:");  u8x8.print(leftSpeed);
    u8x8.setCursor(0,1); u8x8.print("R:");  u8x8.print(rightSpeed);
    u8x8.setCursor(0,2); u8x8.print("E:");  u8x8.print(err_s);
    u8x8.setCursor(0,3); u8x8.print("PL:"); u8x8.print(pwmL);
    u8x8.setCursor(0,4); u8x8.print("PR:"); u8x8.print(pwmR);

    lastTime = millis();
  }
}
