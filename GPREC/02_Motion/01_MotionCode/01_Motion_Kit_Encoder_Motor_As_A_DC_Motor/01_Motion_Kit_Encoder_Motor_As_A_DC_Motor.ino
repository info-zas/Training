void setup() { 
  pinMode(5, OUTPUT);  // PWM speed control
  pinMode(6, OUTPUT);  // Direction pin 1
  pinMode(4, OUTPUT);  // Direction pin 2
}

void loop() {
 analogWrite(5, 255);   // Speed (0–255), adjust as needed

  forward();
  delay(1000);

  backward();
  delay(1000);
}

// Function to move motor forward
void forward() {
  digitalWrite(6, HIGH); // Forward direction
  digitalWrite(4, LOW);
}

// Function to move motor backward
void backward() {
  digitalWrite(6, LOW);  // Reverse direction
  digitalWrite(4, HIGH);
}
