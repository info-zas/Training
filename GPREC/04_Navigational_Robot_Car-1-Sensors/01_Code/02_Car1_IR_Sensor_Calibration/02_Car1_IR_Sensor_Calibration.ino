void setup() {
  pinMode(A1,INPUT);
  Serial.begin(9600);
}

void loop() {
  int data = digitalRead(A1);
  Serial.println(data);
}
