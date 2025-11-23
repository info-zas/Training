#define Left1 3
#define Left2 4
#define Left_Speed 5

#define Right1 8
#define Right2 7
#define Right_Speed 6

void setup() {
  pinMode(Left1, OUTPUT);
  pinMode(Left2,OUTPUT);
  pinMode(Left_Speed,OUTPUT); //pwm

  pinMode(Right1, OUTPUT);
  pinMode(Right2,OUTPUT);
  pinMode(Right_Speed,OUTPUT); //pwm


  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
 
  Serial.begin(9600);
}

void loop(){
  analogWrite(Left_Speed,127);
  analogWrite(Right_Speed,127);
  
  int L = digitalRead(A0);
  int C1 = digitalRead(A1);
  int C2 = digitalRead(A2);
  int R = digitalRead(A3);


  if(L == LOW && C1 == LOW){
  stpLeft();
}
else if(L == HIGH && C1 == LOW){
  stpRight();
}
else if(L == LOW && C1 == HIGH){
  turnRight();
  delay(500);
}
else if(L == HIGH && C1 == HIGH){
  turnRight();
  delay(500);
}

}

void forward(){  //forword
digitalWrite(Left1, HIGH);  
digitalWrite(Left2, LOW);  
digitalWrite(Right1, HIGH);  
digitalWrite(Right2, LOW); 
}

void backward(){  //backward
digitalWrite(Left1, LOW);  
digitalWrite(Left2, HIGH);  
digitalWrite(Right1, LOW);  
digitalWrite(Right2, HIGH); 
}

void turnRight(){ //turnRight
digitalWrite(Left1, HIGH);  
digitalWrite(Left2, LOW);  
digitalWrite(Right1, LOW);  
digitalWrite(Right2, HIGH); 
}

void stpRight() { 
  digitalWrite(Left1, HIGH); 
  digitalWrite(Left2, LOW);
  digitalWrite(Right1, LOW);   
  digitalWrite(Right2, LOW); 
}

void turnLeft(){ //turnLeft
digitalWrite(Left1, LOW);  
digitalWrite(Left2, HIGH);  
digitalWrite(Right1, HIGH);  
digitalWrite(Right2, LOW); 
}

void stpLeft() { 
  digitalWrite(Left1, LOW); 
  digitalWrite(Left2, LOW);
  digitalWrite(Right1, HIGH);   
  digitalWrite(Right2, LOW); 
}

void Stop(){ //turnLeft
digitalWrite(Left1, LOW);  
digitalWrite(Left2, LOW);  
digitalWrite(Right1, LOW);  
digitalWrite(Right2, LOW); 
}







