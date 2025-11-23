#define Left1 7
#define Left2 8
#define Left_Speed 6

#define Right1 3
#define Right2 4
#define Right_Speed 5

void setup() {
  pinMode(Left1, OUTPUT);
  pinMode(Left2,OUTPUT);
  pinMode(Left_Speed,OUTPUT); //pwm

  pinMode(Right1, OUTPUT);
  pinMode(Right2,OUTPUT);
  pinMode(Right_Speed,OUTPUT); //pwm

}

void loop() {
  analogWrite(Left_Speed,150);
  analogWrite(Right_Speed,150);

  forward();
  delay(2000);
  backward();
  delay(2000);
  turnRight();
  delay(500);
  turnLeft();
  //stpLeft();
  delay(500);
  Stop();
  delay(1500);
  //stpLeft();
  //delay(500);*/
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


