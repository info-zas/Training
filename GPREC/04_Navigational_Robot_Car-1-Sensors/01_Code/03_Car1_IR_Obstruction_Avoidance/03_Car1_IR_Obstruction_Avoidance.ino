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
  
  Serial.begin(9600);
}

void loop() {
  analogWrite(Left_Speed,127);
  analogWrite(Right_Speed,127);

  int ir_c1 = digitalRead(A1);
  int ir_c2 = digitalRead(A2);

  Serial.print(ir_c1);
  Serial.print(" ");
  Serial.println(ir_c2);
  
  if(ir_c1 == 0 and ir_c2 == 0){
    forward();
  }
  
  else if ((ir_c1 == 1 and ir_c2 == 1) or (ir_c1 == 1 and ir_c2 == 0) or (ir_c1 == 0 and ir_c2 == 1)){
    Stop();
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

void turnLeft(){ //turnLeft
digitalWrite(Left1, LOW);  
digitalWrite(Left2, HIGH);  
digitalWrite(Right1, HIGH);  
digitalWrite(Right2, LOW); 
}

void Stop(){ //turnLeft
digitalWrite(Left1, LOW);  
digitalWrite(Left2, LOW);  
digitalWrite(Right1, LOW);  
digitalWrite(Right2, LOW); 
}



