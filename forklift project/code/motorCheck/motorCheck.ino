//Motor A connections
int enA = 3;
int in1 = 2;
int in2 = 4;
//Motor B connections
int in3 = 5;
int in4 = 7;
int enB = 6;

void setup() {
  // set all motor a connection pins:
  pinMode (enA, OUTPUT);
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT);
  // set all motor b connection pins;
  pinMode (enB, OUTPUT);
  pinMode (in3, OUTPUT);
  pinMode (in4, OUTPUT);

  //turn off motor A 
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  //turn off motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {
 bothRunning();
  delay(500);
 speedControl();
  delay(500);
  
  // put your main code here, to run repeatedly:

}

void bothRunning(){
  //set motor a to half speed
  analogWrite(enA, 255);
  //set motor b to half speed
  analogWrite(enB, 255);

  //turn on motor a
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //turn on motor b
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(2000);


}
void directionControl(){
  //set motor to maximum speed
  analogWrite(enA, 255);

  //Turn on the motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(2000);

  //Change the motor direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(2000);

  //turn off the m otor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

}

//controlling thte spee do fthe motor
void speedControl(){
  //turn on motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  //accelerate from zero to maximum speed
  for (int i = 0; i < 256; i++){
    analogWrite(enA, i);
    delay(20);
  }

  //Decelerate from maximum speed to 0
  for(int i= 255; i>=0; i--){
    analogWrite(enA,i);
  }

  //turn o ff motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}
