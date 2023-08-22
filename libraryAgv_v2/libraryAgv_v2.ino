
//Pin declaration
  //Sensor LSS05
  int leftSensor = A1;
  int leftMiddleSensor = A2;
  int middleSensor = A3;
  int rightMiddleSensor = A4;
  int rightSensor = A5;

  //MotorDriver MDD10A
  int dirPin1 = 5; //direction pin motor 1
  int pwmPin1 = 6; //pwm pin motor 1
  int dirPin2 = 4; //direction pin motor 2
  int pwmPin2 = 3; //pwm pin motor 2

  //Ultrasonic sensor
  int ultraSonicTrigger = 7; //HCSR04 Trigger Pin
  int ultraSonicEcho = 8;   //HCSR04 Echo Pin
  
  int minDistance = 5;  //Distance in cm of which the ultrasonic sensor will trigger


void setup() {
  //Motor driver pin setup
  pinMode(dirPin1, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  
  digitalWrite(dirPin1, HIGH);
  digitalWrite(dirPin2, HIGH);

   //LSS05 Auto-Calibrating Line Sensor Pin Setup
  pinMode(leftSensor,INPUT);
  pinMode(leftMiddleSensor,INPUT);
  pinMode(middleSensor,INPUT);
  pinMode(rightMiddleSensor,INPUT);
  pinMode(rightSensor,INPUT);
}

void loop() {
  

  if(obstacleDetected() == true){
    stopMotor();
  } else{ 
      moveMotor(); 
  }
}

void moveMotor(){
   if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==1)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 150);   //PWM Speed Control
    analogWrite(pwmPin2, 150);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==1)&&(digitalRead(middleSensor)==1)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 150);   //PWM Speed Control
    analogWrite(pwmPin2, 90);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==1)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 150);   //PWM Speed Control
    analogWrite(pwmPin2, 40);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==1)&&(digitalRead(leftMiddleSensor)==1)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 150);   //PWM Speed Control
    analogWrite(pwmPin2, 30);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==1)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 150);   //PWM Speed Control
    analogWrite(pwmPin2, 0);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==1)&&(digitalRead(rightMiddleSensor)==1)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 90);   //PWM Speed Control
    analogWrite(pwmPin2, 150);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==1)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 40);   //PWM Speed Control
    analogWrite(pwmPin2, 150);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==1)&&(digitalRead(rightSensor)==1))
  {
    analogWrite(pwmPin1, 30);   //PWM Speed Control
    analogWrite(pwmPin2, 150);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==1))
  {
    analogWrite(pwmPin1, 0);   //PWM Speed Control
    analogWrite(pwmPin2, 150);   //PWM Speed Control
  }
}

void stopMotor(){
    analogWrite(pwmPin1, 0);   //PWM Speed Control
    analogWrite(pwmPin2, 0);   //PWM Speed Control
}


int obstacleDistance(){
  digitalWrite(ultraSonicTrigger, LOW);
  delayMicroseconds(2);  
  digitalWrite(ultraSonicTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraSonicTrigger, LOW);

  int duration = pulseIn(ultraSonicEcho, HIGH);
  int distance = duration * 0.0034/2;

  return distance;
}

bool obstacleDetected(){
  bool detect;
  if(obstacleDistance() <= minDistance){ //change this value to adjust the minimum distance to stop the agv.
   detect = true;
  } else { detect = false; }
  return detect;
}
