#include <HCSR04.h>
#include <SPI.h>
#include <MFRC522.h>

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
  
  int minDistance = 15;  //Distance in cm of which the ultrasonic sensor will trigger
  Ultrasonic ultrasonic(ultraSonicTrigger,ultraSonicEcho); //Create ultrasonic instance

  //RFID Sensor {SCK Pin 13, MOSI Pin 11, MISO Pin 12}
  int ssPin =  10;
  int rstPin = 9;
  MFRC522 mfrc522(ssPin, rstPin);  // Create MFRC522 instance
  int waitTime = 3000; //wait time for AGV if detect RFID Card
  bool detectCard = false;
  long interval = 2000;
  long previousMillis = 0; 

void setup() {
  //Initialize serial port
  Serial.begin(9600);

  //Initialize RFID module
  SPI.begin();
  mfrc522.PCD_Init();   // Init MFRC522
  mfrc522.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details
  Serial.println("RFID sensor ready...");
  
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

  // Initialize HC-SR04 Ultrasonic Sensor

  Serial.println("Ultrasonic sensor initialized...");
}

void loop() {
  //To prevent RFID card detect twice right after detecting one
  unsigned long currentMillis = millis();
  
  Serial.print("Distance: \t");
  Serial.print(obstacleDistance());
  Serial.print("\t");
  Serial.print("Obstacle Detected: \t");
  Serial.print(obstacleDetected());
  Serial.print("\t");
  Serial.println();

  if(obstacleDetected() == true){
    stopMotor();
  }else{
      checkRFIDCard();
      moveMotor(); 
  }
}

void moveMotor(){
   if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==1)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 190);   //PWM Speed Control
    analogWrite(pwmPin2, 190);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==1)&&(digitalRead(middleSensor)==1)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 240);   //PWM Speed Control
    analogWrite(pwmPin2, 180);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==1)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 240);   //PWM Speed Control
    analogWrite(pwmPin2, 130);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==1)&&(digitalRead(leftMiddleSensor)==1)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 240);   //PWM Speed Control
    analogWrite(pwmPin2, 120);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==1)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 240);   //PWM Speed Control
    analogWrite(pwmPin2, 0);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==1)&&(digitalRead(rightMiddleSensor)==1)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 180);   //PWM Speed Control
    analogWrite(pwmPin2, 240);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==1)&&(digitalRead(rightSensor)==0))
  {
    analogWrite(pwmPin1, 130);   //PWM Speed Control
    analogWrite(pwmPin2, 240);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==1)&&(digitalRead(rightSensor)==1))
  {
    analogWrite(pwmPin1, 120);   //PWM Speed Control
    analogWrite(pwmPin2, 240);   //PWM Speed Control
  }
  else if((digitalRead(leftSensor)==0)&&(digitalRead(leftMiddleSensor)==0)&&(digitalRead(middleSensor)==0)&&(digitalRead(rightMiddleSensor)==0)&&(digitalRead(rightSensor)==1))
  {
    analogWrite(pwmPin1, 0);   //PWM Speed Control
    analogWrite(pwmPin2, 240);   //PWM Speed Control
  }
}

void stopMotor(){
    analogWrite(pwmPin1, 0);   //PWM Speed Control
    analogWrite(pwmPin2, 0);   //PWM Speed Control
}


int obstacleDistance(){
 int distance = ultrasonic.read();
  return distance;
}

bool obstacleDetected(){
  bool detect;
  if(obstacleDistance() <= minDistance){ //change this value to adjust the minimum distance to stop the agv.
   detect = true;
  } else { detect = false; }
  return detect;
}

void checkRFIDCard(){
  unsigned long currentMillis = millis();
  detectCard = false;
     
     // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }

  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) 
  {
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  content.toUpperCase();
  
  if (content.substring(1) == "06 E5 4A 1B"){

     if(currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;
      
      Serial.println("RFID Card 1 detected, pause AGV for waitTime miliseconds");
      detectCard = true;
      stopMotor();
      
      delay(waitTime);
     }
   
  }

  if (content.substring(1) == "AA 2B 56 D3"){

     if(currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;
      
      Serial.println("RFID Card 2 detected, pause AGV for waitTime miliseconds");
      detectCard = true;
      stopMotor();
      
      delay(waitTime);
     }
   
  }
}
