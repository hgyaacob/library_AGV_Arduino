/* 
Author : Muhammad Yaacob bin Hasan
Project : Automated Guided Vehicle for Library
Date : 01/04/2019
Filename : main.ino
*/
#include <SPI.h>
#include "MFRC522.h"
#include "defines.h"
#include "QTRSensors.h"


//Variable declaration

QTRSensorsRC qtr((const uint8_t[]){19,18,17,16,15,14}, NUM_SENSORS, TIMEOUT, PIN_EMITTER);
MFRC522 mfrc522(PIN_RFID_SDA,PIN_RFID_RST); //Create mfrc522 instance


int agvPosition;
uint16_t sensorValues[NUM_SENSORS];
bool moveState;
int minDistance = 5;

void setup() {  

  //Calibrate QTR-8RC Sensor
  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  Serial.println("Calibrating...");
 

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  // Initialize HC-SR04 Ultrasonic Sensor
  pinMode(PIN_HCSR04_TRIGGER, OUTPUT);
  pinMode(PIN_HCSR04_ECHO, OUTPUT);
  Serial.println("Ultrasonic sensor initialized...");

  //Initialize MDD10A Motor Driver
  pinMode(PIN_DIR_1, OUTPUT);
  pinMode(PIN_PWM_1, OUTPUT);
  pinMode(PIN_DIR_2, OUTPUT);
  pinMode(PIN_PWM_2, OUTPUT);
  Serial.println("MDD10A Initialized...");

  //Initialize SPI port for MFRC522
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("RFID Initialized...");

  //Initialize Serial Port
  Serial.begin(9600); 
  Serial.println("Opening serial port..."); 
  delay(2000);
}

int lastError = 0;

void loop() {
  
  agvPosition = getPosition();
  stopMotor();
  checkRFIDCard();
  if(obstacleDetected() == false){
    moveState = true;
    moveMotor();
  } else{
    moveState = false;
    stopMotor();
  }
  
  
}


int obstacleDistance(){
  digitalWrite(PIN_HCSR04_TRIGGER, LOW);
  delayMicroseconds(2);  
  digitalWrite(PIN_HCSR04_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_HCSR04_TRIGGER, LOW);

  int duration = pulseIn(PIN_HCSR04_ECHO, HIGH);
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

uint16_t getPosition()
{
  uint16_t position = qtr.readLine(sensorValues);
  return position;
}

void setMotorSpeed(bool rightDirection, bool leftDirection, int rightSpeed, int leftSpeed){
  digitalWrite(PIN_DIR_1, rightDirection);
  digitalWrite(PIN_DIR_2, leftDirection);
  
  analogWrite(PIN_PWM_1, rightSpeed);
  analogWrite(PIN_PWM_2, leftSpeed);
  
}

void moveMotor(){
  int error = agvPosition - 2500;
  
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;

  int rightMotorSpeed = M1_DEFAULT_SPEED + motorSpeed;
  int leftMotorSpeed = M2_DEFAULT_SPEED - motorSpeed;
  
  if (rightMotorSpeed > M1_MAX_SPEED){ rightMotorSpeed = M1_MAX_SPEED;} // prevent the motor from going beyond max speed
  if (leftMotorSpeed > M2_MAX_SPEED){ leftMotorSpeed = M2_MAX_SPEED;} // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0){ rightMotorSpeed = 0; }// keep the motor speed positive
  if (leftMotorSpeed < 0){ leftMotorSpeed = 0; }// keep the motor speed positive

  
  setMotorSpeed(1,1,rightMotorSpeed,leftMotorSpeed);

  Serial.print("Obstacle :");
  Serial.print("\t");
  Serial.print(obstacleDetected());
  Serial.print("\t");
  Serial.print("Distance :");
  Serial.print("\t");
  Serial.print(minDistance);
  Serial.print("\t");
  Serial.print("Position :");
  Serial.print("\t");
  Serial.print(agvPosition);
  Serial.print("\t");
  Serial.print("Error :");
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
   Serial.print("Right Speed :");
  Serial.print("\t");
  Serial.print(rightMotorSpeed);
  Serial.print("\t");
  Serial.print("Left Speed :");
  Serial.print("\t");
  Serial.print(leftMotorSpeed);
  Serial.println();
}



void stopMotor(){
  analogWrite(PIN_PWM_1, 0);
  analogWrite(PIN_PWM_2, 0);
}


bool checkRFIDCard(){
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
    delay(3000); //change here to adjust pause time when stopping at RFID card
  }
}
