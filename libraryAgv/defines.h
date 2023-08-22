/* 
Author : Muhammad Yaacob bin Hasan
Project : Automated Guided Vehicle for Library
Date : 01/04/2019
Filename : defines.h

Description : Parameters for the AGV and also program
*/

#ifndef _DEFINES_H
#define _DEFINES_H

//Parameters for QTR-8RC Sensor
  //Tune this value for stability
#define KP 0.1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD 0.1 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 

  //Default and maximum speeds
#define M1_DEFAULT_SPEED 10 //base speed for right motor
#define M2_DEFAULT_SPEED 10 //base speed for left motor
#define M1_MAX_SPEED 30 //max speed for right motor
#define M2_MAX_SPEED 30 //max speed for left motor

  //Sensor settings
#define MIDDLE_SENSOR 3 
#define NUM_SENSORS 6//number of sensors used
#define TIMEOUT 2500 //waits for 2500 us for sensor outputs to go low
#define PIN_EMITTER 2  // emitter is controlled by digital pin 2
#define DEBUG 0 // set to 1 if serial debug output needed

//Parameters for MDD10A Motor Driver
#define PIN_DIR_1 5 //DIRECTION pun for motor 1
#define PIN_PWM_1 6 //PWM pin for motor 1
#define PIN_DIR_2 4 //DIRECTION pun for motor 2
#define PIN_PWM_2 3 //PWM pin for motor 2

//Parameter for ultrasonic sensor
#define PIN_HCSR04_TRIGGER 7
#define PIN_HCSR04_ECHO 8

//Parameter for RFID MFRC522 reader
#define PIN_RFID_SDA  10
#define PIN_RFID_SCK  13
#define PIN_RFID_MOSI 11
#define PIN_RFID_MISO 12
#define PIN_RFID_RST  9

#endif
