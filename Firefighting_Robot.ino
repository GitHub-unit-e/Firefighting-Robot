#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define Northecho 12
#define Northtrig 13
#define Eastecho 8
#define Easttrig 11
#define Southecho 4
#define Southtrig 5
#define Westecho 6
#define Westtrig 7

#define flameOne A3
#define flameTwo A2
#define flameThree A1

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorOne = AFMS.getMotor(1);
Adafruit_DCMotor *motorTwo = AFMS.getMotor(2);
Adafruit_DCMotor *motorThree = AFMS.getMotor(3);
Adafruit_DCMotor *motorFour = AFMS.getMotor(4);
Servo servo;

int servoSpeed = 93;
int motorSpeed = 60;
int highGear = motorSpeed + 20;
int lowGear = motorSpeed - 20;

long northD = 0;
long eastD = 0;
long southD = 0;
long westD = 0;

long prevNorthD = 0;
long prevEastD = 0;
long prevSouthD = 0;
long prevWestD = 0;

int robotDir = 0;//North: 0, East: 1, South: 2, West: 3

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  servo.attach(10);
  pinMode(Northecho, INPUT); pinMode(Eastecho, INPUT); pinMode(Southecho, INPUT); pinMode(Westecho, INPUT); 
  pinMode(Northtrig, OUTPUT); pinMode(Easttrig, OUTPUT); pinMode(Southtrig, OUTPUT); pinMode(Westtrig, OUTPUT);
  pinMode(flameOne, INPUT); pinMode(flameTwo, INPUT); pinMode(flameThree, INPUT);
  setSpeedNorm(); servo.write(90);

  robotDir = 0;
}

void loop() {
  
  readDistance();
  moveRobot();

}

void readDistance() {
  prevNorthD = northD; prevEastD = eastD; prevSouthD = southD; prevWestD = westD;
  delay(50);digitalWrite(Northtrig, LOW); delayMicroseconds(2); digitalWrite(Northtrig, HIGH); delayMicroseconds(10); digitalWrite(Northtrig, LOW); northD = pulseIn(Northecho, HIGH) / 58.2;if (northD > 75) {northD = 100;}
  delay(50);digitalWrite(Easttrig, LOW); delayMicroseconds(2); digitalWrite(Easttrig, HIGH); delayMicroseconds(10); digitalWrite(Easttrig, LOW); eastD = pulseIn(Eastecho, HIGH) / 58.2;if (eastD > 75) {eastD = 100;}
  delay(50);digitalWrite(Southtrig, LOW); delayMicroseconds(2); digitalWrite(Southtrig, HIGH); delayMicroseconds(10); digitalWrite(Southtrig, LOW); southD = pulseIn(Southecho, HIGH) / 58.2;if (southD > 75) {southD = 100;}
  delay(50);digitalWrite(Westtrig, LOW); delayMicroseconds(2); digitalWrite(Westtrig, HIGH); delayMicroseconds(10); digitalWrite(Westtrig, LOW); westD = pulseIn(Westecho, HIGH) / 58.2;if (westD > 75) {westD = 100;}
}

void setSpeedNorm() {motorOne->setSpeed(motorSpeed); motorTwo->setSpeed(motorSpeed); motorThree->setSpeed(motorSpeed); motorFour->setSpeed(motorSpeed);}
void setSpeedNW() {motorOne->setSpeed(highGear); motorTwo->setSpeed(lowGear); motorThree->setSpeed(highGear); motorFour->setSpeed(lowGear);}
void setSpeedNE() {motorOne->setSpeed(lowGear); motorTwo->setSpeed(highGear); motorThree->setSpeed(lowGear); motorFour->setSpeed(highGear);}

int indent = 6;
bool turnBool = true;

void moveRobot() {//increase delay
  
  switch(robotDir) {
    
    case 0://North
    if (northD > 10) {
      if (eastD < indent) {setSpeedNW();} else if (westD < indent) {setSpeedNE();} else {setSpeedNorm();}
      motorOne->run(FORWARD); motorTwo->run(FORWARD); motorThree->run(BACKWARD); motorFour->run(BACKWARD);
      //if (northD > 30) {if (((eastD - prevEastD > 30) && (eastD - prevEastD < 100)) || ((westD - prevWestD > 30) && (westD - prevWestD < 100))) {delay(500); if (eastD > westD) {robotDir = 1;} else {robotDir = 3;}}}
      if (isSideOpen() && turnBool) {delay(1000); turnBool = false; if (eastD > westD) {robotDir = 1;} else {robotDir = 3;}}
      if (!isSideOpen()) {turnBool = true;}
    } else {
      if (eastD > westD) {robotDir = 1;} else {robotDir = 3;}
      //if (eastD > 20) {robotDir = 1;} else {robotDir = 3;}
      //if (20 > westD) {robotDir = 1;} else {robotDir = 3;}
      printDs();
    }
    break;
    
    case 1://East
    if (eastD > 10) {
      if (northD < indent) {setSpeedNW();} else if (southD < indent) {setSpeedNE();} else {setSpeedNorm();}
      motorOne->run(BACKWARD); motorTwo->run(FORWARD); motorThree->run(FORWARD); motorFour->run(BACKWARD);
      //if (eastD > 30) {if (((northD - prevNorthD > 30) && (northD - prevNorthD < 100)) || ((southD - prevSouthD > 30) && (southD - prevSouthD < 100))) {delay(500); if (northD > southD) {robotDir = 0;} else {robotDir = 2;}}}
      if (isSideOpen() && turnBool) {delay(1000); turnBool = false; if (northD > southD) {robotDir = 0;} else {robotDir = 2;}}
      if (!isSideOpen()) {turnBool = true;}
    } else {
      if (southD > northD) {robotDir = 2;} else {robotDir = 0;}
      //if (southD > 20) {robotDir = 2;} else {robotDir = 0;}
      //if (20 > northD) {robotDir = 2;} else {robotDir = 0;}
      printDs();
    }
    break;
    
    case 2://South
    if (southD > 10) {
      if (westD < indent) {setSpeedNW();} else if (eastD < indent) {setSpeedNE();} else {setSpeedNorm();}
      motorOne->run(BACKWARD); motorTwo->run(BACKWARD); motorThree->run(FORWARD); motorFour->run(FORWARD);
      //if (southD > 30) {if (((eastD - prevEastD > 30) && (eastD - prevEastD < 100)) || ((westD - prevWestD > 30) && (westD - prevWestD < 100))) {delay(500); if (eastD > westD) {robotDir = 1;} else {robotDir = 3;}}}
      if (isSideOpen() && turnBool) {delay(1000); turnBool = false; if (eastD > westD) {robotDir = 1;} else {robotDir = 3;}}
      if (!isSideOpen()) {turnBool = true;}
    } else {
      if (westD > eastD) {robotDir = 3;} else {robotDir = 1;}
      //if (westD > 20) {robotDir = 3;} else {robotDir = 1;}
      //if (20 > eastD) {robotDir = 3;} else {robotDir = 1;}
      printDs();
    }
    break;
    
    case 3://West
    if (westD > 10) {
      if (southD < indent) {setSpeedNW();} else if (northD < indent) {setSpeedNE();} else {setSpeedNorm();}
      motorOne->run(FORWARD); motorTwo->run(BACKWARD); motorThree->run(BACKWARD); motorFour->run(FORWARD);
      // ?if (westD > 30) {if (((northD - prevNorthD > 30) && (northD - prevNorthD < 100)) || ((southD - prevSouthD > 30) && (southD - prevSouthD < 100))) {delay(500); if (northD > southD) {robotDir = 0;} else {robotDir = 2;}}}
      if (isSideOpen() && turnBool) {delay(1000); turnBool = false; if (northD > southD) {robotDir = 0;} else {robotDir = 2;}}
      if (!isSideOpen()) {turnBool = true;}
    } else {
      if (northD > southD) {robotDir = 0;} else {robotDir = 2;}
      //if (northD > 20) {robotDir = 0;} else {robotDir = 2;}
      //if (20 > southD) {robotDir = 0;} else {robotDir = 2;}
      printDs();
    }
    break;
  }
}

bool isSideOpen() {
  int openLength = 75;
  if (((northD > openLength) && (eastD > openLength) && (southD > openLength)) || ((northD > openLength) && (eastD > openLength) && (westD > openLength)) || ((northD > openLength) && (southD > openLength) && (westD > openLength)) || ((eastD > openLength) && (southD > openLength) && (westD > openLength))) {return true;}
  return false;
}

void printDs() {Serial.println(northD); Serial.println(eastD); Serial.println(southD); Serial.println(westD); Serial.println();}

void moveNorth() {motorOne->run(FORWARD); motorTwo->run(FORWARD); motorThree->run(BACKWARD); motorFour->run(BACKWARD);}
void moveEast() {motorOne->run(BACKWARD); motorTwo->run(FORWARD); motorThree->run(FORWARD); motorFour->run(BACKWARD);}
void moveSouth() {motorOne->run(BACKWARD); motorTwo->run(BACKWARD); motorThree->run(FORWARD); motorFour->run(FORWARD);}
void moveWest() {motorOne->run(FORWARD); motorTwo->run(BACKWARD); motorThree->run(BACKWARD); motorFour->run(FORWARD);}

void stopRobot() {motorOne->run(RELEASE); motorTwo->run(RELEASE); motorThree->run(RELEASE); motorFour->run(RELEASE);}

