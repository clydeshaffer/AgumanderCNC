#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <SPI.h>
#include <Pixy.h>
#include <elapsedMillis.h>

//REQUIRED LIBRARIES (LICENCED SEPARATELY)
////ElapsedMillis
////Pixy (CMUCam5)
////Adafruit MotorShield

//reference to camera module
Pixy pixy;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *xAxis = AFMS.getMotor(1);
Adafruit_DCMotor *yAxis = AFMS.getMotor(2);
Adafruit_DCMotor *zAxis = AFMS.getMotor(3);

//AUTOMATICALLY INCREMENTED TIME VALUE
elapsedMillis timeElapsed;

//STATES FOR THE STATE MACHINE
int cncMode = -1;
const int NONE = -1;
const int CALIBRATE = 0;
const int GOTO_TARGET = 2;
const int AT_TARGET = 3;

//TARGET POSITION FOR GOTO_TARGET state
int target_x = 500;
int target_y = 500;

//CALIBRATION VALUES
//TODO: ADD CALIBRATION STEP TO DETERMINE REQUIRED MOTOR POWER TO MOVE ALONG EACH AXIS
int xMin = 1000;
int xMax = 0;
int yMin = 1000;
int yMax = 1000;

//Global cache for beacon tracking info
int lastReportedX;
int lastReportedY;



//returns x position as 0-1000 value based on min and max
int convertX(int rawX) {
  return ((rawX - xMin) * (10000 / (xMax - xMin))) / 10;
}

//returns y position as 0-1000 value based on min and max
int convertY(int rawY) {
  return ((rawY - yMin) * (10000 / (yMax - yMin))) / 10;
}

int maxSpeed = 196;
//speedValue ranges from -100 to 100, as percentage of maxSpeed
void setSpeedPercent(Adafruit_DCMotor *motor, int speedValue) {
  if(speedValue > 100) speedValue = 100;
  if(speedValue < -100) speedValue = -100;
  int actualSpeed = speedValue * maxSpeed / 100;
  if(actualSpeed < 0) {
     actualSpeed *= -1;
     motor->run(BACKWARD);
  } else {
    motor->run(FORWARD);
  }
  motor->setSpeed(actualSpeed);
}

//Called on boot
void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Agumander's Cheapo CNC - Online!");
  Serial.setTimeout(6000);
  
  pixy.init();

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  xAxis->setSpeed(0);
  yAxis->setSpeed(0);
  zAxis->setSpeed(0);
}

//Called over and over again automatically
void loop() {
  uint16_t blocks;
  char buf[32]; 

  //get this frame's beacon position  
  blocks = pixy.getBlocks();
  if (blocks)
  {
    ////VISUAL GANTRY TRACKING
    //If calibrating, set Min and Max axis values
    if(cncMode == CALIBRATE) {
      if(pixy.blocks[0].x < xMin) xMin = pixy.blocks[0].x;
      if(pixy.blocks[0].x > xMax) xMax = pixy.blocks[0].x;
      if(pixy.blocks[0].y < yMin) yMin = pixy.blocks[0].y;
      if(pixy.blocks[0].y > yMax) yMax = pixy.blocks[0].y;
    }
    
    //cache x and y values
    lastReportedX = pixy.blocks[0].x;
    lastReportedY = pixy.blocks[0].y;
    
    //TODO: track center of block rather than top left corner
  }
  
  if(cncMode == NONE) {
    cncMode = CALIBRATE;
    timeElapsed = 0;
    Serial.print("MODE IS CALIBRATE\n");
  } else if(cncMode == CALIBRATE) {
    
    //When calibrating, set positive speed for ten seconds then negative speed for ten seconds
    int dir = (timeElapsed < 10000) ? 100 : -100;    
    setSpeedPercent(xAxis, dir);
    setSpeedPercent(yAxis, dir);
    
    
    if(timeElapsed > 20000) {
      timeElapsed = 0;
      cncMode = GOTO_TARGET;
      Serial.print("MODE IS GOTO_TARGET\n");
      sprintf(buf, "X Min: %d\tMax: %d\n", xMin, xMax);
      Serial.print(buf);
      sprintf(buf, "Y Min: %d\tMax: %d\n", yMin, yMax);
      Serial.print(buf);
    }
  } else if(cncMode == GOTO_TARGET) {
    int deltaX = target_x - convertX(lastReportedX);
    int deltaY = target_y - convertY(lastReportedY);
    
    //two comparisons might be faster than two mults, an add, and a comparison
    //square imprecision area should contribute slightly to temporal accuracy
    if(abs(deltaX) < 20 && abs(deltaY) < 20) {
      setSpeedPercent(xAxis, 0);
      setSpeedPercent(yAxis, 0);
      cncMode = AT_TARGET;
      Serial.print("TARGET REACHED\n");
      sprintf(buf, "X error: %d\n", deltaX);
      Serial.print(buf);
      sprintf(buf, "Y error: %d\n", deltaY);
      Serial.print(buf);
    } else {
      setSpeedPercent(xAxis, -deltaX);
      setSpeedPercent(yAxis, -deltaY);
    }
  } else {
      setSpeedPercent(xAxis, 0);
      setSpeedPercent(yAxis, 0);
  }
}
