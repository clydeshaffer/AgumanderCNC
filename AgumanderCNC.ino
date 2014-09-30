#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <SPI.h>
#include <Pixy.h>
#include <elapsedMillis.h>
#include <SD.h>

//REQUIRED LIBRARIES
////ElapsedMillis
////Pixy (CMUCam5)
////Adafruit MotorShield

//OTHER LIBRARIES INCLUDED IN ARDUINO IDE

//reference to camera module
Pixy pixy;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *xAxis = AFMS.getMotor(1);
Adafruit_DCMotor *yAxis = AFMS.getMotor(2);
Adafruit_DCMotor *zAxis = AFMS.getMotor(3);

int laserPin = 9;

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

int toolDutyCycle = 0; //0-100 (percent) tool activation power or whatever

//CALIBRATION VALUES
//TODO: ADD CALIBRATION STEP TO DETERMINE REQUIRED MOTOR POWER TO MOVE ALONG EACH AXIS
int xMin = 1000;
int xMax = 0;
int yMin = 1000;
int yMax = 0;

//Global cache for beacon tracking info
int lastReportedX;
int lastReportedY;

File dataFile;

//returns x position as 0-1000 value based on min and max
int convertX(int rawX) {
  return ((rawX - xMin) * (10000 / (xMax - xMin))) / 10;
}

//returns y position as 0-1000 value based on min and max
int convertY(int rawY) {
  return ((rawY - yMin) * (10000 / (yMax - yMin))) / 10;
}

int maxSpeed = 128;
//speedValue ranges from -100 to 100, as percentage of maxSpeed
void setSpeedPercent(Adafruit_DCMotor *motor, int speedValue) {
  if(speedValue > 100) speedValue = 100;
  if(speedValue < -100) speedValue = -100;
  int actualSpeed = speedValue * maxSpeed / 100;
  if(actualSpeed < 0) {
    actualSpeed *= -1;
    motor->run(BACKWARD);
  } 
  else {
    motor->run(FORWARD);
  }
  motor->setSpeed(actualSpeed);
}

//Reads three bytes and sets target parameters
//if doMove is set FALSE then xPos and yPos are unchanged
//toolPower only gets like three bits of resolution, at current
boolean readNextCmd (int *xPos, int *yPos, int *toolPower) {
  if(dataFile.available()) {
    int x, y, tool;
    x = dataFile.parseInt();
    y = dataFile.parseInt();
    tool = dataFile.parseInt();
    if(x >= 0 && x <= 1000) *xPos = x;
    if(y >= 0 && y <= 1000) *yPos = y;
    if(tool >= 0 && tool <= 255) *toolPower = tool;
    return true;
  } else {
    return false;
  }
}

void setLaserPower(int dutyCycle) {
  if(dutyCycle > 255) dutyCycle = 255;
  if(dutyCycle < 0) dutyCycle = 0;
  analogWrite(laserPin, dutyCycle);
}

//Called on boot
void setup() {
  pinMode(laserPin, OUTPUT);
  setLaserPower(0);
  
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Agumander's Cheapo CNC - Online!");
  Serial.setTimeout(10000);

  Serial.println("Initializing Pixy...");
  pixy.init();

  Serial.println("Initializing motors...");
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  xAxis->setSpeed(0);
  yAxis->setSpeed(0);
  zAxis->setSpeed(0);
  
  Serial.print("\nInitializing SD card...");
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  if(SD.exists("data.agu")) Serial.println("data.agu exists");
  dataFile = SD.open("data.agu");
  if(dataFile) Serial.println("Found instruction data!");
  else Serial.println("ERROR: Data not found.");
  Serial.println("Setup Complete!");
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
    sprintf(buf, "At (%d, %d)", lastReportedX, lastReportedY);
    Serial.println(buf);
    Serial.print("MODE IS CALIBRATE\n");
  } 
  else if(cncMode == CALIBRATE) {

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
  } 
  else if(cncMode == GOTO_TARGET) {
    int deltaX = target_x - convertX(lastReportedX);
    int deltaY = target_y - convertY(lastReportedY);
    
    if(abs(deltaX) < 20 && abs(deltaY) < 20) {
      //TODO: Normalize speeds so axes arrive at target simultaneously
      setSpeedPercent(xAxis, 0);
      setSpeedPercent(yAxis, 0);
      Serial.print("Reading commands... bytes available: ");
      Serial.println(Serial.available());
      if(readNextCmd(&target_x, &target_y, &toolDutyCycle)) {
        Serial.print("Moving to: ");
        sprintf(buf, "(%d, %d) ", target_x, target_y);
        Serial.print(buf);
        sprintf(buf, "Tool Duty Cycle: %d\n", toolDutyCycle);
        Serial.print(buf);
        setLaserPower(toolDutyCycle);
      } else {
        Serial.print("CONNECTION TIMED OUT!\n");
        cncMode = AT_TARGET;  
        setLaserPower(0);
      }
    } 
    else {
      setSpeedPercent(xAxis, -deltaX);
      setSpeedPercent(yAxis, -deltaY);
    }
  } 
  else {
    setSpeedPercent(xAxis, 0);
    setSpeedPercent(yAxis, 0);
  }
}
