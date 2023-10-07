/*
  GCode interface
  created   Aug 2023
  by Benjamin Jakobs, University of Edinburgh
*/

// Include required libraries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <EEPROM.h>

// Forward declaring functions
void moveMotor();
void home();
void calibrate();
void CMD_G1();
int interpretCMD();
void interface();

// User variables
float xCalibration = 25;     // Steps/mm of the x-axis
float yCalibration = 6.25;   // Steps/mm of the y-axis
const int xSoftLimit = 110;  // in mm
const int ySoftLimit = 110;  // in mm

// Pin definitions
const int motorPin[8] = { 22, 23, 24, 25, 26, 27, 28, 29 };  // Pins in pairs (Step, Direction) for each motor
const int xLimitPin = 18;
const int yLimitPin = 19;
const int iLimitPin = 20;
const int jLimitPin = 21;

const int bufferSize = 128;  // Adjust the buffer size as needed
char buffer[bufferSize];     // Create buffer

// System variables
const int inputFileSize = 100;      // Number of lines alloted for storing the file data
const int maxSpeed = 200;           // Max steps/sec
const int motorAcceleration = 500;  // Motor acceleration in steps/sec^2
const float homeFeed = 4;           // Feedrate (mm/sec) during homing process after first trigger
const float homeSeek = 10;          // Feedrate (mm/sec) during homing process before first trigger (should be small enough to stop machine from crashing into switch)
const float homePulloff = 4;        // Distance in mm retracted after hitting limit switch during homing (also sets 0,0 relative to limit switches)
const int homeDelay = 500;          // Delay in ms between homing operations
const int calibrationSteps = 200;   // Number of steps done during calibration

// EEPROM addresses
const int xCalibration_Address = 0;  // Length 4 since float values are stored
const int yCalibration_Address = 4;

// Global GCode & operating variables
float xValue, yValue, iValue, jValue;
int feedrate;
bool homed = false;
String rxString;

long positionToMove[4];
AccelStepper xMotor(1, motorPin[0], motorPin[1]);
AccelStepper yMotor(1, motorPin[2], motorPin[3]);
AccelStepper iMotor(1, motorPin[4], motorPin[5]);
AccelStepper jMotor(1, motorPin[6], motorPin[7]);

MultiStepper motorControl;

void setup() {
  // Define pinmodes
  pinMode(xLimitPin, INPUT_PULLUP);
  pinMode(yLimitPin, INPUT_PULLUP);
  pinMode(iLimitPin, INPUT_PULLUP);
  pinMode(jLimitPin, INPUT_PULLUP);

  // Configure motor parameters
  xMotor.setMaxSpeed(maxSpeed);
  yMotor.setMaxSpeed(maxSpeed);
  iMotor.setMaxSpeed(maxSpeed);
  jMotor.setMaxSpeed(maxSpeed);
  xMotor.setAcceleration(motorAcceleration);
  yMotor.setAcceleration(motorAcceleration);
  iMotor.setAcceleration(motorAcceleration);
  jMotor.setAcceleration(motorAcceleration);

  // Configure Multistepper object
  motorControl.addStepper(xMotor);
  motorControl.addStepper(yMotor);
  motorControl.addStepper(iMotor);
  motorControl.addStepper(jMotor);

  // Actualise settings from EEPROM
  //EEPROM.get(xCalibration_Address, xCalibration);
  //EEPROM.get(yCalibration_Address, yCalibration);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
    ;
}

void loop() {
  interface();
}

// Movement scripts
void moveMotor(float xMoveTo, float yMoveTo, int XYFeed, float iMoveTo, float jMoveTo, int IJFeed) {
  // Set speed according to feedrate (given from GCode in mm/min required for setMaxSpeed() in steps/sec)
  xMotor.setMaxSpeed(xCalibration * XYFeed / 60);
  yMotor.setMaxSpeed(yCalibration * XYFeed / 60);
  iMotor.setMaxSpeed(xCalibration * IJFeed / 60);
  jMotor.setMaxSpeed(yCalibration * IJFeed / 60);
  // Move motors appropriate number of steps (required steps/mm)
  positionToMove[0] = xMoveTo * xCalibration;
  positionToMove[1] = yMoveTo * yCalibration;
  positionToMove[2] = iMoveTo * xCalibration;
  positionToMove[3] = jMoveTo * yCalibration;
  motorControl.moveTo(positionToMove);
  motorControl.runSpeedToPosition();

  Serial.print("next");
}

// Homing sequence
void CMD_G28() {
  // Initialise trigger counters & pulloff booleans
  int xTrigger = 0;
  int iTrigger = 0;
  int yTrigger = 0;
  int jTrigger = 0;
  xPulloffSet = false;
  iPulloffSet = false;
  yPulloffSet = false;
  jPulloffSet = false;
  // Set xMotor & iMotor moving at rate homeSeek towards 0
  xMotor.setSpeed(-homeSeek * xCalibration);
  iMotor.setSpeed(-homeSeek * xCalibration);
  while (xTrigger && iTrigger <= 4) {
    // Every cycle, read limit switch values & increment if triggered
    xTrigger += digitalRead(xLimitPin);
    iTrigger += digitalRead(iLimitPin);
    // If not triggered yet, run towards 0
    if (xTrigger == 0) {
      xMotor.runSpeed();
    }
    // If already hit limit, set new target to homePulloff and run to it
    else if (xTrigger == 1) {
      if (xPulloffSet == false) {
        xMotor.move(homePulloff * xCalibration);
        xPulloffSet = true;
      }
      if (xMotor.distanceToGo() != 0) {
        xMotor.run();
      }
      // If at target, increment trigger counter
      else {
        xTrigger++;
      }
    }
    // If pulloff is complete, set speed homeFeed towards 0
    else if (xTrigger == 2) {
      if (xFeedSet == false) {
        xMotor.setSpeed(-homeSeek * xCalibration);
        xFeedSet = true;
      }
      xMotor.runSpeed();
      xPulloffSet = false;
    }
    // If already hit limit again, set target to homePulloff again and run to it
    else if (xTrigger == 3) {
      if (xPulloffSet == false) {
        xMotor.move(homePulloff * xCalibration);
        xPulloffSet = true;
      }
      if (xMotor.distanceToGo() != 0) {
        xMotor.run();
      }
      else {
        xTrigger++;
      }
    }
    // Stop motor after 4 movements are complete
    else {
      xMotor.stop();
    }
    if (iTrigger == 0) {
      iMotor.runSpeed();
    }
    else if (iTrigger == 1) {
      if (iPulloffSet == false) {
        iMotor.move(homePulloff * xCalibration);
        iPulloffSet = true;
      }
      if (iMotor.distanceToGo() != 0) {
        iMotor.run();
      }
      else {
        iTrigger++;
      }
    }
    else if (iTrigger == 2) {
      if (iFeedSet == false) {
        iMotor.setSpeed(-homeSeek * xCalibration);
        iFeedSet = true;
      }
      iMotor.runSpeed();
      iPulloffSet = false;
    }
    else if (iTrigger == 3) {
      if (iPulloffSet == false) {
        iMotor.move(homePulloff * xCalibration);
        iPulloffSet = true;
      }
      if (iMotor.distanceToGo() != 0) {
        iMotor.run();
      }
      else {
        iTrigger++;
      }
    }
    else {
      iMotor.stop();
    }
  }
  yMotor.setSpeed(-homeSeek * yCalibration);
  jMotor.setSpeed(-homeSeek * yCalibration);
  while (yTrigger && jTrigger <= 4) {
    // Every cycle, read limit switch values & increment if triggered
    yTrigger += digitalRead(yLimitPin);
    jTrigger += digitalRead(jLimitPin);
    // If not triggered yet, run towards 0
    if (yTrigger == 0) {
      yMotor.runSpeed();
    }
    // If already hit limit, set new target to homePulloff and run to it
    else if (yTrigger == 1) {
      if (yPulloffSet == false) {
        yMotor.move(homePulloff * yCalibration);
        yPulloffSet = true;
      }
      if (yMotor.distanceToGo() != 0) {
        yMotor.run();
      }
      // If at target, increment trigger counter
      else {
        yTrigger++;
      }
    }
    // If pulloff is complete, set speed homeFeed towards 0
    else if (yTrigger == 2) {
      if (yFeedSet == false) {
        yMotor.setSpeed(-homeSeek * yCalibration);
        yFeedSet = true;
      }
      yMotor.runSpeed();
      yPulloffSet = false;
    }
    // If already hit limit again, set target to homePulloff again and run to it
    else if (yTrigger == 3) {
      if (yPulloffSet == false) {
        yMotor.move(homePulloff * yCalibration);
        yPulloffSet = true;
      }
      if (yMotor.distanceToGo() != 0) {
        yMotor.run();
      }
      else {
        yTrigger++;
      }
    }
    // Stop motor after 4 movements are complete
    else {
      yMotor.stop();
    }
    if (jTrigger == 0) {
      jMotor.runSpeed();
    }
    else if (jTrigger == 1) {
      if (jPulloffSet == false) {
        jMotor.move(homePulloff * yCalibration);
        jPulloffSet = true;
      }
      if (jMotor.distanceToGo() != 0) {
        jMotor.run();
      }
      else {
        jTrigger++;
      }
    }
    else if (jTrigger == 2) {
      if (jFeedSet == false) {
        jMotor.setSpeed(-homeSeek * yCalibration);
        jFeedSet = true;
      }
      iMotor.runSpeed();
      iPulloffSet = false;
    }
    else if (jTrigger == 3) {
      if (jPulloffSet == false) {
        jMotor.move(homePulloff * yCalibration);
        jPulloffSet = true;
      }
      if (jMotor.distanceToGo() != 0) {
        jMotor.run();
      }
      else {
        jTrigger++;
      }
    }
    else {
      jMotor.stop();
    }
  }
}

  // Set homed to true
  homed = true;
  // Output to serial that homing is complete
  Serial.print("next");
}

// Calibration function used for calculating axis travel in mm
void calibrate(float& xCalibration, float& yCalibration) {
  // Move calibrationSteps in x
  xMotor.setSpeed(homeSeek * xCalibration);
  xMotor.move(calibrationSteps);
  while (xMotor.distanceToGo() != 0) {
    xMotor.run();
  }
  // Request user to input measured value
  Serial.print("input");
  // Read input value and write to xCalibration
  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  Serial.print(rxString);
  xCalibration = calibrationSteps / rxString.toFloat();
  // Move calibrationSteps in y
  yMotor.setSpeed(homeSeek * yCalibration);
  yMotor.move(calibrationSteps);
  while (yMotor.distanceToGo() != 0) {
    yMotor.run();
  }
  // Request user to input measured value
  Serial.print("input");
  // Read input value and write to yCalibration
  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  Serial.print(rxString);
  yCalibration = calibrationSteps / rxString.toFloat();

  // Store to EEPROM
  //EEPROM.put(xCalibration_Address, xCalibration);
  //EEPROM.put(yCalibration_Address, yCalibration);

  Serial.print("next");
}

// Interprets the G1 command parameters and moves motors
void CMD_G1(const char* input, float& xValue, float& yValue, float& iValue, float& jValue, int& feedrate) {
  const char* xPointer = strchr(input, 'X');
  if (xPointer) {
    xValue = atof(xPointer + 1);
  }
  const char* yPointer = strchr(input, 'Y');
  if (yPointer) {
    yValue = atof(yPointer + 1);
  }
  const char* iPointer = strchr(input, 'I');
  if (iPointer) {
    iValue = atof(iPointer + 1);
  }
  const char* jPointer = strchr(input, 'J');
  if (jPointer) {
    jValue = atof(jPointer + 1);
  }
  const char* feedPointer = strchr(input, 'F');
  if (feedPointer) {
    feedrate = atoi(feedPointer + 1);
  }

  moveMotor(xValue, yValue, feedrate, iValue, jValue, feedrate);
}

// Determines which command is being called in Gcode
int interpretCMD(const char* input) {
  const char* commandPointer = strchr(input, 'G');  // Find the pointer to the letter 'G' in the string
  if (commandPointer) {
    return atoi(commandPointer + 1);  // If there is a 'G', store the value of the following integer in the 'command' variable
  } else {
    return 0;
  }
}

void interface() {
  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  delay(10);

  int command = interpretCMD(rxString.c_str());

  switch (command) {
    case 1:
      CMD_G1(rxString.c_str(), xValue, yValue, iValue, jValue, feedrate);
      break;
    case 28:
      CMD_G28();
      break;
    case 33:
      CMD_G33();
      break;
    default:
      Serial.print("next");
      break;
  }
}