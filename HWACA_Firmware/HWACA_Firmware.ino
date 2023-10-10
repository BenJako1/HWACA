/*
  GCode interface
  created Aug 2023
  by Benjamin Jakobs, University of Edinburgh
*/

// Include required libraries
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <EEPROM.h>

// Forward declaring functions
void moveMotor();
void CMD_G28();
void CMD_G33();
void CMD_G1();
int interpretCMD();
void interface();

// User variables
float xCalibration = 25 * 2;    // Steps/mm of the x-axis
float yCalibration = 6.25 * 2;  // Steps/mm of the y-axis
const int xSoftLimit = 110;     // in mm
const int ySoftLimit = 110;     // in mm

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
const int xCalibration_Address = 4;  // Length 4 since float values are stored
const int yCalibration_Address = 8;

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
}

void CMD_G28() {
  bool xFeed = false;
  bool iFeed = false;
  bool yFeed = false;
  bool jFeed = false;
  bool xPulloff = false;
  bool iPulloff = false;
  bool yPulloff = false;
  bool jPulloff = false;

  xMotor.setSpeed(-homeSeek * xCalibration);
  iMotor.setSpeed(-homeSeek * xCalibration);

  while (xFeed == false || iFeed == false) {
    xMotor.runSpeed();
    iMotor.runSpeed();
    if (!digitalRead(xLimitPin)) {
      xMotor.setSpeed(0);
      xFeed = true;
    }
    if (!digitalRead(iLimitPin)) {
      iMotor.setSpeed(0);
      iFeed = true;
    }
  }
  xMotor.setSpeed(homeSeek * xCalibration);
  iMotor.setSpeed(homeSeek * xCalibration);
  xMotor.move(homePulloff * xCalibration);
  iMotor.move(homePulloff * xCalibration);
  while (xPulloff == false || iPulloff == false) {
    xMotor.run();
    iMotor.run();
    if (!xMotor.distanceToGo()) {
      xMotor.stop();
      xPulloff = true;
    }
    if (!iMotor.distanceToGo()) {
      iMotor.stop();
      iPulloff = true;
    }
  }
  xFeed = false;
  iFeed = false;
  xPulloff = false;
  iPulloff = false;

  xMotor.setSpeed(-homeFeed * xCalibration);
  iMotor.setSpeed(-homeFeed * xCalibration);

  while (xFeed == false || iFeed == false) {
    xMotor.runSpeed();
    iMotor.runSpeed();
    if (!digitalRead(xLimitPin)) {
      xMotor.setSpeed(0);
      xFeed = true;
    }
    if (!digitalRead(iLimitPin)) {
      iMotor.setSpeed(0);
      iFeed = true;
    }
  }
  xMotor.setSpeed(homeFeed * xCalibration);
  iMotor.setSpeed(homeFeed * xCalibration);
  xMotor.move(homePulloff * xCalibration);
  iMotor.move(homePulloff * xCalibration);
  while (xPulloff == false || iPulloff == false) {
    xMotor.run();
    iMotor.run();
    if (!xMotor.distanceToGo()) {
      xMotor.stop();
      xPulloff = true;
    }
    if (!iMotor.distanceToGo()) {
      iMotor.stop();
      iPulloff = true;
    }
  }
  xMotor.setCurrentPosition(0);
  iMotor.setCurrentPosition(0);

  yMotor.setSpeed(-homeSeek * yCalibration);
  jMotor.setSpeed(-homeSeek * yCalibration);

  while (yFeed == false || jFeed == false) {
    yMotor.runSpeed();
    jMotor.runSpeed();
    if (!digitalRead(yLimitPin)) {
      yMotor.setSpeed(0);
      yFeed = true;
    }
    if (!digitalRead(jLimitPin)) {
      jMotor.setSpeed(0);
      jFeed = true;
    }
  }
  yMotor.setSpeed(homeSeek * yCalibration);
  jMotor.setSpeed(homeSeek * yCalibration);
  yMotor.move(homePulloff * yCalibration);
  jMotor.move(homePulloff * yCalibration);
  while (yPulloff == false || jPulloff == false) {
    yMotor.run();
    jMotor.run();
    if (!yMotor.distanceToGo()) {
      yMotor.stop();
      yPulloff = true;
    }
    if (!jMotor.distanceToGo()) {
      jMotor.stop();
      jPulloff = true;
    }
  }
  yFeed = false;
  jFeed = false;
  yPulloff = false;
  jPulloff = false;

  yMotor.setSpeed(-homeFeed * yCalibration);
  jMotor.setSpeed(-homeFeed * yCalibration);

  while (yFeed == false || jFeed == false) {
    yMotor.runSpeed();
    jMotor.runSpeed();
    if (!digitalRead(yLimitPin)) {
      yMotor.setSpeed(0);
      yFeed = true;
    }
    if (!digitalRead(jLimitPin)) {
      jMotor.setSpeed(0);
      jFeed = true;
    }
  }
  yMotor.setSpeed(homeFeed * yCalibration);
  jMotor.setSpeed(homeFeed * yCalibration);
  yMotor.move(homePulloff * yCalibration);
  jMotor.move(homePulloff * yCalibration);
  while (yPulloff == false || jPulloff == false) {
    yMotor.run();
    jMotor.run();
    if (!yMotor.distanceToGo()) {
      yMotor.stop();
      yPulloff = true;
    }
    if (!jMotor.distanceToGo()) {
      jMotor.stop();
      jPulloff = true;
    }
  }
  yMotor.setCurrentPosition(0);
  jMotor.setCurrentPosition(0);
}

// Calibration function used for calculating axis travel in mm
void CMD_G33(float& xCalibration, float& yCalibration) {
  // Move calibrationSteps in x
  xMotor.setSpeed(homeSeek * xCalibration);
  xMotor.move(calibrationSteps);
  while (xMotor.distanceToGo() != 0) {
    xMotor.run();
  }
  // Request user to input measured value
  Serial.print(":Input distance travelled (mm): ");
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
  Serial.print(":Input distance travelled (mm): ");
  // Read input value and write to yCalibration
  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  Serial.print(rxString);
  yCalibration = calibrationSteps / rxString.toFloat();

  // Store to EEPROM
  EEPROM.put(xCalibration_Address, xCalibration);
  EEPROM.put(yCalibration_Address, yCalibration);
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
void interpretCMD(const char* input, int& command, int& EESave) {
  const char* commandPointer = strchr(input, 'G');  // Find the pointer to the 'G' character in the string
  if (commandPointer) {
    command = atoi(commandPointer + 1);  // If there is a 'G', store the value of the following integer in the 'command' variable
  } else {
    command = 0;
  }
  const char* EESavePointer = strchr(input, '$');  // Find the pointer to the '$' character in the string
  if (EESavePointer) {
    EESave = atoi(EESavePointer + 1);  // If there is a '$', store the value of the following integer in the 'command' variable
  } else {
    EESave = 0;
  }
}

float interpretEEValue(const char* input) {
  const char* valuePointer = strchr(input, '=');  // Find the pointer to the 'G' character in the string
  if (valuePointer) {
    return atof(valuePointer + 1);  // If there is a 'G', store the value of the following integer in the 'command' variable
  }
}

void interface() {
  static int command;
  static int EESave;

  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  delay(10);

  interpretCMD(rxString.c_str(), command, EESave);

  switch (command) {
    case 1:
      CMD_G1(rxString.c_str(), xValue, yValue, iValue, jValue, feedrate);
      break;
    case 28:
      CMD_G28();
      break;
    case 33:
      CMD_G33(xCalibration, yCalibration);
      break;
    default:
      break;
  }

  switch (EESave) {
    case 1:
      EEPROM.get(xCalibration_Address, xCalibration);
      EEPROM.get(yCalibration_Address, yCalibration);
      Serial.print("!xCalibration:");
      Serial.print(xCalibration);
      Serial.print(", yCalibration:");
      Serial.print(yCalibration);
    case 2:
      EEPROM.put(xCalibration_Address, interpretEEValue(rxString.c_str()));
      break;
    case 3:
      EEPROM.put(yCalibration_Address, interpretEEValue(rxString.c_str()));
      break;
  }

  // + character requests next command
  Serial.print("+");
}