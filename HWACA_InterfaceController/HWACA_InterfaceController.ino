/*
  GCode interface
  created   Aug 2023
  by Benjamin Jakobs, University of Edinburgh
*/

#include <SPI.h>
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
String rxString;
bool homed = false;

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
  xMotor.setMaxSpeed(XYFeed);
  yMotor.setMaxSpeed(XYFeed);
  iMotor.setMaxSpeed(IJFeed);
  jMotor.setMaxSpeed(IJFeed);
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
void home() {
  // Set xMotor moving at homeSeek rate towards 0
  xMotor.setSpeed(-homeSeek * xCalibration);
  // When a trigger is detected, pull off and stop
  while (digitalRead(xLimitPin) == 1) {
    xMotor.runSpeed();
  }
  xMotor.stop();
  delay(homeDelay);
  xMotor.move(homePulloff * xCalibration);
  while (xMotor.distanceToGo() != 0) {
    xMotor.run();
  }
  xMotor.stop();
  delay(homeDelay);
  // Set xMotor moving at homeFeed rate towards 0
  xMotor.setSpeed(-homeFeed * xCalibration);
  // When a trigger is detected, pull off and stop
  while (digitalRead(xLimitPin) == 1) {
    xMotor.runSpeed();
  }
  xMotor.stop();
  delay(homeDelay);
  xMotor.move(homePulloff * xCalibration);
  while (xMotor.distanceToGo() != 0) {
    xMotor.run();
  }
  xMotor.stop();
  // Set current position to 0
  xMotor.setCurrentPosition(0);

  // Set yMotor moving at homeSeek rate towards 0
  yMotor.setSpeed(-homeSeek * yCalibration);
  // When a trigger is detected, pull off and stop
  while (digitalRead(yLimitPin) == 1) {
    yMotor.runSpeed();
  }
  yMotor.stop();
  delay(homeDelay);
  yMotor.move(homePulloff * yCalibration);
  while (yMotor.distanceToGo() != 0) {
    yMotor.run();
  }
  yMotor.stop();
  delay(homeDelay);
  // Set yMotor moving at homeFeed rate towards 0
  yMotor.setSpeed(-homeFeed * yCalibration);
  // When a trigger is detected, pull off and stop
  while (digitalRead(yLimitPin) == 1) {
    yMotor.runSpeed();
  }
  yMotor.stop();
  delay(homeDelay);
  yMotor.move(homePulloff * yCalibration);
  while (yMotor.distanceToGo() != 0) {
    yMotor.run();
  }
  yMotor.stop();
  // Set current position to 0
  yMotor.setCurrentPosition(0);

  // Set iMotor moving at homeSeek rate towards 0
  iMotor.setSpeed(-homeSeek * xCalibration);
  // When a trigger is detected, pull off and stop
  while (digitalRead(iLimitPin) == 1) {
    iMotor.runSpeed();
  }
  iMotor.stop();
  delay(homeDelay);
  iMotor.move(homePulloff * xCalibration);
  while (iMotor.distanceToGo() != 0) {
    iMotor.run();
  }
  iMotor.stop();
  delay(homeDelay);
  // Set iMotor moving at homeFeed rate towards 0
  iMotor.setSpeed(-homeFeed * xCalibration);
  // When a trigger is detected, pull off and stop
  while (digitalRead(iLimitPin) == 1) {
    iMotor.runSpeed();
  }
  iMotor.stop();
  delay(homeDelay);
  iMotor.move(homePulloff * xCalibration);
  while (iMotor.distanceToGo() != 0) {
    iMotor.run();
  }
  iMotor.stop();
  // Set current position to 0
  iMotor.setCurrentPosition(0);

  // Set jMotor moving at homeSeek rate towards 0
  jMotor.setSpeed(-homeSeek * yCalibration);
  // When a trigger is detected, pull off and stop
  while (digitalRead(jLimitPin) == 1) {
    jMotor.runSpeed();
  }
  jMotor.stop();
  delay(homeDelay);
  jMotor.move(homePulloff * yCalibration);
  while (jMotor.distanceToGo() != 0) {
    jMotor.run();
  }
  jMotor.stop();
  delay(homeDelay);
  // Set jMotor moving at homeFeed rate towards 0
  jMotor.setSpeed(-homeFeed * yCalibration);
  // When a trigger is detected, pull off and stop
  while (digitalRead(jLimitPin) == 1) {
    jMotor.runSpeed();
  }
  jMotor.stop();
  delay(homeDelay);
  jMotor.move(homePulloff * yCalibration);
  while (jMotor.distanceToGo() != 0) {
    jMotor.run();
  }
  jMotor.stop();
  // Set current position to 0
  jMotor.setCurrentPosition(0);

  // Set homed to true
  homed = true;
  // Output to serial that homing is complete
  Serial.println("next");
}

// Calibration function used for calculating axis travel in mm
void calibrate(float& xCalibration, float& yCalibration) {
  // Type "ready" for x calibration
  Serial.println("Type 'ready' for x calibration or anything else to quit calibration.");
  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  if (rxString.equals("ready")) {
    // Move calibrationSteps in x
    xMotor.setSpeed(homeSeek * xCalibration);
    xMotor.move(calibrationSteps);
    while (xMotor.distanceToGo() != 0) {
      xMotor.run();
    }
  } else {
    Serial.println("Calibration cancelled.");
    return;
  }
  // Request user to input measured value
  Serial.print("What is the distance travelled in mm? Input: ");
  // Read input value and write to xCalibration
  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  Serial.println(rxString);
  xCalibration = calibrationSteps / rxString.toFloat();
  Serial.print("xCalibration value: ");
  Serial.println(xCalibration);
  // Type "ready" for y calibration
  Serial.println("Type 'ready' for y calibration or anything else to quit calibration.");
  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  if (rxString.equals("ready")) {
    // Move calibrationSteps in y
    yMotor.setSpeed(homeSeek * yCalibration);
    yMotor.move(calibrationSteps);
    while (yMotor.distanceToGo() != 0) {
      yMotor.run();
    }
  } else {
    Serial.println("Calibration cancelled.");
    return;
  }
  // Request user to input measured value
  Serial.print("What is the distance travelled in mm? Input: ");
  // Read input value and write to yCalibration
  while (!Serial.available())
    ;
  rxString = Serial.readStringUntil('\n');
  Serial.println(rxString);
  yCalibration = calibrationSteps / rxString.toFloat();
  Serial.print("yCalibration value: ");
  Serial.println(yCalibration);

  // Store to EEPROM
  EEPROM.put(xCalibration_Address, xCalibration);
  EEPROM.put(yCalibration_Address, yCalibration);

  Serial.println("next.");
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

  moveMotor(xValue, yValue, 500, iValue, jValue, 500);
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
      home();
      break;
    case 33:
      //calibrate();
      break;
    default:
      Serial.print("next");
      break;
  }
}
