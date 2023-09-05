/*
  GCode interpreter from SD Card
  created   Aug 2023
  by Benjamin Jakobs, University of Edinburgh
*/

#include <SPI.h>
#include <SD.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <EEPROM.h>

// User variables
float xCalibration = 25;     // Steps/mm of the x-axis
float yCalibration = 6.25;   // Steps/mm of the y-axis
const int xSoftLimit = 110;  // in mm
const int ySoftLimit = 110;  // in mm

// Pin definitions
const int SCKPin = 52;
const int CSPin = 53;
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
const float homeFeed = 50;          // Feedrate (mm/sec) during homing process after first trigger
const float homeSeek = 250;         // Feedrate (mm/sec) during homing process before first trigger (should be small enough to stop machine from crashing into switch)
const float homePulloff = 4;        // Distance in mm retracted after hitting limit switch during homing (also sets 0,0 relative to limit switches)
const int homeDelay = 500;          // Delay in ms between homing operations
const int calibrationSteps = 200;   // Number of steps done during calibration

// EEPROM addresses
const int xCalibration_Address = 0;
const int yCalibration_Address = 4;

// Global GCode & operating variables
float xValue, yValue, iValue, jValue;  // Create x & y values variables
int feedrate, command;                 // Create feedrate and command variable (e.g. '1' for G1)
String userInput;
bool homed = false;
long positionToMove[4];
volatile bool interruptTrigger = false;
float XYPositionArray[inputFileSize][2] = {};
float IJPositionArray[inputFileSize][2] = {};
float XYInfoArray[inputFileSize][2] = {};
float IJInfoArray[inputFileSize][2] = {};
bool functionBreak = false;


String fileName[2] = { "Path-1.txt", "Path-2.txt" };
File file;

AccelStepper xMotor(1, motorPin[0], motorPin[1]);
AccelStepper yMotor(1, motorPin[2], motorPin[3]);
AccelStepper iMotor(1, motorPin[4], motorPin[5]);
AccelStepper jMotor(1, motorPin[6], motorPin[7]);

MultiStepper motorControl;

void setup() {
  //Wait for board to reset (prevents garbled serial while uploading)
  delay(1000);

  // Define pinmodes
  pinMode(xLimitPin, INPUT_PULLUP);
  pinMode(yLimitPin, INPUT_PULLUP);
  pinMode(iLimitPin, INPUT_PULLUP);
  pinMode(jLimitPin, INPUT_PULLUP);
  pinMode(CSPin, OUTPUT);

  // Configure motor settings
  xMotor.setMaxSpeed(maxSpeed);
  yMotor.setMaxSpeed(maxSpeed);
  iMotor.setMaxSpeed(maxSpeed);
  jMotor.setMaxSpeed(maxSpeed);
  xMotor.setAcceleration(motorAcceleration);
  yMotor.setAcceleration(motorAcceleration);
  iMotor.setAcceleration(motorAcceleration);
  jMotor.setAcceleration(motorAcceleration);

  motorControl.addStepper(xMotor);
  motorControl.addStepper(yMotor);
  motorControl.addStepper(iMotor);
  motorControl.addStepper(jMotor);

  // Actualise settings from EEPROM
  EEPROM.get(xCalibration_Address, xCalibration);
  EEPROM.get(yCalibration_Address, yCalibration);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
    ;

  // Initialise SD card and fine file sizes
  initialiseSD();

  //Delay to avoid interefence from interrupts
  delay(150);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(xLimitPin), limitInterrupt_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(yLimitPin), limitInterrupt_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(iLimitPin), limitInterrupt_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(jLimitPin), limitInterrupt_ISR, FALLING);
  // Output confimation
  Serial.println("Setup complete.");
}

void loop() {
  userInterface();
  functionBreak = false;
}

void runProgram() {
  Serial.print("Starting...");
  // Read both files
  readFile(fileName[0]);
  Serial.print("done reading file 1...");
  readFile(fileName[1]);
  Serial.print("done reading file 2...moving...");

  if (functionBreak == false) {
    for (int i = 0; i < inputFileSize; i++) {
      moveMotor(XYPositionArray[i][0], XYPositionArray[i][1], 100, IJPositionArray[i][0], IJPositionArray[i][1], 100);
    }
    Serial.println("done.");
  } else {
    Serial.println("path is out of bounds...stopping execution.");
  }
}

void readFile(String fileToRead) {
  file = SD.open(fileToRead);
  if (file) {
    unsigned int index = 0;
    unsigned int writeIndex = 0;
    char c;

    while (file.available()) {
      c = file.read();  // Read a character from the file

      if (c == '\n') {
        buffer[index] = '\0';  // Null-terminate the buffer
        if (fileToRead == fileName[0]) {
          interpretXY(buffer);
          XYPositionArray[writeIndex][0] = xValue;
          XYPositionArray[writeIndex][1] = yValue;
          XYInfoArray[writeIndex][0] = command;
          XYInfoArray[writeIndex++][1] = feedrate;
          if (xValue < 0 || xValue >= xSoftLimit || yValue < 0 || yValue >= ySoftLimit) {
            functionBreak = true;
          }
        } else if (fileToRead == fileName[1]) {
          interpretIJ(buffer);
          IJPositionArray[writeIndex][0] = iValue;
          IJPositionArray[writeIndex][1] = jValue;
          IJInfoArray[writeIndex][0] = command;
          IJInfoArray[writeIndex++][1] = feedrate;
          if (iValue < 0 || iValue >= xSoftLimit || jValue < 0 || jValue >= ySoftLimit) {
            functionBreak = true;
          }
        }
        // Clear the buffer for the next line
        memset(buffer, 0, bufferSize);
        index = 0;
      } else if (index < bufferSize - 1) {
        buffer[index++] = c;  // Add the character to the buffer
      }
    }
  } else {
    Serial.println("error opening file.");
  }
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
}

// Homing sequence
void home() {
  Serial.print("Homing...");
  delay(150);
  // Detach interrupts
  detachInterrupt(digitalPinToInterrupt(xLimitPin));
  detachInterrupt(digitalPinToInterrupt(yLimitPin));
  delay(150);

  // Set xMotor moving at homeSeek rate towards 0
  xMotor.setSpeed(-homeSeek);
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
  xMotor.setSpeed(-homeFeed);
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
  yMotor.setSpeed(-homeSeek);
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
  yMotor.setSpeed(-homeFeed);
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

  // Repeat for other motors

  // Re-attach interrupts
  delay(150);
  attachInterrupt(digitalPinToInterrupt(xLimitPin), limitInterrupt_ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(yLimitPin), limitInterrupt_ISR, FALLING);
  // Set homed to true
  homed = true;
  // Output to serial that homing is complete
  Serial.println("homing complete.");
}

// Calibration function used for calculating axis travel in mm
void calibrate(float& xCalibration, float& yCalibration) {
  // Type "ready" for x calibration
  Serial.println("Type 'ready' for x calibration or anything else to quit calibration.");
  while (!Serial.available())
    ;
  userInput = Serial.readStringUntil('\n');
  if (userInput.equals("ready")) {
    // Move calibrationSteps in x
    xMotor.setSpeed(homeSeek);
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
  userInput = Serial.readStringUntil('\n');
  Serial.println(userInput);
  xCalibration = calibrationSteps / userInput.toFloat();
  Serial.print("xCalibration value: ");
  Serial.println(xCalibration);
  // Type "ready" for y calibration
  Serial.println("Type 'ready' for y calibration or anything else to quit calibration.");
  while (!Serial.available())
    ;
  userInput = Serial.readStringUntil('\n');
  if (userInput.equals("ready")) {
    // Move calibrationSteps in y
    yMotor.setSpeed(homeSeek);
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
  userInput = Serial.readStringUntil('\n');
  Serial.println(userInput);
  yCalibration = calibrationSteps / userInput.toFloat();
  Serial.print("yCalibration value: ");
  Serial.println(yCalibration);

  // Store to EEPROM
  EEPROM.put(xCalibration_Address, xCalibration);
  EEPROM.put(yCalibration_Address, yCalibration);

  Serial.println("Calibration complete.");
}

// Takes the buffer string (one line of Gcode) and outputs the float values following X and Y as well as integers of the command and feedrate
void interpretXY(const char* input) {
  const char* commandPointer = strchr(input, 'G');  // Find the pointer to the letter 'G' in the string
  if (commandPointer) {
    command = atoi(commandPointer + 1);  // If there is a 'G', store the value of the following integer in the 'command' variable
  }
  const char* feedPointer = strchr(input, 'F');
  if (feedPointer) {
    feedrate = atoi(feedPointer + 1);
  }
  const char* xPointer = strchr(input, 'X');
  if (xPointer) {
    xValue = atof(xPointer + 1);
  }
  const char* yPointer = strchr(input, 'Y');
  if (yPointer) {
    yValue = atof(yPointer + 1);
  }
}

void interpretIJ(const char* input) {
  const char* commandPointer = strchr(input, 'G');  // Find the pointer to the letter 'G' in the string
  if (commandPointer) {
    command = atoi(commandPointer + 1);  // If there is a 'G', store the value of the following integer in the 'command' variable
  }
  const char* feedPointer = strchr(input, 'F');
  if (feedPointer) {
    feedrate = atoi(feedPointer + 1);
  }
  const char* xPointer = strchr(input, 'X');
  if (xPointer) {
    iValue = atof(xPointer + 1);
  }
  const char* yPointer = strchr(input, 'Y');
  if (yPointer) {
    jValue = atof(yPointer + 1);
  }
}

void interpretInput(const char* input, char& userCommand, float& userVar1, float& userVar2, float& userVar3) {
  const char* userCommandPointer = strchr(input, '/');  // Find the pointer to the letter 'G' in the string
  if (userCommandPointer) {
    userCommand = *(userCommandPointer + 1);  // If there is a 'G', store the value of the following integer in the 'command' variable
  } else {
    userCommand = 0;
  }
  const char* userVar1Pointer = strchr(input, 'A');
  if (userVar1Pointer) {
    userVar1 = atof(userVar1Pointer + 1);
  } else {
    userVar1 = 0;
  }
  const char* userVar2Pointer = strchr(input, 'B');
  if (userVar2Pointer) {
    userVar2 = atof(userVar2Pointer + 1);
  } else {
    userVar2 = 0;
  }
  const char* userVar3Pointer = strchr(input, 'C');
  if (userVar3Pointer) {
    userVar3 = atof(userVar3Pointer + 1);
  } else {
    userVar3 = 0;
  }
}

void userInterface() {
  char userCommand;
  float userVar1, userVar2, userVar3;

  Serial.println("Input command.");

  while (!Serial.available())
    ;
  userInput = Serial.readStringUntil('\n');

  interpretInput(userInput.c_str(), userCommand, userVar1, userVar2, userVar3);

  Serial.print(userCommand);
  Serial.print(" ");
  Serial.print(userVar1);
  Serial.print(" ");
  Serial.print(userVar2);
  Serial.print(" ");
  Serial.println(userVar3);

  switch (userCommand) {
    case 'r':
      // run command
      if (homed == true) {
        Serial.print("Running...");
        runProgram();
      } else {
        Serial.println("Machine should be homed first. Type 'override' to run anyway or anything else to cancel.");
        while (!Serial.available())
          ;
        userInput = Serial.readStringUntil('\n');
        if (userInput.equals("override")) {
          runProgram();
        } else {
          break;
        }
      }
      break;
    case 'h':
      // home command
      home();
      break;
    case 'c':
      //calibrate command
      calibrate(xCalibration, yCalibration);
      break;
    case 'j':
      //jog command
      moveMotor(xMotor.currentPosition() + userVar1, yMotor.currentPosition() + userVar2, userVar3, 0, 0, 0);
      break;
    case 'i':
      // Initialise SD
      initialiseSD();
      break;
    default:
      Serial.println("Invalid command.");
      break;
  }
}


void initialiseSD() {
  Serial.print("Initializing SD...");
  if (!SD.begin(SCKPin)) {
    Serial.println("initialization failed!");
  } else {
    Serial.println("initialization done.");
  }
}

// ISR for limit switch trigger
void limitInterrupt_ISR() {
  //xMotor.stop();
  //yMotor.stop();
}