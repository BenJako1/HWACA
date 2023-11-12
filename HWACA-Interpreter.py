'''
Serial Arduino interface for HWACA
Benjamin Jakobs
Created Sep 2023
'''

import serial
import time
from pathlib import Path

filepath = '/Users/ben/Desktop/Projects/HWACA-Project/Output/Toolpath_N-10_N-10.gcode' # Define filename
#filepath = '/Users/ben/Desktop/Projects/HWACA-Project/Output/Toolpath_N-10_WortmannFX05-191.gcode' # Alternate filename

arduino = serial.Serial(port='/dev/cu.usbmodem11401', baudrate=9600, timeout=0.01) # Create serial object "ardiuno"

# ---------------------------------------------------------------------------------------------------------------------------
# Define Functions

# Function to read file and save data to list
def readFile(inputPath):
    path = Path(inputPath)  # Create path
    if not path.exists():   # Check whether path exists
        raise ValueError(f"Cannot find path={str(path)}")

    file = open(path, 'r') # Open file in read mode
    fileList = file.read().splitlines() # Create list where each element is a line  from the file
    file.close()
    return fileList

# Write string to serial
def serialWrite(inputString):
    arduino.write(bytes(inputString, 'utf-8')) # Write string to serial, converting bytes to 8-bit binary
    arduino.write(bytes('\n', 'utf-8')) # Terminate with newline

# Send file data to Arduino
def sendFileData():
    writeList = readFile(filepath)
    time.sleep(1)
    i = 0
    while i < len(writeList):
        rxString = ''
        arduino.write(bytes(writeList[i], 'utf-8')) # Write string to serial, converting bytes to 8-bit binary
        arduino.write(bytes('\n', 'utf-8')) # Terminate with newline
        time.sleep(0.02)    #delay
        while not '+' in rxString:   # Block until next message is recieved
            rxString = arduino.readline().decode('utf-8')
            if rxString and rxString[0] == '!':
                print(rxString[1:])
        i += 1
    return True

# ---------------------------------------------------------------------------------------------------------------------------
# User interface

while True:
    rxString = ''
    skipBlock = False
    txCommand = input("User input:")  # Wait for input from user
    if txCommand and txCommand[0] == '>':
        userCommand = txCommand[1:]
        if userCommand == 'run':
            skipBlock = sendFileData()
    else:
        serialWrite(txCommand)
    time.sleep(0.02)    #delay
    if skipBlock == False:
        while not '+' in rxString:   # Block until request for next command is recieved
            rxString = arduino.readline().decode('utf-8')
            if rxString and rxString[0] == ':':
                serialWrite(input(rxString[1:]))
            if rxString and rxString[0] == '!':
                print(rxString[1:])
