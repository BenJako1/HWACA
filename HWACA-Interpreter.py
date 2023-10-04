'''
Serial Arduino interface for HWACA
Benjamin Jakobs
Created Sep 2023
'''

import serial
import time
from pathlib import Path

# ---------------------------------------------------------------------------------------------------------------------------

filepath = '/Users/ben/Desktop/Projects/HWACA/Output/ToolPath_N-10_N-10.gcode' # Define filename

arduino = serial.Serial(port='/dev/cu.usbmodem11401', baudrate=9600, timeout=0.01) # Create serial object "ardiuno"

def readFile(inputPath):
    path = Path(inputPath)  # Create path
    if not path.exists():   # Check whether path exists
        raise ValueError(f"Cannot find path={str(path)}")

    file = open(path, 'r') # Open file in read mode
    fileList = file.read().splitlines() # Create list where each element is a line  from the file
    file.close()

    return fileList

#commands_df = pd.DataFrame(readFile(filepath), columns=['CMD','X','Y','I','J','FR'])
#print(readFile(filepath))

# ---------------------------------------------------------------------------------------------------------------------------

writeList = readFile(filepath)
time.sleep(2)
i = 0
while i < len(writeList):
    arduino.write(bytes(writeList[i], 'utf-8')) # Write string to serial, converting bytes to 8-bit binary
    arduino.write(bytes('\n', 'utf-8'))
    time.sleep(0.02)
    rxString = ''
    while rxString == '':
        rxString = arduino.readline().decode('utf-8')
        print(rxString, i, writeList[i])   # Read from serial (note: blocking for time = timeout)
    i += 1