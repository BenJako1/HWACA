#importing libraries
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt

# Geometry variables
xFlip = [True, True]
camberLength = [75, 75]
xOffset = [2, 2]
yOffset = [2, 2]

# Specify paths & feedrate
filename = ['N-10', 'N-10']
filepath = [f'Input/{filename[0]}.txt', f'Input/{filename[1]}.txt']
feedrate = 1000 # in mm/min (constant, will be dynamically changed in later versions)

# ------------------------------------------------------------------------------------------------------------------
# Function definition

# Function for reading the .txt data file
def readData(inputPath):
    # Create path
    path = Path(inputPath)
    # As a precaution, check that the file exists
    if not path.exists():
        raise ValueError(f"Cannot find path={str(path)}")
    
    with open(path, "r") as readFile:
        vertices = []
        dataList = readFile.read().splitlines() # Create list where each element is a line
        for i in range(len(dataList)):
            buffer = dataList[i].split()
            vertices.append([float(buffer[0]), float(buffer[1])])
    return vertices

# Visualisation function
def visualise(xAxis, yAxis):
    plt.plot(xAxis, yAxis)
    plt.title('Aerofoil Plot')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis((0, 100, -50, 50))
    plt.grid()
    plt.show()

# ------------------------------------------------------------------------------------------------------------------
# Reading Gcode file

XY_df = pd.DataFrame(readData(filepath[0]), columns = ['X','Y'])    # Create dataframe from array
XY_df.loc[-1] = [XY_df['X'].iloc[-1], XY_df['Y'].iloc[-1]]  # Adding a row to have same beginning and end point
XY_df.index = XY_df.index + 1   # Shifting index
XY_df = XY_df.sort_index()  # Sorting by index
IJ_df = pd.DataFrame(readData(filepath[1]), columns = ['I','J'])    # Create dataframe from array
IJ_df.loc[-1] = [IJ_df['I'].iloc[-1], IJ_df['J'].iloc[-1]]  # Adding a row to have same beginning and end point
IJ_df.index = IJ_df.index + 1   # Shifting index
IJ_df = IJ_df.sort_index()  # Sorting by index
df = pd.concat([XY_df, IJ_df], axis=1)  # Combine dataframes

# ------------------------------------------------------------------------------------------------------------------
# Geometry configuration

if xFlip[0]: df['X'] = 1 - df['X']
if xFlip[1]: df['I'] = 1 - df['I']
df['X'] = df['X'] * camberLength[0]
df['Y'] = df['Y'] * camberLength[0]
df['I'] = df['I'] * camberLength[1]
df['J'] = df['J'] * camberLength[1]
df['X'] = df['X'] + xOffset[0]
df['Y'] = df['Y'] + yOffset[0]
df['I'] = df['I'] + xOffset[1]
df['J'] = df['J'] + yOffset[1]

# ------------------------------------------------------------------------------------------------------------------
# Visualisation

visualise(df['X'], df['Y'])

# ------------------------------------------------------------------------------------------------------------------
# Gcode compilation

with open(f'Output/Toolpath_{filename[0]}_{filename[1]}.gcode', 'w') as file: #create file and set mode to write
    file.write(f";gCode for cutting an aerofoil type {filename[0]}, {filename[1]}")
    file.write('\n')
    file.write('G28')
    
    for index in range(len(df)):
        file.write('\n')
        file.write(f'G1 X{df.X[index]} Y{df.Y[index]} I{df.I[index]} J{df.J[index]} F{feedrate}')
    
    file.write('\n')
    file.write('G1 X0 Y0 I0 J0')
file.close()