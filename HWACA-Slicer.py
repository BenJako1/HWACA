'''
Slicer for HWACA
Benjamin Jakobs
Created Sep 2023
'''

#importing libraries
import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

# Geometry variables
xFlip = [True, True]
camberLength = [100, 100]
xOffset = [0.5, 0.5]
yOffset = [0.5, 0.5]

# Specify paths & feedrate
filename = ['N-10', 'N-10']
filepath = [f'/Users/ben/Desktop/Projects/HWACA-Project/Input/{filename[0]}.txt', f'/Users/ben/Desktop/Projects/HWACA-Project/Input/{filename[1]}.txt']
feedrate = 100 # in steps/sec (constant, will be dynamically changed in later versions)

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
def visualise(xAxis, yAxis, iAxis, jAxis):
    plt.plot(xAxis, yAxis)
    plt.plot(iAxis, jAxis)
    plt.title('Aerofoil Profiles')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis((0, 100, -50, 50))
    plt.grid()
    plt.show()

# Equalising length of dataframes
def expandDF(df1, df2):
    if len(df1) > len(df2):
        df = df2
        expanded = "df2"
        length = len(df1)
    elif len(df1) < len(df2):
        df = df1
        expanded = "df1"
        length = len(df2)
    else:
        return pd.concat([df1, df2], axis=1)  # Combine dataframes
    x = []
    y = []
    distance = []
    while len(df) < length:
        for i in range(1,len(df)):
            # Store x_n - x_n-1 and y_n - y_n-1 in seperate arrays
            x.append(df.iloc[i,0])
            y.append(df.iloc[i,1])
            distance.append(np.sqrt((df.iloc[i,0] - df.iloc[i-1,0])**2 + (df.iloc[i,1] - df.iloc[i-1,1])**2))
        distance = list(map(abs, distance))
        index = np.argmax(distance)
        newrow = [(x[index]+x[index-1])/2, (y[index]+y[index-1])/2]
        df = pd.concat([df.iloc[:index+1], pd.DataFrame([newrow], columns=list(df.columns)), df.iloc[index+1:]])
        # Reset the indices
        df = df.reset_index(drop=True)
    if expanded == 'df1':
        df = pd.concat([df, df2], axis=1)  # Combine dataframes
    elif expanded == 'df2':
        df = pd.concat([df1, df], axis=1)  # Combine dataframes
    return df

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

# ------------------------------------------------------------------------------------------------------------------
# Equalising number of points

df = expandDF(XY_df, IJ_df)

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

visualise(df['X'], df['Y'], df['I'], df['J'])

# ------------------------------------------------------------------------------------------------------------------
# Gcode compilation

with open(f'/Users/ben/Desktop/Projects/HWACA-Project/Output/Toolpath_{filename[0]}_{filename[1]}.gcode', 'w') as file: #create file and set mode to write
    file.write(f";gCode for cutting an aerofoil type {filename[0]}, {filename[1]}")
    
    for index in range(len(df)):
        file.write('\n')
        file.write(f'G1 X{round(df.X[index], 4)} Y{round(df.Y[index], 4)} I{round(df.I[index], 4)} J{round(df.J[index], 4)} F{feedrate}')
    
    file.write('\n')
    file.write('G1 X0 Y0 I0 J0')
file.close()