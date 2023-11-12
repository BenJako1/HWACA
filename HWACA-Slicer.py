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
yFlip = [False, False]
chordLength = [90, 60] # Scales the aeroil
sweepAngle = 10  # Sweep angle of wings in degrees
dihedralAngle = 0  # Dihedral angle in degrees
tipRotation = 0
xOffset = [0, 0]
yOffset = [0, 0]

xyBlockOffset = 0    # Distance from the XY plane to face of foam block in mm
ijBlockOffset = 0    # Distance from the IJ plane to face of foam block in mm
blockWidth = 100    # Width of the block in mm

# Specify paths & feedrate
filename = ['N-10', 'N-10']
filepath = [f'/Users/ben/Desktop/Projects/HWACA-Project/Input/{filename[0]}.txt', f'/Users/ben/Desktop/Projects/HWACA-Project/Input/{filename[1]}.txt']
outputFilepath = f'/Users/ben/Desktop/Projects/HWACA-Project/Output/Toolpath_{rootFilename}_{tipFilename}.gcode'
feedrate = 100  # Max travel speed of wire in mm/sec

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
    plt.axis((0, 110, -55, 55))
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

# Flipping aerofoils if required
if xFlip[0]: df['X'] = 1 - df['X']
if xFlip[1]: df['I'] = 1 - df['I']
if yFlip[0]: df['Y'] = max(df['Y']) - df['Y']
if yFlip[1]: df['J'] = max(df['J']) - df['J']
# Scaling to fit desired chord length
df['X'] = df['X'] * chordLength[0]
df['Y'] = df['Y'] * chordLength[0]
df['I'] = df['I'] * chordLength[1]
df['J'] = df['J'] * chordLength[1]
# Applying rotation to the tip using Euler's formula
rotationCenter = [max(df['I']), df['J'][df['I'].idxmax()]]
theta = np.radians(tipRotation)
df['I'] = df['I'] - rotationCenter[0]
df['J'] = df['J'] - rotationCenter[1]
points = df['I'] + 1j * df['J']
rotated_points = points * np.exp(1j * theta)
df['I_new'] = rotated_points.apply(lambda z: z.real)
df['J_new'] = rotated_points.apply(lambda z: z.imag)
df['I_new'] = df['I_new'] + rotationCenter[0]
df['J_new'] = df['J_new'] + rotationCenter[1]
df['I'] = df['I_new']
df['J'] = df['J_new']
df = df.drop(columns=['I_new', 'J_new'])
# Aligning leading edges
df['I'] = df['I'] + max(df['X']) - max(df['I'])
df['J'] = df['J'] + df['Y'][df['X'].idxmax()] - df['J'][df['I'].idxmax()]
# Accounting for sweep and dihedral
df['I'] = df['I'] - blockWidth * np.arctan(np.deg2rad(sweepAngle))
df['J'] = df['J'] + blockWidth * np.arctan(np.deg2rad(dihedralAngle))
# Accounting for block offset, allowing for dimensionally accurate wings
for i in range(len(df)):
    df['X'][i] = df['X'][i] + (df['X'][i] - df['I'][i]) * xyBlockOffset/ blockWidth
    df['Y'][i] = df['Y'][i] + (df['Y'][i] - df['J'][i]) * xyBlockOffset/ blockWidth
    df['I'][i] = df['I'][i] + (df['I'][i] - df['X'][i]) * ijBlockOffset/ blockWidth
    df['J'][i] = df['J'][i] + (df['J'][i] - df['Y'][i]) * ijBlockOffset/ blockWidth
# Aligning minimums to axes & offsetting
xMin = min(df['X'])
yMin = min(df['Y'])
iMin = min(df['I'])
jMin = min(df['J'])
df['X'] = df['X'] - min(xMin, iMin)
df['I'] = df['I'] - min(xMin, iMin)
df['Y'] = df['Y'] - min(yMin, jMin)
df['J'] = df['J'] - min(yMin, jMin)
df['X'] = df['X'] + xOffset[0]
df['Y'] = df['Y'] + yOffset[0]
df['I'] = df['I'] + xOffset[1]
df['J'] = df['J'] + yOffset[1]

# ------------------------------------------------------------------------------------------------------------------
# Visualisation

visualise(df['X'], df['Y'], df['I'], df['J'])

# ------------------------------------------------------------------------------------------------------------------
# Gcode compilation

with open(outputFilepath, 'w') as file: #create file and set mode to write
    file.write(f";gCode for cutting an aerofoil type {rootFilename}, {tipFilename}")
    file.write(f"G104")
    file.write('\n')
    
    for index in range(len(df)):
        file.write('\n')
        file.write(f'G1 X{round(df.X[index], 4)} Y{round(df.Y[index], 4)} I{round(df.I[index], 4)} J{round(df.J[index], 4)} F{feedrate}')
    
    file.write('\n')
    file.write('G1 X0 Y0 I0 J0')
    file.write(f"G105")
    file.write('\n')
file.close()