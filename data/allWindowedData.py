import os
import pandas as pd
import numpy as np
from collections import Counter

def removeMagInCollumn():
    allCollumnList = ['AccelXMean','AccelXStd','AccelXMin','AccelXMax','AccelYMean','AccelYStd','AccelYMin','AccelYMax','AccelZMean','AccelZStd','AccelZMin','AccelZMax','GyroXMean','GyroXStd','GyroXMin','GyroXMax','GyroYMean','GyroYStd','GyroYMin','GyroYMax','GyroZMean','GyroZStd','GyroZMin','GyroZMax','MagXMean','MagXStd','MagXMin','MagXMax','MagYMean','MagYStd','MagYMin','MagYMax','MagZMean','MagZStd','MagZMin','MagZMax','Behavior','WindowStart','WindowEnd']
    collumnList = []

    for col in allCollumnList:
        if 'Mag' not in col:
            collumnList.append( col )

    print(collumnList)
    return collumnList

def getAllDirectory( path='.' ):
    """
    Get all directory names in the given path.

    :param path: The path to scan for directories (default is the current directory).
    :return: A list of directory names.
    """
    try:
        # List all entries in the given path and filter for directories
        directory_names = [name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name))]
        return directory_names
    except Exception as e:
        print(f"An error occurred: {e}")
        return []

def WindowSensorData(dataFrame, windowSizeSec=2.0, stepSizeSec=1.0):
    """
    Create time-based windows from the sensor data.
    Each window is assigned the majority Behavior among its samples.
    """
    # Ensure data is sorted by time
    dataFrame = dataFrame.sort_values('Time_s').reset_index(drop=True)

    # Determine the start and end of the data
    startTime = dataFrame['Time_s'].iloc[0]
    endTime = dataFrame['Time_s'].iloc[-1]

    windowedRows = []
    currentStart = startTime

    while currentStart < endTime:
        currentEnd = currentStart + windowSizeSec

        # Slice the DataFrame for the current window
        windowData = dataFrame[
            (dataFrame['Time_s'] >= currentStart) &
            (dataFrame['Time_s'] < currentEnd)
        ]

        if len(windowData) == 0:
            # Move to the next window
            currentStart += stepSizeSec
            continue

        # Compute example statistical features
        features = {}
        sensorCols = ['AccelX','AccelY','AccelZ','GyroX','GyroY','GyroZ']

        for col in sensorCols:
            colArray = windowData[col].values
            features[f'{col}Mean'] = np.mean(colArray)
            features[f'{col}Std'] = np.std(colArray)
            features[f'{col}Min'] = np.min(colArray)
            features[f'{col}Max'] = np.max(colArray)

        # Determine the window label
        behaviorList = windowData['Behavior'].values
        majorityLabel = Counter(behaviorList).most_common(1)[0][0]
        features['Behavior'] = majorityLabel

        # Record window start/end times for reference
        features['WindowStart'] = currentStart
        features['WindowEnd'] = currentEnd

        windowedRows.append(features)

        # Slide the window by stepSizeSec
        currentStart += stepSizeSec

    return pd.DataFrame(windowedRows)

#
#   MAIN
#

folders = getAllDirectory()
df_list = []
for folder in folders:
    mappedFile = folder + '/mapped_data.csv'
    df = pd.read_csv(mappedFile)
    df = df.drop(columns=['MagX', 'MagY', 'MagZ'])
    df = df[df['Behavior'].isin([ 'stand', 'sit', 'lay down'])]
    windowed_df = WindowSensorData(df, windowSizeSec=3.0, stepSizeSec=0.1)
    df_list.append(windowed_df)
df_combined = pd.concat(df_list, ignore_index=True)
df_combined.to_csv( 'sample_data.csv', index=False)
print(df_combined.head())