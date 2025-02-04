

##################################################################################
#
#   global import
#

import argparse
import cv2
import csv
import time
import os
import ast
import pandas as pd
import numpy as np
from collections import Counter

##################################################################################
#
#   local import
#

##################################################################################
#
#   variable
#

BehaviorDictFileName = "behaviorDict.py"

##################################################################################
#
#   helper function
#

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
        # sensorCols = ['AccelX','AccelY','AccelZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ']
        
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

##################################################################################
#
#   class object
#

class DataPrepare():
    ''' prepare data from device to do machine learning
        1. convert .mov file to .mp4
            ffmpeg -i input.mov -c:v libx264 -c:a aac output.mp4
        2. map video and device data to create annotation file
            createAnnotationFile()
        3. 
    '''
    
    def __init__(self, folderName, dataOffset):
        self.folderName = folderName
        self.dataOffset = dataOffset
        self.videoFileName = folderName + '/video.mp4'
        self.deviceDataFileName = folderName + '/device_data.txt'
        self.annotationFileName = folderName + '/annotation.csv'
        self.mappedDataFileName = folderName + '/mapped_data.csv'
        self.windowedDataFileName = folderName + '/windowed_data.csv'

    def moveAnnotationDataTime(self, seconds):
        input_file = self.annotationFileName
        self.annotationFileName = self.folderName + '/new_annotation.csv'
        self.mappedDataFileName = self.folderName + '/new_mapped_data.csv'
        self.windowedDataFileName = self.folderName + '/new_windowed_data.csv'
        output_file = self.annotationFileName
        print(input_file)
        print(output_file)
        with open(input_file, 'r', newline='') as infile, open(output_file, 'w', newline='') as outfile:
            reader = csv.DictReader(infile)
            fieldnames = reader.fieldnames
            
            writer = csv.DictWriter(outfile, fieldnames=fieldnames)
            writer.writeheader()
        
            for row in reader:
                # Convert DataTime_s to float, add 2, and update the value
                row['DataTime_s'] = str(float(row['VideoTime_s']) + seconds)
                writer.writerow(row)


    def createAnnotationFile( self ):
        ''' crete annotaion.csv file by mapping video to device data using cv2

            required: video.mp4, device_data.txt
            output: annotaion.csv
        '''

        mark_interval = 1.0   # mark behavior every x seconds
        speed_factor = 10.0    # 2x speed playback for video

        #   get behaviorDict
        if os.path.exists(BehaviorDictFileName):
            with open(BehaviorDictFileName, "r") as f:
                behaviorDict = ast.literal_eval(f.read().split("=", 1)[1].strip())
        else:
            behaviorDict = {}

        video_path = self.videoFileName
        data_offset = self.dataOffset

        #   get VideoCapture object
        cap = cv2.VideoCapture(video_path)
        assert cap.isOpened(), f"Error opening video file {video_path}"

        # Read the video width and height
        width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        duration = frame_count / fps
        
        # Create a named window
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL)

        # Optionally, resize the window to the exact frame size:
        cv2.resizeWindow("Video", width, height)
        print(f"Video: {video_path}")
        print(f"Video Duration: {duration:.2f} seconds, FPS: {fps}")
        print("Available behaviors:")
        for num, behav in behaviorDict.items():
            print(f'[{num}] - {behav}')
        paused = False
        last_mark_time = 0.0
        annotation_file = self.annotationFileName

        with open(annotation_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["VideoTime_s", "DataTime_s", "Behavior"])

            while True:
                if not paused:
                    if speed_factor > 1:
                        # skip frames (speed_factor - 1) times
                        for i in range(int(speed_factor) - 1):
                            ret, _ = cap.read()
                            if not ret:
                                break
                    ret, frame = cap.read()
                    if not ret:
                        print("End of video reached.")
                        break

                    current_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
                    video_time = current_frame / fps

                    display_text = (f"Time: {video_time:.2f}s "
                                    f"(Press 'p' to pause, 'q' to quit) "
                                    f"Speed: {speed_factor}x")
                    cv2.putText(frame, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                1, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.imshow("Video", frame)

                    # Check if it's time to mark behavior
                    if video_time >= last_mark_time + mark_interval:
                        # Pause to get user input
                        paused = True
                        # Display instruction on screen
                        while True:
                            instruction_frame = frame.copy()
                            cv2.putText(instruction_frame, "Press 0-9 for behavior", (10,70), cv2.FONT_HERSHEY_SIMPLEX,
                                        1, (255, 255, 0), 2, cv2.LINE_AA)
                            cv2.putText(instruction_frame, "Press 'q' to quit", (10,110), cv2.FONT_HERSHEY_SIMPLEX,
                                        1, (255, 255, 0), 2, cv2.LINE_AA)
                            cv2.imshow("Video", instruction_frame)

                            key = cv2.waitKey(0) & 0xFF
                            if key == ord('q'):
                                # Quit annotation
                                cv2.destroyAllWindows()
                                break
                            elif key == ord('p'):
                                # Resume without annotation
                                paused = False
                                break
                            elif chr(key).isdigit():
                                behav_num = int(chr(key))
                                if behav_num in behaviorDict:
                                    behavior = behaviorDict[behav_num]
                                    data_time = video_time + data_offset
                                    writer.writerow([video_time, data_time, behavior])
                                    print(f"Annotation saved: Video {video_time:.2f}s, Data {data_time:.2f}s, Behavior: {behavior}")
                                    last_mark_time = video_time
                                    paused = False
                                    break
                                else:
                                    # Unknown behavior
                                    error_frame = instruction_frame.copy()
                                    cv2.putText(error_frame, f"Unknown behavior [{behav_num}]!", (10,150),
                                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA)
                                    cv2.imshow("Video", error_frame)
                                    cv2.waitKey(1000) # Show error message briefly
                                    # Loop again to wait for a valid key
                            else:
                                # Unrecognized key, ignore
                                pass

                        if key == ord('q'):
                            break  # Exit entire loop
                else:
                    # Paused by user pressing 'p', show the last frame
                    cv2.imshow("Video", frame)
                    key = cv2.waitKey(0) & 0xFF
                    if key == ord('p'):
                        paused = False
                    elif key == ord('q'):
                        print("Quitting annotation...")
                        break

                if not paused:
                    # Running at speed_factor times speed
                    wait_time = int(1000/(fps*speed_factor))
                else:
                    # If paused, we handled keys above
                    wait_time = 0

                # If we are not paused, handle 'q' and 'p' during normal playback as well
                if not paused:
                    # Check keys quickly for quitting or pausing
                    key = cv2.waitKey(wait_time) & 0xFF
                    if key == ord('q'):
                        print("Quitting annotation...")
                        break
                    elif key == ord('p'):
                        paused = True

        cap.release()
        cv2.destroyAllWindows()

    #rewrite
    def createAnnotationFile(self):
        """
        Create annotation.csv file by mapping video to device data using cv2.
        - mark_interval = 0.5 seconds
        - No skipping frames
        - Speed up by adjusting wait_time (no skipping)
        """

        # Adjust these as desired:
        mark_interval = 0.5  # Mark behavior every 0.5 seconds
        speed_factor = 5.0   # Increase or decrease as needed, no skipping frames

        #   get behaviorDict
        if os.path.exists(BehaviorDictFileName):
            with open(BehaviorDictFileName, "r") as f:
                behaviorDict = ast.literal_eval(f.read().split("=", 1)[1].strip())
        else:
            behaviorDict = {}

        video_path = self.videoFileName
        data_offset = self.dataOffset

        #   get VideoCapture object
        cap = cv2.VideoCapture(video_path)
        assert cap.isOpened(), f"Error opening video file {video_path}"

        # Read video properties
        width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        duration = frame_count / fps

        # Create a named window
        cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Video", width, height)

        print(f"Video: {video_path}")
        print(f"Video Duration: {duration:.2f} seconds, FPS: {fps}")
        print("Available behaviors:")
        for num, behav in behaviorDict.items():
            print(f'[{num}] - {behav}')

        paused = False
        last_mark_time = 0.0
        annotation_file = self.annotationFileName

        with open(annotation_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["VideoTime_s", "DataTime_s", "Behavior"])

            while True:
                if not paused:
                    # Read next frame
                    ret, frame = cap.read()
                    if not ret:
                        print("End of video reached.")
                        break

                    current_frame = cap.get(cv2.CAP_PROP_POS_FRAMES)
                    video_time = current_frame / fps

                    display_text = (f"Time: {video_time:.2f}s "
                                    f"(Press 'p' to pause, 'q' to quit) "
                                    f"Speed: {speed_factor}x")
                    cv2.putText(frame, display_text, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    cv2.imshow("Video", frame)

                    # Check if it's time to mark behavior
                    if video_time >= last_mark_time + mark_interval:
                        # Pause to get user input
                        paused = True
                        while True:
                            instruction_frame = frame.copy()
                            cv2.putText(instruction_frame, "Press 0-9 for behavior", (10,70),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                            cv2.putText(instruction_frame, "Press 'q' to quit, 'p' to skip annotation",
                                        (10,110), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
                            cv2.imshow("Video", instruction_frame)

                            key = cv2.waitKey(0) & 0xFF
                            if key == ord('q'):
                                # Quit annotation
                                cv2.destroyAllWindows()
                                return  # Exit entire function
                            elif key == ord('p'):
                                # Resume without annotation
                                paused = False
                                break
                            elif chr(key).isdigit():
                                behav_num = int(chr(key))
                                if behav_num in behaviorDict:
                                    behavior = behaviorDict[behav_num]
                                    data_time = video_time + data_offset
                                    writer.writerow([video_time, data_time, behavior])
                                    print(f"Annotation saved: "
                                        f"Video {video_time:.2f}s, Data {data_time:.2f}s, Behavior: {behavior}")
                                    last_mark_time = video_time
                                    paused = False
                                    break
                                else:
                                    # Unknown behavior
                                    error_frame = instruction_frame.copy()
                                    cv2.putText(error_frame, f"Unknown behavior [{behav_num}]!", (10,150),
                                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                                    cv2.imshow("Video", error_frame)
                                    cv2.waitKey(1000)  # Show error message briefly
                                    # Loop again for a valid key
                            else:
                                # Unrecognized key, ignore
                                pass

                    # If not paused, run in faster time
                    if not paused:
                        wait_time = int(1000 / (fps * speed_factor))
                        key = cv2.waitKey(wait_time) & 0xFF
                        if key == ord('q'):
                            print("Quitting annotation...")
                            break
                        elif key == ord('p'):
                            paused = True

                else:
                    # Paused by user pressing 'p'
                    cv2.imshow("Video", frame)
                    key = cv2.waitKey(0) & 0xFF
                    if key == ord('p'):
                        paused = False
                    elif key == ord('q'):
                        print("Quitting annotation...")
                        break

        cap.release()
        cv2.destroyAllWindows()

    def createMappedData( self ):
        ''' mapping device data and annotation file

            required: device_data.txt, annotation_data.csv
            output: mapped_data.csv
        '''

        deviceFile = self.deviceDataFileName
        annotationFile = self.annotationFileName

        # 1. Load Device Data
        # ----------------------------------------------------
        # Your device data might be in a .txt file with CSV-like structure.
        # Adjust delimiter if needed. If it's truly comma-separated, delimiter="," is fine.
        device_df = pd.read_csv( deviceFile, delimiter=',')

        # device_df columns expected:
        # ['Time','AccelX','AccelY','AccelZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ']

        # 2. Load Annotation Data
        # ----------------------------------------------------
        annotation_df = pd.read_csv( annotationFile )

        # annotation_df columns expected:
        # ['VideoTime_s','DataTime_s','Behavior']

        # 3. Convert Device Times from ms to seconds (if indeed in ms)
        # ------------------------------------------------------------
        # If your 'Time' column is already in seconds, skip this step.
        device_df['Time_s'] = device_df['Time'] / 1000.0

        # 4. Define a Helper Function to Find Closest Row
        # ----------------------------------------------------
        # For a given annotation time (in seconds), find the row in device_df whose Time_s is closest.
        def find_closest_device_row(anno_time_s, df):
            """
            Returns the row from df that has the closest Time_s to anno_time_s.
            """
            # idxmin finds the index of the minimum difference
            idx = (df['Time_s'] - anno_time_s).abs().idxmin()
            return df.loc[idx]

        # 5. Iterate Over Each Annotation Row and Merge with Device Data
        # ---------------------------------------------------------------
        mapped_rows = []

        for i, anno_row in annotation_df.iterrows():
            anno_time = anno_row['DataTime_s']
            behavior = anno_row['Behavior']
            
            # Get the closest device row for this annotation time
            device_row = find_closest_device_row(anno_time, device_df)
            
            # Build a combined record
            combined_record = {
                'AnnotationTime_s': anno_time,
                'Behavior': behavior
            }
            
            # Copy all relevant device columns (including Time_s for reference)
            # You can choose which ones to include if you don't need them all
            for col in ['Time','Time_s','AccelX','AccelY','AccelZ','GyroX','GyroY','GyroZ','MagX','MagY','MagZ']:
                combined_record[col] = device_row[col]
            
            mapped_rows.append(combined_record)

        # 6. Create a New DataFrame with Mapped Rows
        # ----------------------------------------------------
        mapped_df = pd.DataFrame(mapped_rows)

        # 7. Save or Inspect the Result
        # ----------------------------------------------------
        fileName = self.mappedDataFileName
        mapped_df.to_csv(fileName, index=False)
        print("Mapped data saved to", fileName)
        print(mapped_df.head(10))

    def createWindowedData( self ):
        ''' created windowed data from mapped data

            required: mapped_data.csv
            output: windowed_data.csv
        '''
        deviceDf = pd.read_csv( self.mappedDataFileName )  # after merging annotation data
        windowedDf = WindowSensorData(deviceDf, windowSizeSec=3.0, stepSizeSec=0.1)
        windowedDf.to_csv( self.windowedDataFileName, index=False)
        print(windowedDf.head())

##################################################################################
#
#   main
#
       
def main():
    # Initialize the argument parser
    parser = argparse.ArgumentParser(description="Process folder names and data offset.")

    # Add arguments
    parser.add_argument(
        '--folder',
        type=str,
        required=True,
        help="Comma-separated list of folder names (e.g., xxx,yyy,zzz)."
    )
    parser.add_argument(
        '--dataOffset',
        type=float,
        required=True,
        help="Data offset as a float."
    )

    parser.add_argument(
        '--moveAnnotationFile',
        type=int,
        required=False,
        help="if want to move existing annotation file [input 1 for use this]",
        default=False
    )

    # Parse arguments
    args = parser.parse_args()

    # Process the folder argument
    folderNameList = [folder.strip() for folder in args.folder.split(',')]
    dataOffset = args.dataOffset
    moveAnnotationFile = args.moveAnnotationFile

    # Print or use the parsed arguments
    print("Folder List:", folderNameList)
    if moveAnnotationFile :
        print(f"Move annotation file for {dataOffset} sec")
        for folderName in folderNameList:
            dp = DataPrepare( folderName=folderName, dataOffset=dataOffset )
            dp.moveAnnotationDataTime( dataOffset )
            dp.createMappedData()
            dp.createWindowedData()

    else:
        print("Data Offset:", dataOffset)

        for folderName in folderNameList:
            dp = DataPrepare( folderName=folderName, dataOffset=dataOffset )
            dp.createAnnotationFile()
            dp.createMappedData()
            dp.createWindowedData()


if __name__ == "__main__":
    main()



        