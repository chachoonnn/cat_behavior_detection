import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

def plot_accel_gyro(df, ddd=None):
    fig, axes = plt.subplots(3, 1, figsize=(16, 9), sharex=True)
    df.dropna(subset=['AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ'], inplace=True)
    
    # # Format x-axis for better readability
    # axes[2].xaxis.set_major_locator(mdates.AutoDateLocator())
    # axes[2].xaxis.set_major_formatter(mdates.DateFormatter('%M:%SSSS'))
    
    # Plot Acceleration over Time
    axes[0].plot(df['Time']/1000, df['AccelX'], label='AccelX', marker='.', linestyle='-')
    axes[0].plot(df['Time']/1000, df['AccelY'], label='AccelY', marker='.', linestyle='-')
    axes[0].plot(df['Time']/1000, df['AccelZ'], label='AccelZ', marker='.', linestyle='-')
    axes[0].legend()
    axes[0].grid(True)
    
    # Plot Gyroscope over Time
    axes[1].plot(df['Time']/1000, df['GyroX'], label='GyroX', marker='.', linestyle='-')
    axes[1].plot(df['Time']/1000, df['GyroY'], label='GyroY', marker='.', linestyle='-')
    axes[1].plot(df['Time']/1000, df['GyroZ'], label='GyroZ', marker='.', linestyle='-')
    axes[1].legend()
    axes[1].grid(True)
    
    # Plot Behavior over Time
    if ddd is not None:
        axes[2].scatter(ddd['DataTime_s'], ddd['Behavior'], color='red', label='Behavior', marker='|')
        axes[2].set_xlabel('Time')
        axes[2].legend()
        axes[2].grid(True)
    
    plt.tight_layout()
    plt.show()

# Load data from file
file_path = 'data/aegon_20250121_2346'
file_path = 'data/aegon_20250122_0004'
# file_path = 'data/aegon_20250122_2139'
# file_path = 'data/aegon_20250123_0029'
# file_path = 'data/aegon_20250123_2155'
df_device = pd.read_csv(file_path + '/device_data.TXT')
df_behav = pd.read_csv(file_path + '/annotation.csv')

plot_accel_gyro(df_device, df_behav)

# Interactive loop to update time
while True:
    try:
        increment = float(input("Enter a float value to add to time (999 to stop): "))
    
        df_device['Time'] = df_device['Time'] + increment*1000
        plot_accel_gyro(df_device, df_behav)
    except ValueError:
        print("Invalid input. Please enter a float number.")
        break

#1 -> -7 sec
#2 -> -5.25 sec
#3 -> -7 sec
#4 -> -7
#5 -> -4