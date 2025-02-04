import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder, StandardScaler
from sklearn.tree import DecisionTreeClassifier
from sklearn.metrics import confusion_matrix, classification_report
from DataPrepare import WindowSensorData

import m2cgen as m2c
import os

def getAllDirectory( path='.' ):
    """
    Get all directory names in the given path.

    :param path: The path to scan for directories (default is the current directory).
    :return: A list of directory names.
    """
    try:
        # List all entries in the given path and filter for directories
        directory_names = [path + name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name))]
        return directory_names
    except Exception as e:
        print(f"An error occurred: {e}")
        return []
    
def create_cpp_file( model, name ):
    cpp_code = m2c.export_to_c(model)
    with open(name, "w") as f:
        f.write(cpp_code)
    size = os.path.getsize(name)/1028
    print(f"File: {name}, size: {size} KB")

def filter_low_variance_windows(windowed_df, std_threshold=0.02):
    """
    Remove windows whose average acceleration/gyro standard deviation 
    is below 'std_threshold', indicating near-complete stillness.
    """
    # We'll combine the std columns: 'AccelXStd','AccelYStd','AccelZStd','GyroXStd','GyroYStd','GyroZStd'
    std_cols = [c for c in windowed_df.columns if c.endswith('Std')]
    filtered_rows = []
    for i, row in windowed_df.iterrows():
        # compute the average of these 6 std columns
        mean_std = np.mean([row[col] for col in std_cols])
        if mean_std >= std_threshold:
            filtered_rows.append(row)
        else:
            # It's extremely stable => skip or keep fewer
            pass
    return pd.DataFrame(filtered_rows)

def downsample_stable_windows(windowed_df, keep_ratio=0.1):
    """
    Keep only a fraction (e.g. 10%) of consecutive stable windows 
    with the same label to reduce duplication.
    """
    new_rows = []
    prev_label = None
    stable_count = 0

    for i, row in windowed_df.iterrows():
        label = row['Behavior']
        if label == prev_label:
            # We are in a stable segment of the same label
            stable_count += 1
            # Keep only a fraction of these stable windows:
            if np.random.rand() < keep_ratio:
                new_rows.append(row)
        else:
            # Label changed => definitely keep this row
            new_rows.append(row)
            stable_count = 0
        prev_label = label

    return pd.DataFrame(new_rows).reset_index(drop=True)


def DesitionTree( df, fileName ):
    ################################################
    #
    #   have stand, sit, lay down
    #
    # 1) Load your data into a DataFrame
    # df = pd.read_csv(data_dir + "/sample_data.csv")  # Use your dataset file
    # 2) Select the features and label
    feature_cols = [
        "AccelXMean", "AccelXStd", "AccelXMin", "AccelXMax",
        "AccelYMean", "AccelYStd", "AccelYMin", "AccelYMax",
        "AccelZMean", "AccelZStd", "AccelZMin", "AccelZMax",
        "GyroXMean",  "GyroXStd",  "GyroXMin",  "GyroXMax",
        "GyroYMean",  "GyroYStd",  "GyroYMin",  "GyroYMax",
        "GyroZMean",  "GyroZStd",  "GyroZMin",  "GyroZMax"
        # Adjust if you have fewer/more features
    ]
    X = df[feature_cols].values

    # 3) Encode labels (e.g. 'stand', 'walk', 'run', etc.)
    label_encoder = LabelEncoder()
    y = label_encoder.fit_transform(df["Behavior"])  # e.g. stand=0, walk=1, run=2, etc.
    print("const char* behaviors[3] = ", set(label_encoder.classes_),';\n')
    # 4) Scale features (optional but recommended)
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)
    
    print('Mean: ', scaler.mean_)
    print('Scale: ', scaler.scale_)
        

    # 5) Split data for training & testing
    X_train, X_test, y_train, y_test = train_test_split(
        X_scaled, y, test_size=0.2, random_state=42
    )
    ################
    from sklearn.utils.class_weight import compute_class_weight

    unique_classes = np.unique(y_train)
    class_weights = compute_class_weight('balanced', classes=unique_classes, y=y_train)

    # Convert to a dictionary: {class_label: weight, ...}
    class_weight_dict = {i: w for i, w in zip(unique_classes, class_weights)}

    #################
    # Create and train a decision tree classifier.
    # We set some constraints (max_depth, min_samples_leaf) to keep the tree small.
    dt = DecisionTreeClassifier(
        max_depth=15,
        min_samples_leaf=20,
        random_state=42,
        # min_samples_split=20,  # tweak
        # criterion='entropy',   # try 'entropy'
        # class_weight=class_weight_dict
    )
    dt.fit(X_train, y_train)

    # Evaluate on test set
    test_acc = dt.score(X_test, y_test)
    print(f"Test accuracy: {test_acc*100:.2f}%")

    # Show confusion matrix and classification report
    from sklearn.tree import export_text

    y_pred = dt.predict(X_test)
    print("Confusion Matrix:")
    print(confusion_matrix(y_test, y_pred))
    print("Classification Report:")
    print(classification_report(y_test, y_pred))

    ####################
    tree_rules = export_text(dt, feature_names=feature_cols)
    # print(tree_rules)

    ###################

    create_cpp_file(dt, f'{fileName}.c')
    return dt


folders = ['data/aegon_20250121_2346', 'data/aegon_20250122_0004', 'data/aegon_20250122_2139', 'data/aegon_20250123_0029', 'data/aegon_20250123_2155']


# df_list = []
# df_no_sit_list = []
# for folder in folders:
#     # mappedFile = folder + '/new_mapped_data.csv'
#     mappedFile = folder + '/mapped_data.csv'
#     df = pd.read_csv(mappedFile)
#     # print(df)
#     df = df.drop(columns=['MagX', 'MagY', 'MagZ'])
#     df_no_sit = df[df['Behavior'].isin([ 'stand', 'lay down'])]
#     df = df[df['Behavior'].isin([ 'stand', 'sit', 'lay down'])]

#     windowed_df = WindowSensorData(df, windowSizeSec=4.0, stepSizeSec=0.5)
#     windowed_df_no_sit = WindowSensorData(df_no_sit, windowSizeSec=3.0, stepSizeSec=0.5)

#     df_list.append(windowed_df)
#     df_no_sit_list.append(windowed_df_no_sit)
# df_combined = pd.concat(df_list, ignore_index=True)
# df_no_sit_combined = pd.concat(df_no_sit_list, ignore_index=True)


# dt1 = DesitionTree(df_combined, 'desitionTree_model_new_2')
# dt2 = DesitionTree(df_no_sit_combined, 'desitionTree_model_1245')

df_list_no_stable = []
df_no_low_variance = []
for folder in folders:
    # mappedFile = folder + '/new_mapped_data.csv'
    mappedFile = folder + '/mapped_data.csv'
    df = pd.read_csv(mappedFile)
    # print(df)
    df = df.drop(columns=['MagX', 'MagY', 'MagZ'])
    df = df[df['Behavior'].isin([ 'stand', 'sit', 'lay down'])]

    windowed_df = WindowSensorData(df, windowSizeSec=3.0, stepSizeSec=0.5)
    windowed_df_no_stable = downsample_stable_windows(windowed_df, keep_ratio=0.1)
    windowed_df_no_low_variance = filter_low_variance_windows(windowed_df, std_threshold=0.02)

    df_list_no_stable.append(windowed_df_no_stable)
    df_no_low_variance.append(windowed_df_no_low_variance)
df_combined_no_stable = pd.concat(df_list_no_stable, ignore_index=True)
df_combined_no_low_variance = pd.concat(df_no_low_variance, ignore_index=True)


# dt1 = DesitionTree(df_combined_no_stable, 'desitionTree_model_no_stable')
dt2 = DesitionTree(df_combined_no_low_variance, 'desitionTree_model_no_low_variance_1')
# dt2 = DesitionTree(df_no_sit_combined, 'desitionTree_model_1245')
