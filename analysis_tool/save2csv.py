#!/usr/bin/env python
import rospy
from uwb.msg import UwbData 
from geometry_msgs.msg import PoseStamped
import rosbag
import pandas as pd
from datetime import datetime
import argparse
import os
import numpy as np
parser = argparse.ArgumentParser(description="Process a ROS bag file.")

parser.add_argument("--bag_file", type=str, required=True, help="Path to the ROS bag file.")
parser.add_argument("--output", type=str, required=True, help="Path to save csv file")
parser.add_argument("--gt", type=str, required=True, help="Path to save gt csv file")
args = parser.parse_args()

base_path, extension = os.path.splitext(args.output)

bag = rosbag.Bag(args.bag_file)

df_uwb_1 = pd.DataFrame(columns=['Time', 'Address','Range','Rxpower'])
df_uwb_2 = pd.DataFrame(columns=['Time', 'Address','Range','Rxpower'])
df_uwb_3 = pd.DataFrame(columns=['Time', 'Address','Range','Rxpower'])
df_uwb_4 = pd.DataFrame(columns=['Time', 'Address','Range','Rxpower'])
df_motion = pd.DataFrame(columns=[
    'Time', 
    'Position X', 'Position Y', 'Position Z', 
    'Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W'
])


for index, (topic, msg, t) in enumerate(bag.read_messages(topics=['/UwbData_tag_1'])):
    time = datetime.fromtimestamp(t.to_sec())
    Address = msg.Address
    Range = msg.range 
    Rxpower = msg.rxPower
    if (Address == 81):
        df_uwb_1.loc[index] = [time,Address ,Range,Rxpower]
    elif (Address == 82):
        df_uwb_2.loc[index] = [time,Address ,Range,Rxpower]
    elif (Address == 83):
        df_uwb_3.loc[index] = [time,Address ,Range,Rxpower]
    elif (Address == 84):
        df_uwb_4.loc[index] = [time,Address ,Range,Rxpower]

position_threshold = 0.001  # Position change threshold
orientation_threshold = 0.001  # Orientation change threshold

prev_position = None
prev_orientation = None
# for index, (topic, msg, t) in enumerate(bag.read_messages(topics=['/qualisys/tag1/pose'])):
for index, (topic, msg, t) in enumerate(bag.read_messages(topics=['/mavros/vision_pose/pose'])):
    time = datetime.fromtimestamp(t.to_sec())
    position = msg.pose.position
    orientation = msg.pose.orientation

    # Convert to numpy arrays for easy computation
    current_position = np.array([position.x, position.y, position.z])
    current_orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])

    # If previous values exist, compare the difference
    if prev_position is not None and prev_orientation is not None:
        position_diff = np.linalg.norm(current_position - prev_position)
        orientation_diff = np.linalg.norm(current_orientation - prev_orientation)

        # Check if the difference is greater than the threshold
        if position_diff > position_threshold or orientation_diff > orientation_threshold:
            df_motion.loc[index] = [
                time,
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ]
    else:
        # If no previous values, save the first pose
        df_motion.loc[index] = [
            time,
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w
        ]

    # Update previous values
    prev_position = current_position
    prev_orientation = current_orientation

bag.close()


# df_motion.set_index('Time', inplace=True)
# df_motion = df_motion.resample('100ms').first().interpolate() 
# df_motion.reset_index(inplace=True)
df_motion.to_csv(args.gt, index=False)

df_combined_1 = pd.merge_asof(df_uwb_1.sort_values('Time'), df_motion.sort_values('Time'), on='Time', tolerance=pd.Timedelta('10ms'), direction='nearest').dropna()
df_combined_2 = pd.merge_asof(df_uwb_2.sort_values('Time'), df_motion.sort_values('Time'), on='Time', tolerance=pd.Timedelta('10ms'), direction='nearest').dropna()
df_combined_3 = pd.merge_asof(df_uwb_3.sort_values('Time'), df_motion.sort_values('Time'), on='Time', tolerance=pd.Timedelta('10ms'), direction='nearest').dropna()
df_combined_4 = pd.merge_asof(df_uwb_4.sort_values('Time'), df_motion.sort_values('Time'), on='Time', tolerance=pd.Timedelta('10ms'), direction='nearest').dropna()

# CSV 파일로 저장
df_combined_1.to_csv(f'{base_path}_1{extension}', index=False)
df_combined_2.to_csv(f'{base_path}_2{extension}', index=False)
df_combined_3.to_csv(f'{base_path}_3{extension}', index=False)
df_combined_4.to_csv(f'{base_path}_4{extension}', index=False)
