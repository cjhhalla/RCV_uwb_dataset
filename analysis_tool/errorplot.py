import numpy as np
import argparse
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
import os
import matplotlib.cm as cm
from matplotlib.colors import Normalize
import yaml
from matplotlib.legend_handler import HandlerPathCollection
from scipy.spatial.transform import Rotation as R
from matplotlib.colors import Normalize

def quaternion_inverse(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def normalize_angle(angle):
    """Normalize an angle to the range [-180, 180] degrees."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (w, x, y, z)

def calculate_relative_yaw(q1, q2):
    q1_inv = quaternion_inverse(q1)
    q_relative = quaternion_multiply(q2, q1_inv)
    w, x, y, z = q_relative
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    yaw_degrees = np.degrees(yaw)
    return yaw_degrees

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])

def calculate_azimuth_angle(row, q2, pos2):
    # 각 기기의 회전 행렬 계산
    q1 = np.array([row['Orientation W'],row['Orientation X'],row['Orientation Y'],row['Orientation Z']])
    pos1 = np.array([row['Position X'], row['Position Y'], row['Position Z']])

    R1 = quaternion_to_rotation_matrix(q1)
    R2 = quaternion_to_rotation_matrix(q2)
    
    # 각 기기의 x축 방향 벡터
    x1_world = R1[:, 0]
    x2_world = R2[:, 0]
    
    # 기기 1에서 기기 2로의 벡터
    vector12 = np.array(pos2) - np.array(pos1)
    
    # 방위각 계산
    azimuth12 = np.arccos(np.dot(vector12, x1_world) / (np.linalg.norm(vector12) * np.linalg.norm(x1_world)))
    azimuth21 = np.arccos(np.dot(-vector12, x2_world) / (np.linalg.norm(vector12) * np.linalg.norm(x2_world)))
    
    return np.degrees(azimuth12), np.degrees(azimuth21)

def calculate_angles(row, pos):
    pos1 = np.array(pos)
    pos2 = np.array([row['Position X'], row['Position Y'], row['Position Z']])
    
    vector = pos1 - pos2
    
    azimuth = np.arctan2(vector[1], vector[0])
    azimuth_degrees = np.degrees(azimuth)
    
    hypotenuse = np.sqrt(vector[0]**2 + vector[1]**2)
    elevation = np.arctan2(vector[2], hypotenuse)
    elevation_degrees = np.degrees(elevation)
    
    return azimuth_degrees, elevation_degrees

parser = argparse.ArgumentParser(description="Plot error data")

parser.add_argument("--anchor1", type=str, required=False, help="Path to the csv file.")
parser.add_argument("--anchor2", type=str, required=False, help="Path to the csv file.")
parser.add_argument("--anchor3", type=str, required=False, help="Path to the csv file.")
parser.add_argument("--anchor4", type=str, required=False, help="Path to the csv file.")
parser.add_argument("--yaml", type=str, required=True, help="Path to the yaml file.")
args = parser.parse_args()

anchor1 = pd.read_csv(args.anchor1)
anchor2 = pd.read_csv(args.anchor2)
anchor3 = pd.read_csv(args.anchor3)
anchor4 = pd.read_csv(args.anchor4)

with open(args.yaml, 'r') as file:
    data = yaml.safe_load(file)
columns = ['Anchor', 'Position X', 'Position Y', 'Position Z', 'Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W']
anchors_df = pd.DataFrame(columns=columns)
for anchor, anchor_data in data.items():
    position = anchor_data[0]['position']['data']
    # orientation = R.from_euler('xyz',anchor_data[1]['orientation']['data'])
    # quaternion = orientation.as_quat()
    # quaternion = quaternion.tolist()    
    quaternion = anchor_data[1]['orientation']['data']
    anchor_info = [anchor] + position + quaternion
    anchor_series = pd.Series(anchor_info, index=columns)
    anchors_df = pd.concat([anchors_df, anchor_series.to_frame().T], ignore_index=True)


y_offset = -0.05
z_offset = 0.045

# Anchor 1
anchor1_pos = np.array([anchors_df.iloc[0]['Position X'], anchors_df.iloc[0]['Position Y']+y_offset, anchors_df.iloc[0]['Position Z']+z_offset])
anchor1_quat = np.array([anchors_df.iloc[0]['Orientation W'], anchors_df.iloc[0]['Orientation X'], anchors_df.iloc[0]['Orientation Y'], anchors_df.iloc[0]['Orientation Z']])

anchor1['Computed Distance'] = anchor1.apply(lambda row: np.linalg.norm(anchor1_pos - np.array([row['Position X'], row['Position Y'], row['Position Z']])), axis=1)
anchor1.loc[anchor1['Range']>0, 'Distance Error'] = (anchor1['Range'] - anchor1['Computed Distance'])
anchor1 = anchor1[anchor1['Distance Error'].abs() <= 1]
anchor1['Azimuth Angle'], anchor1['Elevation Angle'] = zip(*anchor1.apply(lambda row: calculate_angles(row, anchor1_pos), axis=1))
anchor1['Azimuth Angle_tag_2_anchor'], anchor1['Azimuth Angle_anchor_2_tag'] = zip(*anchor1.apply(lambda row: calculate_azimuth_angle(row, anchor1_quat, anchor1_pos), axis=1))
anchor1['Relative Yaw'] = anchor1.apply(lambda row: calculate_relative_yaw(anchor1_quat, (row['Orientation W'], row['Orientation X'], row['Orientation Y'], row['Orientation Z'])), axis=1)

# Anchor 2
anchor2_pos = np.array([anchors_df.iloc[1]['Position X'], anchors_df.iloc[1]['Position Y']+y_offset, anchors_df.iloc[1]['Position Z']+z_offset])
anchor2_quat = np.array([anchors_df.iloc[1]['Orientation W'], anchors_df.iloc[1]['Orientation X'], anchors_df.iloc[1]['Orientation Y'], anchors_df.iloc[1]['Orientation Z']])

anchor2['Computed Distance'] = anchor2.apply(lambda row: np.linalg.norm(anchor2_pos - np.array([row['Position X'], row['Position Y'], row['Position Z']])), axis=1)
anchor2.loc[anchor2['Range']>0, 'Distance Error'] = (anchor2['Range'] - anchor2['Computed Distance'])
anchor2 = anchor2[anchor2['Distance Error'].abs() <= 1]
anchor2['Azimuth Angle'], anchor2['Elevation Angle'] = zip(*anchor2.apply(lambda row: calculate_angles(row, anchor2_pos), axis=1))
anchor2['Azimuth Angle_tag_2_anchor'], anchor2['Azimuth Angle_anchor_2_tag'] = zip(*anchor2.apply(lambda row: calculate_azimuth_angle(row, anchor2_quat, anchor2_pos), axis=1))
anchor2['Relative Yaw'] = anchor2.apply(lambda row: calculate_relative_yaw(anchor2_quat, (row['Orientation W'], row['Orientation X'], row['Orientation Y'], row['Orientation Z'])), axis=1)

# Anchor 3
anchor3_pos = np.array([anchors_df.iloc[2]['Position X'], anchors_df.iloc[2]['Position Y']+y_offset, anchors_df.iloc[2]['Position Z']+z_offset])
anchor3_quat = np.array([anchors_df.iloc[2]['Orientation W'], anchors_df.iloc[2]['Orientation X'], anchors_df.iloc[2]['Orientation Y'], anchors_df.iloc[2]['Orientation Z']])

anchor3['Computed Distance'] = anchor3.apply(lambda row: np.linalg.norm(anchor3_pos - np.array([row['Position X'], row['Position Y'], row['Position Z']])), axis=1)
anchor3.loc[anchor3['Range']>0, 'Distance Error'] = (anchor3['Range'] - anchor3['Computed Distance'])
anchor3 = anchor3[anchor3['Distance Error'].abs() <= 1]
anchor3['Azimuth Angle'], anchor3['Elevation Angle'] = zip(*anchor3.apply(lambda row: calculate_angles(row, anchor3_pos), axis=1))
anchor3['Azimuth Angle_tag_2_anchor'], anchor3['Azimuth Angle_anchor_2_tag'] = zip(*anchor3.apply(lambda row: calculate_azimuth_angle(row, anchor3_quat, anchor3_pos), axis=1))
anchor3['Relative Yaw'] = anchor3.apply(lambda row: calculate_relative_yaw(anchor3_quat, (row['Orientation W'], row['Orientation X'], row['Orientation Y'], row['Orientation Z'])), axis=1)

# Anchor 4
anchor4_pos = np.array([anchors_df.iloc[3]['Position X'], anchors_df.iloc[3]['Position Y']+y_offset, anchors_df.iloc[3]['Position Z']+z_offset])
anchor4_quat = np.array([anchors_df.iloc[3]['Orientation W'], anchors_df.iloc[3]['Orientation X'], anchors_df.iloc[3]['Orientation Y'], anchors_df.iloc[3]['Orientation Z']])

anchor4['Computed Distance'] = anchor4.apply(lambda row: np.linalg.norm(anchor4_pos - np.array([row['Position X'], row['Position Y'], row['Position Z']])), axis=1)
anchor4.loc[anchor4['Range']>0, 'Distance Error'] = (anchor4['Range'] - anchor4['Computed Distance'])
anchor4 = anchor4[anchor4['Distance Error'].abs() <= 1]
anchor4['Azimuth Angle'], anchor4['Elevation Angle'] = zip(*anchor4.apply(lambda row: calculate_angles(row, anchor4_pos), axis=1))
anchor4['Azimuth Angle_tag_2_anchor'], anchor4['Azimuth Angle_anchor_2_tag'] = zip(*anchor4.apply(lambda row: calculate_azimuth_angle(row, anchor4_quat, anchor4_pos), axis=1))
anchor4['Relative Yaw'] = anchor4.apply(lambda row: calculate_relative_yaw(anchor4_quat, (row['Orientation W'], row['Orientation X'], row['Orientation Y'], row['Orientation Z'])), axis=1)

def convert_angle(angle):
    return (angle + 360) % 360
def mean_angle(degrees):
    """Compute the mean of angles (in degrees)."""
    radians = np.deg2rad(degrees)
    mean_sin = np.mean(np.sin(radians))
    mean_cos = np.mean(np.cos(radians))
    mean_angle = np.arctan2(mean_sin, mean_cos)
    return np.rad2deg(mean_angle)

anchor1['Converted Relative Yaw'] = convert_angle(anchor1['Relative Yaw'].values)
anchor2['Converted Relative Yaw'] = convert_angle(anchor2['Relative Yaw'].values)
anchor3['Converted Relative Yaw'] = convert_angle(anchor3['Relative Yaw'].values)
anchor4['Converted Relative Yaw'] = convert_angle(anchor4['Relative Yaw'].values)


a1_yaw=mean_angle(anchor1['Converted Relative Yaw'].values)
a1_azi=mean_angle(anchor1['Azimuth Angle'].values)
a1_ele=mean_angle(anchor1['Elevation Angle'].values[300:600])

a2_yaw=mean_angle(anchor2['Converted Relative Yaw'].values)
a2_azi=mean_angle(anchor2['Azimuth Angle'].values)
a2_ele=mean_angle(anchor2['Elevation Angle'].values[300:600])

a3_yaw=mean_angle(anchor3['Converted Relative Yaw'].values)
a3_azi=mean_angle(anchor3['Azimuth Angle'].values)
a3_ele=mean_angle(anchor3['Elevation Angle'].values[300:600])

a4_yaw=mean_angle(anchor4['Converted Relative Yaw'].values)
a4_azi=mean_angle(anchor4['Azimuth Angle'].values)
a4_ele=mean_angle(anchor4['Elevation Angle'].values[300:600])


print(a1_ele)
print(a2_ele)
print(a3_ele)
print(a4_ele)

# a2_yaw=np.mean(anchor2['Relative Yaw'].values)
# a3_yaw=np.mean(anchor3['Relative Yaw'].values)
# a4_yaw=np.mean(anchor4['Relative Yaw'].values)

# print(a1_yaw - 90)
# print(a1_azi)
# print(a1_ele)
def convert_angle(angle):
    return (angle + 360) % 360
fig, ax = plt.subplots()
# Relative Yaw
# # Anchor 1
# anchor1['Converted Relative Yaw'] = convert_angle(anchor1['Relative Yaw'].values)
# ax.scatter(anchor1['Converted Relative Yaw'].values, anchor1['Distance Error'].values, s=20, color='red', 
#            label=fr'Anchor 1 $\beta_A: {a1_ele:.2f}^\degree$')
# anchor2['Converted Relative Yaw'] = convert_angle(anchor2['Relative Yaw'].values)
# # # Anchor 2
# ax.scatter(anchor2['Converted Relative Yaw'].values, anchor2['Distance Error'].values, s=20, color='green', 
#            label=fr'Anchor 2 $\beta_A: {a2_ele:.2f}^\degree$')
# anchor3['Converted Relative Yaw'] = convert_angle(anchor3['Relative Yaw'].values)
# # # Anchor 3
# ax.scatter(anchor3['Converted Relative Yaw'].values, anchor3['Distance Error'].values, s=20, color='blue', 
#            label=fr'Anchor 3 $\beta_A: {a3_ele:.2f}^\degree$')
# anchor4['Converted Relative Yaw'] = convert_angle(anchor4['Relative Yaw'].values)
# # # Anchor 4
# ax.scatter(anchor4['Converted Relative Yaw'].values, anchor4['Distance Error'].values, s=20, color='purple', 
#            label=fr'Anchor 4 $\beta_A: {a4_ele:.2f}^\degree$')
# Azimuth
# Anchor 1
# anchor1['Converted Azimuth Angle'] = convert_angle(anchor1['Azimuth Angle'].values)
# ax.scatter(anchor1['Azimuth Angle'], anchor1['Distance Error'].values, s=20, color='red', 
#            label=fr'Anchor 1 $\gamma: {a1_yaw:.2f}^\degree$')

# # Anchor 2
# anchor2['Converted Azimuth Angle'] = convert_angle(anchor2['Azimuth Angle'].values)
# ax.scatter(anchor2['Azimuth Angle'], anchor2['Distance Error'].values, s=20, color='green', 
#            label=fr'Anchor 2 $\gamma: {a2_yaw:.2f}^\degree$')

# # Anchor 3
# anchor3['Converted Azimuth Angle'] = convert_angle(anchor3['Azimuth Angle'].values)
# ax.scatter(anchor3['Azimuth Angle'], anchor3['Distance Error'].values, s=20, color='blue', 
#            label=fr'Anchor 3 $\gamma: {a3_yaw:.2f}^\degree$')

# # Anchor 4
# anchor4['Converted Azimuth Angle'] = convert_angle(anchor4['Azimuth Angle'].values)
# ax.scatter(anchor4['Azimuth Angle'], anchor4['Distance Error'].values, s=20, color='purple', 
#            label=fr'Anchor 4 $\gamma: {a4_yaw:.2f}^\degree$')
# # Elevation
# Anchor 1
ax.scatter(anchor1['Elevation Angle'].values, anchor1['Distance Error'].values, s=20, color='red', 
           label=fr'Anchor 1 $\gamma: {a1_yaw:.2f}^\degree$')

# Anchor 2
ax.scatter(anchor2['Elevation Angle'].values, anchor2['Distance Error'].values, s=20, color='green', 
           label=fr'Anchor 2 $\gamma: {a2_yaw:.2f}^\degree$')

# Anchor 3
ax.scatter(anchor3['Elevation Angle'].values, anchor3['Distance Error'].values, s=20, color='blue', 
           label=fr'Anchor 3 $\gamma: {a3_yaw:.2f}^\degree$')

# Anchor 4
ax.scatter(anchor4['Elevation Angle'].values, anchor4['Distance Error'].values, s=20, color='purple', 
           label=fr'Anchor 4 $\gamma: {a4_yaw:.2f}^\degree$')

# # Range
# # Anchor 1
# ax.scatter(anchor1['Range'].values, anchor1['Distance Error'].values, s=20, color='red', 
#            label=fr'Anchor 1 $\beta_A: {a1_ele:.2f}^\degree$')

# # Anchor 2
# ax.scatter(anchor2['Range'].values, anchor2['Distance Error'].values, s=20, color='green', 
#            label=fr'Anchor 2 $\beta_A: {a2_ele:.2f}^\degree$')

# # Anchor 3
# ax.scatter(anchor3['Range'].values, anchor3['Distance Error'].values, s=20, color='blue', 
#            label=fr'Anchor 3 $\beta_A: {a3_ele:.2f}^\degree$')

# # Anchor 4
# ax.scatter(anchor4['Range'].values, anchor4['Distance Error'].values, s=20, color='purple', 
#            label=fr'Anchor 4 $\beta_A: {a4_ele:.2f}^\degree$')
# norm = Normalize(vmin=0, vmax=0.1)
# ##Last
# anchor1['Converted Azimuth Angle'] = convert_angle(anchor1['Azimuth Angle'])
# anchor2['Converted Azimuth Angle'] = convert_angle(anchor2['Azimuth Angle'])
# anchor3['Converted Azimuth Angle'] = convert_angle(anchor3['Azimuth Angle'])
# anchor4['Converted Azimuth Angle'] = convert_angle(anchor4['Azimuth Angle'])
# print(anchor1['Converted Azimuth Angle'])
# # Anchor 1
# sc1 = ax.scatter(anchor1['Converted Azimuth Angle'], anchor1['Elevation Angle'], c=anchor1['Distance Error'], s=20, cmap='viridis', label=fr'Anchor 1 $\gamma: {a1_yaw:.2f}^\degree$')

# # Anchor 2
# sc2 = ax.scatter(anchor2['Converted Azimuth Angle'], anchor2['Elevation Angle'], c=anchor2['Distance Error'], s=20, cmap='plasma', label=fr'Anchor 2 $\gamma: {a2_yaw:.2f}^\degree$')

# # Anchor 3
# sc3 = ax.scatter(anchor3['Converted Azimuth Angle'], anchor3['Elevation Angle'], c=anchor3['Distance Error'], s=20, cmap='inferno', label=fr'Anchor 3 $\gamma: {a3_yaw:.2f}^\degree$')

# # Anchor 4
# sc4 = ax.scatter(anchor4['Converted Azimuth Angle'], anchor4['Elevation Angle'], c=anchor4['Distance Error'], s=20, cmap='magma', label=fr'Anchor 4 $\gamma: {a4_yaw:.2f}^\degree$')

# # 컬러바 추가
# cbar1 = plt.colorbar(sc1, ax=ax)
# cbar1.set_label('Distance Error')


# Relative Yaw
# ax.set_xlabel(r'Azimuth Angle $\alpha_A^\degree$', fontsize=30, labelpad=0)
# ax.set_ylabel(r'Elevation Angle $\beta_A^\degree$', fontsize=35, labelpad=10)
# ax.set_title(r'Sequence 6 $\alpha_A^\degree$, $\beta_A^\degree$ bias pattern' , fontsize=45, pad=20)
# Azimuth
# ax.set_xlabel(r'Azimuth Angle $\alpha_T^\degree$', fontsize=30, labelpad=0)
# ax.set_ylabel('Bias [m]', fontsize=35, labelpad=10)
# ax.set_title('Sequence 2 Azimuth Anlge bias pattern', fontsize=45, pad=20)
# Elevation
ax.set_xlabel(r'Elevation Angle $\beta_T^\degree$', fontsize=30, labelpad=0)
ax.set_ylabel('Bias [m]', fontsize=35, labelpad=10)
ax.set_title('Sequence 6 Elevation Anlge bias pattern', fontsize=45, pad=20)
# Range
# ax.set_xlabel(r'UWB ranging measurement ${\tilde{d}}$', fontsize=30, labelpad=0)
# ax.set_ylabel('Bias [m]', fontsize=35, labelpad=10)
# ax.set_title('Sequence 5 Range bias pattern', fontsize=45, pad=20)


ax.tick_params(axis='x', labelsize=30)
ax.tick_params(axis='y', labelsize=30)
# ax.set_xlim(-30,15)
ax.xaxis.set_major_locator(plt.MultipleLocator(10))
# ax.set_ylim(-0.6,1.0)

legend = ax.legend(fontsize=20, loc='lower right')
for handle in legend.legendHandles:
    handle.set_sizes([100])

plt.show()
