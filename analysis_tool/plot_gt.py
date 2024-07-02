import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as ticker
import yaml
from scipy.spatial.transform import Rotation as R

# argparse
parser = argparse.ArgumentParser(description="Process a gt file.")
parser.add_argument("--gt", type=str, required=True, help="Path to the gt file.")
parser.add_argument("--yaml", type=str, required=False, help="Path to the yaml file.")
args = parser.parse_args()

# read csv file
df = pd.read_csv(args.gt)

# read yaml file
with open(args.yaml, 'r') as file:
    data = yaml.safe_load(file)

columns = ['Anchor', 'Position X', 'Position Y', 'Position Z', 'Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W']
anchors_df = pd.DataFrame(columns=columns)
for anchor, anchor_data in data.items():
    position = anchor_data[0]['position']['data']
    quaternion = anchor_data[1]['orientation']['data']
    anchor_info = [anchor] + position + quaternion
    anchor_series = pd.Series(anchor_info, index=columns)
    anchors_df = pd.concat([anchors_df, anchor_series.to_frame().T], ignore_index=True)

time = pd.to_datetime(df['Time'])
position_x = df['Position X']
position_y = df['Position Y']
position_z = df['Position Z']
quaternions = df[['Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W']].values

# Convert quaternions to yaw angles
rotations = R.from_quat(quaternions)
euler_angles = rotations.as_euler('xyz', degrees=True)
yaw_angles = euler_angles[:, 2]  # Extract yaw (heading) angle

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

sc = ax.scatter(position_x, position_y, position_z, c=time, cmap='viridis')

ax.set_xlabel('X [m]', fontsize=40, labelpad=35)
ax.set_ylabel('Y [m]', fontsize=40, labelpad=35)
ax.set_zlabel('Z [m]', fontsize=40, labelpad=20)

all_positions_x = pd.concat([position_x, anchors_df['Position X']])
all_positions_y = pd.concat([position_y, anchors_df['Position Y']])
all_positions_z = pd.concat([position_z, anchors_df['Position Z']])

max_range = max(all_positions_x.max() - all_positions_x.min(),
                all_positions_y.max() - all_positions_y.min(),
                all_positions_z.max() - all_positions_z.min())

mid_x = (all_positions_x.max() + all_positions_x.min()) * 0.5
mid_y = (all_positions_y.max() + all_positions_y.min()) * 0.5
mid_z = (all_positions_z.max() + all_positions_z.min()) * 0.5

ax.set_xlim(mid_x - max_range * 0.5, mid_x + max_range * 0.5)
ax.set_ylim(mid_y - max_range * 0.5, mid_y + max_range * 0.5)
ax.set_zlim(mid_z - max_range * 0.5, mid_z + max_range * 0.5)
# ax.set_ylim(-1,1)
# ax.set_zlim(0,2)

ax.xaxis.set_major_locator(plt.MultipleLocator(1))
ax.yaxis.set_major_locator(plt.MultipleLocator(1))
ax.zaxis.set_major_locator(plt.MultipleLocator(1))

color = ['red', 'green', 'blue', 'purple']

for i, row in anchors_df.iterrows():
    ax.scatter(row['Position X'], row['Position Y'], row['Position Z'], color=color[i], s=300)

# Plot yaw directions
arrow_length = 0.5  # Length of the arrows representing yaw direction
for i, (x, y, z, yaw) in enumerate(zip(position_x, position_y, position_z, yaw_angles)):
    if i % 500 == 0:
        dx = arrow_length * np.cos(np.deg2rad(yaw))
        dy = arrow_length * np.sin(np.deg2rad(yaw))
        ax.quiver(x, y, z, dx, dy, 0, color='red', arrow_length_ratio=0.3)

ax.view_init(elev=30, azim=330)
handles, labels = ax.get_legend_handles_labels()
by_label = dict(zip(labels, handles))
ax.legend(by_label.values(), by_label.keys(), fontsize=20, ncol=5)

plt.title('Sequence 6', fontsize=50, pad=10, x=0.3, y=0.95)
ax.tick_params(axis='both', which='major', labelsize=40)
plt.show()
