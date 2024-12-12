#!/usr/bin/env python

import matplotlib.pyplot as plt
import coordinate_transformation

# open a filter output file
FILTER_FILE_NAME = "07"
filter_file = open(
    "/workspace/code/perception/src/experiments/filter_output_datasets/data_"
    + FILTER_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
filter_first_line = filter_file.readline()

# open the corresponding ground truth file
GT_FILE_NAME = "00"
gt_file = open(
    "/workspace/code/perception/src/experiments/Position_Heading_Datasets/"
    + "ground_truth/data_"
    + GT_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
gt_first_line = gt_file.readline()

# open the corresponding sensor data file
SENSOR_FILE_NAME = "00"
sensor_file = open(
    "/workspace/code/perception/src/experiments/Position_Heading_Datasets/"
    + "sensor_data/data_"
    + SENSOR_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
sensor_first_line = sensor_file.readline()


# put all filter output data into a 2-dimensional list
filter_lines = []
while True:
    next_line = filter_file.readline()
    if len(next_line) == 0:
        break
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()
    new_line = []
    for nr in next_line:
        new_line.append(float(nr))
    filter_lines.append(new_line)

# put all ground truth data into a 2-dimensional list
gt_lines = []
while True:
    next_line = gt_file.readline()
    if len(next_line) == 0:
        break
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()
    new_line = []
    for nr in next_line:
        new_line.append(float(nr))
    gt_lines.append(new_line)

# put sensor data into three separate lists
pos_lines = []
speed_lines = []
imu_lines = []
while True:
    next_line = sensor_file.readline()
    if len(next_line) == 0:
        break
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()

    new_line = []
    for nr in next_line:
        if (nr == "pos") or (nr == "speed") or (nr == "imu"):
            new_line.append(nr)
        else:
            new_line.append(float(nr))

    if next_line[1] == "pos":
        pos_lines.append(new_line)
    elif next_line[1] == "speed":
        speed_lines.append(new_line)
    else:
        imu_lines.append(new_line)


"""
put all data (position (x/y/z) and heading)
into 1-dimensional lists
-> pass those to the axes of the plot
"""

# times

filter_time_stamps = []
for line in filter_lines:
    filter_time_stamps.append(line[0])

gt_time_stamps = []
for line in gt_lines:
    gt_time_stamps.append(line[0])

sensor_pos_time_stamps = []
for line in pos_lines:
    sensor_pos_time_stamps.append(line[0])

sensor_imu_time_stamps = []
for line in imu_lines:
    sensor_imu_time_stamps.append(line[0])

# x positions

filter_x_positions = []
for line in filter_lines:
    filter_x_positions.append(line[1])

gt_x_positions = []
for line in gt_lines:
    gt_x_positions.append(line[1])

sensor_x_positions = []
for line in pos_lines:
    sensor_x_positions.append(line[2])

# y positions

filter_y_positions = []
for line in filter_lines:
    filter_y_positions.append(line[2])

gt_y_positions = []
for line in gt_lines:
    gt_y_positions.append(line[2])

sensor_y_positions = []
for line in pos_lines:
    sensor_y_positions.append(line[3])

# z positions

filter_z_positions = []
for line in filter_lines:
    filter_z_positions.append(line[3])

gt_z_positions = []
for line in gt_lines:
    gt_z_positions.append(line[3])

sensor_z_positions = []
for line in pos_lines:
    sensor_z_positions.append(line[4])

# headings

filter_headings = []
for line in filter_lines:
    filter_headings.append(line[4])

gt_headings = []
for line in gt_lines:
    gt_headings.append(line[5])

sensor_headings = []
for line in imu_lines:
    orientation_x = line[6]
    orientation_y = line[7]
    orientation_z = line[8]
    orientation_w = line[9]
    quat = [orientation_x, orientation_y, orientation_z, orientation_w]
    heading = coordinate_transformation.quat_to_heading(quat)
    sensor_headings.append(heading)


def plot_x_position():
    plt.plot(filter_time_stamps, filter_x_positions, label="ekf x position")
    plt.plot(gt_time_stamps, gt_x_positions, label="gt x positions")
    plt.plot(sensor_pos_time_stamps, sensor_x_positions, label="sensor x positions")
    plt.plot()
    plt.legend()
    plt.show()


def plot_y_position():
    plt.plot(filter_time_stamps, filter_y_positions, label="ekf y position")
    plt.plot(gt_time_stamps, gt_y_positions, label="gt y positions")
    plt.plot(sensor_pos_time_stamps, sensor_y_positions, label="sensor y positions")
    plt.plot()
    plt.legend()
    plt.show()


def plot_z_position():
    plt.plot(filter_time_stamps, filter_z_positions, label="ekf z position")
    plt.plot(gt_time_stamps, gt_z_positions, label="gt z positions")
    plt.plot(sensor_pos_time_stamps, sensor_z_positions, label="sensor z positions")
    plt.plot()
    plt.legend()
    plt.show()


def plot_heading():
    plt.plot(filter_time_stamps, filter_headings, label="ekf heading")
    plt.plot(gt_time_stamps, gt_headings, label="gt heading")
    plt.plot(sensor_imu_time_stamps, sensor_headings, label="sensor headings")
    plt.plot()
    plt.legend()
    plt.show()


plot_heading()
