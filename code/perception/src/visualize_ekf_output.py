#!/usr/bin/env python

import matplotlib.pyplot as plt
import coordinate_transformation
from math import pi

FILTER_FILE_NAME_START = "70"
FILTER_FILE_NAME_END = "71"

EXCLUDE_FILTER_FILE_START = "30"
EXCLUDE_FILTER_FILE_END = "67"

GT_FILE_NAME = "00"
SENSOR_FILE_NAME = "00"

# open several filter output files
# -> from data_<FILTER_FILE_NAME_START> to data_<FILTER_FILE_NAME_END>
filter_files = []
for i in range(int(FILTER_FILE_NAME_START), int(FILTER_FILE_NAME_END) + 1):
    if i < 10:
        filter_file_nr = "0" + str(i)
    else:
        filter_file_nr = str(i)
    filter_file = open(
        "/workspace/code/perception/src/experiments/filter_output_datasets/data_"
        + filter_file_nr
        + ".csv",
        mode="r",
    )
    # throw away the first line (column names)
    filter_first_line = filter_file.readline()
    filter_files.append(filter_file)

# open the corresponding ground truth file
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
sensor_file = open(
    "/workspace/code/perception/src/experiments/Position_Heading_Datasets/"
    + "sensor_data/data_"
    + SENSOR_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
sensor_first_line = sensor_file.readline()


filter_datasets = []
for filter in filter_files:
    # put filter output data from one filter into a 2-dimensional list
    filter_lines = []
    while True:
        next_line = filter.readline()
        if len(next_line) == 0:
            break
        next_line = next_line.split(",")
        next_line[-1] = next_line[-1].strip()
        new_line = []
        for nr in next_line:
            new_line.append(float(nr))
        filter_lines.append(new_line)
    filter_datasets.append(filter_lines)

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

filter_time_dataset = []
for filter in filter_datasets:
    filter_time_stamps = []
    for line in filter:
        filter_time_stamps.append(line[0])
    filter_time_dataset.append(filter_time_stamps)

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

filter_x_pos_dataset = []
for filter in filter_datasets:
    filter_x_positions = []
    for line in filter:
        filter_x_positions.append(line[1])
    filter_x_pos_dataset.append(filter_x_positions)

gt_x_positions = []
for line in gt_lines:
    gt_x_positions.append(line[1])

sensor_x_positions = []
for line in pos_lines:
    sensor_x_positions.append(line[2])

# y positions

filter_y_pos_dataset = []
for filter in filter_datasets:
    filter_y_positions = []
    for line in filter:
        filter_y_positions.append(line[2])
    filter_y_pos_dataset.append(filter_y_positions)

gt_y_positions = []
for line in gt_lines:
    gt_y_positions.append(line[2])

sensor_y_positions = []
for line in pos_lines:
    sensor_y_positions.append(line[3])

# z positions

filter_z_pos_dataset = []
for filter in filter_datasets:
    filter_z_positions = []
    for line in filter:
        filter_z_positions.append(line[3])
    filter_z_pos_dataset.append(filter_z_positions)

gt_z_positions = []
for line in gt_lines:
    gt_z_positions.append(line[3])

sensor_z_positions = []
for line in pos_lines:
    sensor_z_positions.append(line[4])

# headings

filter_heading_dataset = []
for filter in filter_datasets:
    filter_headings = []
    for line in filter:
        filter_headings.append(line[4])
    filter_heading_dataset.append(filter_headings)

gt_headings = []
for line in gt_lines:
    heading_in_degrees = line[5]
    heading_in_rad = heading_in_degrees * pi / 180.0
    gt_headings.append(heading_in_rad)

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
    for i in range(len(filter_datasets)):
        # make sure the filter files between EXCLUDE_FILTER_FILE_START
        # and EXCLUDE_FILTER_FILE_END are not plotted
        current_filter_file_nr = i + int(FILTER_FILE_NAME_START)
        if current_filter_file_nr >= int(
            EXCLUDE_FILTER_FILE_START
        ) and current_filter_file_nr <= int(EXCLUDE_FILTER_FILE_END):
            continue

        if int(FILTER_FILE_NAME_START) < 10:
            filter_file_nr = "0" + str(i + int(FILTER_FILE_NAME_START))
        else:
            filter_file_nr = str(int(FILTER_FILE_NAME_START) + i)
        label = "ekf x position " + filter_file_nr
        plt.plot(
            filter_time_dataset[i],
            filter_x_pos_dataset[i],
            label=label,
        )
    plt.plot(gt_time_stamps, gt_x_positions, label="gt x positions")
    # plt.plot(sensor_pos_time_stamps, sensor_x_positions, label="sensor x positions")
    plt.plot()
    plt.legend()
    plt.show()


def plot_y_position():
    for i in range(len(filter_datasets)):
        # make sure the filter files between EXCLUDE_FILTER_FILE_START
        # and EXCLUDE_FILTER_FILE_END are not plotted
        current_filter_file_nr = i + int(FILTER_FILE_NAME_START)
        if current_filter_file_nr >= int(
            EXCLUDE_FILTER_FILE_START
        ) and current_filter_file_nr <= int(EXCLUDE_FILTER_FILE_END):
            continue

        if int(FILTER_FILE_NAME_START) < 10:
            filter_file_nr = "0" + str(i + int(FILTER_FILE_NAME_START))
        else:
            filter_file_nr = str(int(FILTER_FILE_NAME_START) + i)
        label = "ekf y position " + filter_file_nr
        plt.plot(
            filter_time_dataset[i],
            filter_y_pos_dataset[i],
            label=label,
        )
    plt.plot(gt_time_stamps, gt_y_positions, label="gt y positions")
    plt.plot(sensor_pos_time_stamps, sensor_y_positions, label="sensor y positions")
    plt.plot()
    plt.legend()
    plt.show()


def plot_z_position():
    for i in range(len(filter_datasets)):
        # make sure the filter files between EXCLUDE_FILTER_FILE_START
        # and EXCLUDE_FILTER_FILE_END are not plotted
        current_filter_file_nr = i + int(FILTER_FILE_NAME_START)
        if current_filter_file_nr >= int(
            EXCLUDE_FILTER_FILE_START
        ) and current_filter_file_nr <= int(EXCLUDE_FILTER_FILE_END):
            continue

        if int(FILTER_FILE_NAME_START) < 10:
            filter_file_nr = "0" + str(i + int(FILTER_FILE_NAME_START))
        else:
            filter_file_nr = str(int(FILTER_FILE_NAME_START) + i)
        label = "ekf z position " + filter_file_nr
        plt.plot(
            filter_time_dataset[i],
            filter_z_pos_dataset[i],
            label=label,
        )
    plt.plot(gt_time_stamps, gt_z_positions, label="gt z positions")
    plt.plot(sensor_pos_time_stamps, sensor_z_positions, label="sensor z positions")
    plt.plot()
    plt.legend()
    plt.show()


def plot_heading():
    for i in range(len(filter_datasets)):
        # make sure the filter files between EXCLUDE_FILTER_FILE_START
        # and EXCLUDE_FILTER_FILE_END are not plotted
        current_filter_file_nr = i + int(FILTER_FILE_NAME_START)
        if current_filter_file_nr >= int(
            EXCLUDE_FILTER_FILE_START
        ) and current_filter_file_nr <= int(EXCLUDE_FILTER_FILE_END):
            continue

        if int(FILTER_FILE_NAME_START) < 10:
            filter_file_nr = "0" + str(i + int(FILTER_FILE_NAME_START))
        else:
            filter_file_nr = str(int(FILTER_FILE_NAME_START) + i)
        label = "ekf heading " + filter_file_nr
        plt.plot(
            filter_time_dataset[i],
            filter_heading_dataset[i],
            label=label,
        )
    plt.plot(gt_time_stamps, gt_headings, label="gt heading")
    plt.plot(sensor_imu_time_stamps, sensor_headings, label="sensor headings")
    plt.plot()
    plt.legend()
    plt.show()


plot_heading()
