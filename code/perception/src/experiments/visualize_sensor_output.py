#!/usr/bin/env python

import matplotlib.pyplot as plt

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

# x positions

filter_x_positions = []
for line in filter_lines:
    filter_x_positions.append(line[1])

gt_x_positions = []
for line in gt_lines:
    gt_x_positions.append(line[1])

# y positions

filter_y_positions = []
for line in filter_lines:
    filter_y_positions.append(line[2])

gt_y_positions = []
for line in gt_lines:
    gt_y_positions.append(line[2])

# z positions

filter_z_positions = []
for line in filter_lines:
    filter_z_positions.append(line[3])

gt_z_positions = []
for line in gt_lines:
    gt_z_positions.append(line[3])

# headings

filter_headings = []
for line in filter_lines:
    filter_headings.append(line[4])

gt_headings = []
for line in gt_lines:
    gt_headings.append(line[5])

plt.plot(filter_time_stamps, filter_x_positions, label="ekf x position")
plt.plot(gt_time_stamps, gt_x_positions, label="gt x positions")
plt.legend()
plt.show()
