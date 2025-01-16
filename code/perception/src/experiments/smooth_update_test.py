#!/usr/bin/env python

# import matplotlib.pyplot as plt
# from math import pi
import os

# import rospy
import csv

FILTER_FILE_NAME = "70"
GROUP_SIZE = 3


# open the filter file which data you would like to smooth
filter_file = open(
    "/workspace/code/perception/src/experiments/filter_output_datasets/data_"
    + FILTER_FILE_NAME
    + ".csv",
    mode="r",
)
# throw away the first line (column names)
first_line = filter_file.readline()


# put all data into a 2-dimensional list of floats
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


counter = 1
smoothed_x_sum = 0
smoothed_y_sum = 0
smoothed_z_sum = 0
smoothed_h_sum = 0
smoothed_filter_data = []
for i in range(len(filter_lines)):
    if counter <= GROUP_SIZE:
        smoothed_x_sum += filter_lines[i][1]
        smoothed_y_sum += filter_lines[i][2]
        smoothed_z_sum += filter_lines[i][3]
        smoothed_h_sum += filter_lines[i][4]
    if counter == GROUP_SIZE:
        # write data into smoothed_filter_data
        smoothed_x = smoothed_x_sum / GROUP_SIZE
        smoothed_y = smoothed_y_sum / GROUP_SIZE
        smoothed_z = smoothed_z_sum / GROUP_SIZE
        smoothed_h = smoothed_h_sum / GROUP_SIZE
        line = [filter_lines[i][0], smoothed_x, smoothed_y, smoothed_z, smoothed_h]
        smoothed_filter_data.append(line)
        counter = 0
        smoothed_x_sum = 0
        smoothed_y_sum = 0
        smoothed_z_sum = 0
        smoothed_h_sum = 0
    counter += 1


# write calculated data into .csv file


def create_file(folder_path):
    """
    This function creates a new csv file in the folder_path
    in correct sequence looking like data_00.csv, data_01.csv, ...
    and returns the path to the file.
    """
    i = 0
    while True:
        file_path = f"{folder_path}/data_{str(i).zfill(2)}.csv"
        if not os.path.exists(file_path):
            with open(file_path, "w", newline=""):
                pass
            return file_path
        i += 1


# Specify the path to the folder where you want to save the data
path = "/workspace/code/perception/src/experiments/filter_output_datasets"
# Ensure the directories exist
os.makedirs(path, exist_ok=True)

new_filter_data_csv_file_path = create_file(path)

with open(new_filter_data_csv_file_path, "a", newline="") as file:
    writer = csv.writer(file)
    # add first row
    writer.writerow(
        [
            "Time",
            "pos x",
            "pos y",
            "pos z",
            "heading",
        ]
    )
    for i in range(len(smoothed_filter_data)):
        data_line_to_write = [
            smoothed_filter_data[i][0],
            smoothed_filter_data[i][1],
            smoothed_filter_data[i][2],
            smoothed_filter_data[i][3],
            smoothed_filter_data[i][4],
        ]
        writer.writerow(data_line_to_write)
