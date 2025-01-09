#!/usr/bin/env python

base_path = "/workspace/code/perception/src/experiments"

gt_file_nr = "00"
filter_start_nr = "00"
filter_end_nr = "28"
nr_of_digits = 6

gt_file = open(
    base_path + "/Position_Heading_Datasets/ground_truth/data_" + gt_file_nr + ".csv"
)
gt_file.readline()  # throw away the first line (column names)

gt_dataset = []
# convert ground truth data into a 2-dimensional list of doubles
for line in gt_file:
    next_line = line
    next_line = next_line.split(",")
    next_line[-1] = next_line[-1].strip()
    new_line = []
    for nr in next_line:
        new_line.append(float(nr))
    gt_dataset.append(new_line)


filter_files = []
for i in range(int(filter_start_nr), int(filter_end_nr) + 1):
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

filter_datasets = []
for filter in filter_files:
    # put filter output data into a list of 2-dimensional lists of doubles
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

# calculate the mean squared error (MSE) for each filter output dataset
gt_line_index = 0
# for filter in filter_datasets:
# filter = filter_datasets[0]
data_set_nr = int(filter_start_nr)

for filter in filter_datasets:

    line_count = 0
    sum_x_error = 0
    sum_y_error = 0
    sum_z_error = 0
    sum_heading_error = 0

    for line in filter:
        filter_time = round(line[0], nr_of_digits)
        gt_time = round(gt_dataset[gt_line_index][0], nr_of_digits)

        # find the (first) line with the same time in the ground truth
        while filter_time > gt_time:
            gt_line_index += 1
            gt_time = round(gt_dataset[gt_line_index][0], nr_of_digits)

        # calculate the errors and add them to the sum
        x_error = (line[1] - gt_dataset[gt_line_index][1]) ** 2
        y_error = (line[2] - gt_dataset[gt_line_index][2]) ** 2
        z_error = (line[3] - gt_dataset[gt_line_index][3]) ** 2
        heading_error = (line[4] - gt_dataset[gt_line_index][5]) ** 2
        sum_x_error += x_error
        sum_y_error += y_error
        sum_z_error += z_error
        sum_heading_error += heading_error

        line_count += 1

    mse_x = sum_x_error / line_count
    mse_y = sum_y_error / line_count
    mse_z = sum_z_error / line_count
    mse_heading = sum_heading_error / line_count

    if data_set_nr < 10:
        nr = "0" + str(data_set_nr)
    else:
        nr = str(data_set_nr)
    print(
        "dataset "
        + str(nr)
        + " mse -> "
        + "x: "
        + str(round(mse_x, 2))
        + ", \t"
        + "y: "
        + str(round(mse_y, 2))
        + ", \t"
        + "z: "
        + str(round(mse_z, 2))
        + ", \t"
        + "heading: "
        + str(round(mse_heading, 2))
    )
    data_set_nr += 1
