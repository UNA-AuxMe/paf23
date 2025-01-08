#!/usr/bin/env python

from math import atan2, pi  # , sqrt, acos
import coordinate_transformation

base_path = "/workspace/code/perception/src/experiments/Position_Heading_Datasets"
sensor_data_path = base_path + "/sensor_data/data_00.csv"
ground_truth_data_path = base_path + "/ground_truth/data_00.csv"

sensor_data_file = open(sensor_data_path)
ground_truth_data_file = open(ground_truth_data_path)

sensor_data_file.readline()

line_nr = 2
for line in sensor_data_file:
    line = line.split(",")
    line[-1] = line[-1].strip()
    if line[1] == "imu":
        sensor_data_line = line
        break
    line_nr += 1

for line in range(line_nr):
    ground_truth_data_line = ground_truth_data_file.readline()
ground_truth_data_line = ground_truth_data_line.split(",")
ground_truth_data_line[-1] = ground_truth_data_line[-1].strip()


class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __str__(self):
        return f"x: {self.x}\ny: {self.y}\nz: {self.z}\nw: {self.w}\n"


q = Quaternion(
    float(sensor_data_line[6]),
    float(sensor_data_line[7]),
    float(sensor_data_line[8]),
    float(sensor_data_line[9]),
)
print(q)
q.x = -q.x
q.y = -q.y
q.z = -q.z
print(q)

sensor_yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
gt_yaw = float(ground_truth_data_line[5])
print(sensor_yaw)
print(gt_yaw * pi / 180.0)
print(coordinate_transformation.quat_to_heading([q.x, q.y, q.z, q.w]))

sensor_data_file.close()
ground_truth_data_file.close()
