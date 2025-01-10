#!/usr/bin/env python

import numpy as np
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float32, UInt32
from sensor_msgs.msg import Imu
from carla_msgs.msg import CarlaSpeedometer
from rosgraph_msgs.msg import Clock

import rospy
import math
import threading

import os
import csv

from coordinate_transformation import quat_to_heading

GPS_RUNNING_AVG_ARGS = 10

READ_FROM_CSV_FILE: bool = False
READ_FOLDER_PATH: str = (
    "/workspace/code/perception/src/experiments/Position_Heading_Datasets/sensor_data"
)
WRITE_FOLDER_PATH: str = (
    "/workspace/code/perception/src/experiments/filter_output_datasets"
)
FILE_NUM = "00"  # Change this to the number of the sensor_data you want to use

"""
For more information take a look at the documentation:
../../doc/perception/extended_kalman_filter.md

This file implements an Extended Kalman Filter
for the estimation of the position and heading of the vehicle.

The estimation considers the data from the GPS sensor, the Carla Speedometer
and the IMU sensor.
- The GPS sensor provides the position (x/y/z).
- The Carla Speedometer provides the current velocity of the vehicle.
- The IMU sensor provides
  - the linear acceleration (in the x/y/z direction)
  - and the orientation (via a quaternion)
  - as well as the angular velocity (around the x/y/z axis).

Even though the GPS sensor does provide a measurement for the z position of the car,
this coordinate is currently not estimated by the Extended Kalman Filter
and is rather calculated using a running average
of the last GPS_RUNNING_AVG_ARGS measurements of the z position.
"""


class ExtendedKalmanFilter(CompatibleNode):
    """
    This class implements an Extended Kalman Filter (EKF)
    that estimates the heading and position of the car.
    """

    def __init__(self):
        super(ExtendedKalmanFilter, self).__init__("extended_kalman_filter_node")

        self.loginfo("Extended Kalman Filter node started")

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = 0.3
        self.publish_seq = UInt32(0)
        self.frame_id = "map"

        self.now = 0
        self.previous_time = 0
        self.current_time = 0
        self.dt = self.control_loop_rate

        # the state vector is of the following form:
        """
        [
            [0]: x position,
            [1]: y position,
            [2]: velocity,
            [3]: acceleration,
            [4]: heading,
            [5]: angular velocity around z axis
        ]
        """
        self.state_vector_pred = np.zeros((6, 1))
        self.state_vector_corr = np.zeros((6, 1))

        # the covariance matrix of the state:
        self.P_pred = np.zeros((6, 6))
        self.P_corr = np.zeros((6, 6))

        # the measurements are saved into the follwing variables:
        self.heading_m = 0
        self.ang_vel_m = 0
        self.acc_x_m = 0
        self.acc_y_m = 0
        self.x_pos_m = 0
        self.y_pos_m = 0
        self.z_pos_m = 0
        self.vel_m = 0

        # only if all sensors provided data at least one time
        # the state vector can be initialized (in function "run")
        self.pos_initialized = False
        self.vel_initialized = False
        self.acc_initialized = False
        self.heading_initialized = False
        self.ang_vel_initialized = False

        # the matrices needed for the "prediction"
        # and "correction" function are as follows:

        # the process covariance matrix Q
        self.Q = np.diag([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])

        # the Kalman gain (-> gets calculated in function "correction")
        self.K = np.zeros((6, 6))

        # the measurement covariance matrix R
        self.R = np.diag([0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001])

        # for rolling average (of z position)
        self.avg_z = np.zeros((GPS_RUNNING_AVG_ARGS, 1))

        if READ_FROM_CSV_FILE is False:

            # SUBSCRIBER

            # set up the subscriber for the position
            # -> unfiltered_pos publishes GPS data (lat/lon/alt) in x/y/z coordinates
            self.position_subscriber = self.new_subscription(
                PoseStamped,
                "/paf/" + self.role_name + "/unfiltered_pos",
                self.update_position,
                qos_profile=1,
            )

            # set up the subscriber for the IMU data
            # (linear acceleration, orientation, angular velocity)
            self.imu_subscriber = self.new_subscription(
                Imu,
                "/carla/" + self.role_name + "/IMU",
                self.update_imu_data,
                qos_profile=1,
            )

            # set up the subscriber for the velocity
            self.velocity_subscriber = self.new_subscription(
                CarlaSpeedometer,
                "/carla/" + self.role_name + "/Speed",
                self.update_velocity,
                qos_profile=1,
            )

            self.clock_subscriber = self.new_subscription(
                Clock,
                "/clock/",
                self.get_time,
                qos_profile=1,
            )

            # PUBLISHER

            # set up the publisher for the position
            self.ekf_position_publisher = self.new_publisher(
                PoseStamped,
                "/paf/" + self.role_name + "/extended_kalman_pos",
                qos_profile=1,
            )

            # set up the publisher for the heading
            self.ekf_heading_publisher = self.new_publisher(
                Float32,
                "/paf/" + self.role_name + "/extended_kalman_heading",
                qos_profile=1,
            )

        else:
            self.simulated_time = 0

            self.start_time = 0
            self.start_time_set = False
            self.filter_ready = False
            self.input_line = []

            self.stop_saving_data = False
            self.filter_output_csv_created = False
            self.filter_output_csv_file_path = ""
            self.previous_filter_output = []
            self.filter_output_to_write = []

            self.read_file = open(
                str(READ_FOLDER_PATH) + "/data_" + str(FILE_NUM) + ".csv", "r"
            )
            self.read_file.readline()  # first line not needed

            self.clock_subscriber = self.new_subscription(
                Clock,
                "/clock/",
                self.time_check,
                qos_profile=1,
            )

    def run(self):
        """
        Initialize and run the Extended Kalman Filter
        (run = cycle of prediction and correction -> loop function)
        """
        # wait until the car received data from all necessary sensors
        # -> IMU, GPS and Speedometer
        while not (
            self.pos_initialized
            and self.vel_initialized
            and self.acc_initialized
            and self.heading_initialized
            and self.ang_vel_initialized
        ):
            self.loginfo("Extended Kalman Filter is waiting for initialization")
            if (READ_FROM_CSV_FILE is True) and (self.filter_ready is False):
                self.filter_ready = True
            rospy.sleep(1)
        rospy.sleep(1)

        # initialize the state vector and the covariance matrix
        # -> state_vector_corr and P_corr

        a = math.sqrt(self.acc_x_m**2 + self.acc_y_m**2)
        v = self.vel_m + a * self.dt
        heading = self.heading_m + self.ang_vel_m * self.dt
        heading_in_rad = heading * math.pi / 180

        self.state_vector_corr = np.array(
            [
                self.x_pos_m
                + v * math.cos(heading_in_rad) * self.dt
                + 0.5 * a * math.cos(heading_in_rad) * (self.dt**2),
                self.y_pos_m
                + v * math.sin(heading_in_rad) * self.dt
                + 0.5 * a * math.sin(heading_in_rad) * (self.dt**2),
                v,
                a,
                heading,
                self.ang_vel_m,
            ]
        )

        self.P_corr = np.eye(6)

        self.loginfo("Extended Kalman Filter is initialized")

        def loop():
            """
            Cycle of prediction and correction
            after adjusting the estimation
            according to the measurement (correction): data published
            """
            self.loginfo("Extended Kalman Filter started its loop")
            while True:
                self.current_time = self.now
                self.dt = self.current_time - self.previous_time
                self.prediction()
                self.correction()

                if READ_FROM_CSV_FILE is False:
                    # Publish the ekf-data:
                    self.publish_heading()
                    self.publish_position()
                else:
                    # Write the ekf-data into a csv file
                    self.save_filter_output()

                self.previous_time = self.current_time
                rospy.sleep(self.control_loop_rate)

        threading.Thread(target=loop).start()
        self.spin()

    def prediction(self):
        """
        Predict the next state and covariance matrix
        -> saved in state_vector_pred and P_pred
        """
        self.calculate_matrix_A()

        v = self.state_vector_corr[2] + self.state_vector_corr[3] * self.dt
        heading = self.state_vector_corr[4] + self.state_vector_corr[5] * self.dt
        heading_in_rad = heading * math.pi / 180

        self.state_vector_pred = np.array(
            [
                self.state_vector_corr[0]
                + v * math.cos(heading_in_rad) * self.dt
                + 0.5
                * self.state_vector_corr[3]
                * math.cos(heading_in_rad)
                * (self.dt**2),
                self.state_vector_corr[1]
                + v * math.sin(heading_in_rad) * self.dt
                + 0.5
                * self.state_vector_corr[3]
                * math.sin(heading_in_rad)
                * (self.dt**2),
                v,
                self.state_vector_corr[3],
                heading,
                self.state_vector_corr[5],
            ]
        )

        self.P_pred = self.A @ self.P_corr @ self.A.T + self.Q

    def calculate_matrix_A(self):
        """
        Matrix A needs to be linearized in the point state_vector_corr
        """
        heading_in_rad = self.state_vector_corr[4] * math.pi / 180

        self.A = np.eye(6)

        self.A[0, 2] = np.cos(heading_in_rad) * self.dt
        self.A[0, 3] = 0.5 * np.cos(heading_in_rad) * (self.dt**2)
        self.A[0, 4] = -self.state_vector_corr[2] * self.dt * np.sin(
            heading_in_rad
        ) - 0.5 * self.state_vector_corr[3] * (self.dt**2) * np.sin(heading_in_rad)

        self.A[1, 2] = np.sin(heading_in_rad) * self.dt
        self.A[1, 3] = 0.5 * np.sin(heading_in_rad) * (self.dt**2)
        self.A[0, 4] = self.state_vector_corr[2] * self.dt * np.cos(
            heading_in_rad
        ) + 0.5 * self.state_vector_corr[3] * (self.dt**2) * np.cos(heading_in_rad)

        self.A[2, 3] = self.dt

        self.A[4, 5] = self.dt

    def correction(self):
        """
        Correct the state vector according to the measurement
        """
        C = np.eye(6)

        # calculate the Kalman gain
        S = self.R + C @ self.P_pred @ C.T  # helper matrix
        self.K = self.P_pred @ C.T @ np.linalg.inv(S)

        # calculate the corrected state vector
        y = np.array(  # measurement vector
            [
                self.x_pos_m,
                self.y_pos_m,
                self.vel_m,
                np.sqrt(self.acc_x_m**2 + self.acc_y_m**2),
                self.heading_m,
                self.ang_vel_m,
            ]
        )
        r = y - self.state_vector_pred  # measurement residual
        self.state_vector_corr = self.state_vector_pred + self.K @ r

        # calculate the corrected covariance matrix
        Identity = np.eye(6)
        self.P_corr = (Identity - self.K @ C) @ self.P_pred @ (
            Identity - self.K @ C
        ).T + self.K @ self.R @ self.K.T

    def publish_heading(self):
        """
        Publish the extended kalman heading
        """
        ekf_heading = Float32()
        ekf_heading.data = self.state_vector_corr[4]
        print("heading: " + str(ekf_heading.data))
        # print("heading type: " + str(type(ekf_heading)))
        self.ekf_heading_publisher.publish(ekf_heading)

    def publish_position(self):
        """
        Publish the extended kalman location
        """
        ekf_position = PoseStamped()

        ekf_position.header.frame_id = self.frame_id
        ekf_position.header.stamp = rospy.Time.now()
        ekf_position.header.seq = self.publish_seq

        self.publish_seq.data += 1

        ekf_position.pose.position.x = self.state_vector_corr[0]
        ekf_position.pose.position.y = self.state_vector_corr[1]

        ekf_position.pose.position.z = self.z_pos_m
        """print(
            "type(ekf_position.pose.position.z): "
            + str(type(ekf_position.pose.position.z))
        )
        print("type(self.z_pos_m): " + str(type(self.z_pos_m)))"""

        ekf_position.pose.orientation.x = 0
        ekf_position.pose.orientation.y = 0
        ekf_position.pose.orientation.z = 1
        ekf_position.pose.orientation.w = 0

        print(ekf_position.pose)
        # print("position type: " + str(type(ekf_position)))
        self.ekf_position_publisher.publish(ekf_position)

    def update_imu_data(self, imu_data):
        """
        Write the measured IMU data:
        - heading (calculated using quaternions) into heading_m
        - angular velocity (around z axis) into acc_vel_m
        - acceleration in x direction into acc_x_m
        - acceleration in y direction into acc_y_m
        """
        orientation_x = imu_data.orientation.x
        orientation_y = imu_data.orientation.y
        orientation_z = imu_data.orientation.z
        orientation_w = imu_data.orientation.w

        # Calculate the heading based on the orientation given by the IMU
        data_orientation_q = [
            orientation_x,
            orientation_y,
            orientation_z,
            orientation_w,
        ]

        heading = quat_to_heading(data_orientation_q)

        self.heading_m = heading
        self.ang_vel_m = imu_data.angular_velocity.z
        self.acc_x_m = imu_data.linear_acceleration.x
        self.acc_y_m = imu_data.linear_acceleration.y

        if not self.heading_initialized:
            self.heading_initialized = True
        if not self.ang_vel_initialized:
            self.ang_vel_initialized = True
        if not self.acc_initialized:
            self.acc_initialized = True

    def update_position(self, unfiltered_pos):
        """
        Write the measured position into x_pos_m and y_pos_m
        The z position is calculated using a rolling average -> saved in z_pos_m
        -> if first measurement received: the position is initialized
        """
        self.x_pos_m = unfiltered_pos.pose.position.x
        self.y_pos_m = unfiltered_pos.pose.position.y

        z = unfiltered_pos.pose.position.z

        self.avg_z = np.roll(self.avg_z, -1, axis=0)
        self.avg_z[-1] = np.matrix([z])
        avg_z = np.mean(self.avg_z, axis=0)

        self.z_pos_m = avg_z

        if not self.pos_initialized:
            self.pos_initialized = True

    def update_velocity(self, velocity):
        """
        Write the measured velocity into vel_m
        -> if first measurement received: the velocity is initialized
        """
        self.vel_m = velocity.speed

        if not self.vel_initialized:
            self.vel_initialized = True

    def time_check(self, time):
        if self.stop_saving_data is True:
            return

        sec = time.clock.secs
        nsec = time.clock.nsecs
        nsec /= 1000000000

        self.now = sec + nsec
        if self.previous_time == 0:
            self.previous_time = self.now

        now = sec + nsec
        if (self.filter_ready is True) and (self.start_time_set is False):
            self.start_time = now
            self.start_time_set = True
            self.input_line = self.read_file.readline().split(",")
            self.input_line[-1] = self.input_line[-1].strip()
            self.data_start_time = float(self.input_line[0])

        if (self.filter_ready is True) and (self.start_time_set is True):
            self.simulated_time = now - self.start_time + self.data_start_time
            while self.simulated_time >= float(self.input_line[0]):
                if self.input_line[1] == "pos":
                    position = PoseStamped()
                    # position.header = Header()
                    # position.header.stamp = self.simulated_time

                    # Fill in the pose
                    position.pose.position.x = float(self.input_line[2])
                    position.pose.position.y = float(self.input_line[3])
                    position.pose.position.z = float(self.input_line[4])

                    # Assuming you have no orientation information
                    position.pose.orientation.x = 0
                    position.pose.orientation.y = 0
                    position.pose.orientation.z = 0
                    position.pose.orientation.w = 1

                    self.update_position(position)
                    print(
                        "updated position at simulated time " + str(self.simulated_time)
                    )

                if self.input_line[1] == "imu":
                    imu_message = Imu()
                    # imu_message.header = Header()
                    # imu_message.header.stamp = self.simulated_time

                    imu_message.orientation.x = float(self.input_line[6])
                    imu_message.orientation.y = float(self.input_line[7])
                    imu_message.orientation.z = float(self.input_line[8])
                    imu_message.orientation.w = float(self.input_line[9])

                    imu_message.angular_velocity.z = float(self.input_line[10])

                    imu_message.linear_acceleration.x = float(self.input_line[11])
                    imu_message.linear_acceleration.y = float(self.input_line[12])

                    self.update_imu_data(imu_message)
                    print(
                        "updated imu data at simulated time " + str(self.simulated_time)
                    )

                if self.input_line[1] == "speed":
                    velocity = CarlaSpeedometer()

                    velocity.speed = float(self.input_line[5])

                    self.update_velocity(velocity)
                    print(
                        "updated velocity at simulated time " + str(self.simulated_time)
                    )

                self.input_line = self.read_file.readline()
                if len(self.input_line) == 0:
                    self.stop_saving_data = True
                    self.loginfo("Extended Kalman Filter stopped saving filter output")
                    return
                else:
                    self.input_line = self.input_line.split(",")
                    self.input_line[-1] = self.input_line[-1].strip()

    def get_time(self, time):
        sec = time.clock.secs
        nsec = time.clock.nsecs
        nsec /= 1000000000
        self.now = sec + nsec
        if self.previous_time == 0:
            self.previous_time = self.now

    def save_filter_output(self):
        if self.stop_saving_data is True:
            return

        os.makedirs(WRITE_FOLDER_PATH, exist_ok=True)

        # Create the csv files ONCE if it does not exist
        if self.filter_output_csv_created is False:
            self.filter_output_csv_file_path = create_file(WRITE_FOLDER_PATH)
            self.filter_output_csv_created = True

        self.write_csv_filter_output()

    def write_csv_filter_output(self):
        with open(self.filter_output_csv_file_path, "a", newline="") as file:
            writer = csv.writer(file)
            # Check if file is empty and add first row
            if os.stat(self.filter_output_csv_file_path).st_size == 0:
                writer.writerow(
                    [
                        "Time",
                        "pos x",
                        "pos y",
                        "pos z",
                        "heading",
                    ]
                )

            time = self.simulated_time

            ekf_position_x = self.state_vector_corr[0]
            ekf_position_y = self.state_vector_corr[1]
            ekf_position_z = float(self.z_pos_m)

            ekf_heading = self.state_vector_corr[4]

            self.filter_output_to_write = [
                time,
                ekf_position_x,  # pos
                ekf_position_y,
                ekf_position_z,
                ekf_heading,  # heading
            ]
            if self.previous_filter_output != self.filter_output_to_write:
                writer.writerow(self.filter_output_to_write)
                self.previous_filter_output = self.filter_output_to_write


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


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("extended_kalman_filter_node", args=args)

    try:
        node = ExtendedKalmanFilter()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
