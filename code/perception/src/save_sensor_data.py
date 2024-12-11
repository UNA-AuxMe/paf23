#!/usr/bin/env python

"""
This node saves the following data:
- ground truth (heading and position)
- IMU
- Speedometer
- unfiltered_pos
"""
import os
import csv
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import PoseStamped

# from std_msgs.msg import Float32, Header

# from tf.transformations import quaternion_from_euler

from sensor_msgs.msg import Imu
from carla_msgs.msg import CarlaSpeedometer

# from std_msgs.msg import Float32, UInt32

import rospy
import threading
import carla

GPS_RUNNING_AVG_ARGS: int = 10
DATA_SAVING_MAX_TIME: int = 20
FOLDER_PATH: str = "/Position_Heading_Datasets"


class SaveSensorData(CompatibleNode):
    """
    Node publishes a filtered gps signal.
    This is achieved using a rolling average.
    """

    def __init__(self):

        super(SaveSensorData, self).__init__("save_sensor_data")

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "1")

        # carla attributes
        CARLA_HOST = os.environ.get("CARLA_SIM_HOST", "paf-carla-simulator-1")
        CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.world = None
        self.carla_car = None

        # csv file attributes/ flags for plots
        self.sensor_data_csv_created = False
        self.sensor_data_csv_file_path = ""
        self.ground_truth_csv_created = False
        self.ground_truth_csv_file_path = ""

        self.loginfo("Save Sensor Data node started")

        self.time = 0.0

        self.previous_pos = []
        self.pos_to_write = []
        self.previous_imu = []
        self.imu_to_write = []
        self.previous_vel = []
        self.vel_to_write = []
        self.gt_to_write = []

        self.first_line_written = False
        self.stop_saving_data = True

        # Subscriber

        self.position_subscriber = self.new_subscription(
            PoseStamped,
            "/paf/" + self.role_name + "/unfiltered_pos",
            self.save_position,
            qos_profile=1,
        )

        # set up the subscriber for the IMU data
        # (linear acceleration, orientation, angular velocity)
        self.imu_subscriber = self.new_subscription(
            Imu,
            "/carla/" + self.role_name + "/IMU",
            self.save_imu_data,
            qos_profile=1,
        )

        # set up the subscriber for the velocity
        self.velocity_subscriber = self.new_subscription(
            CarlaSpeedometer,
            "/carla/" + self.role_name + "/Speed",
            self.save_velocity,
            qos_profile=1,
        )

    # Subscriber Callbacks

    def save_position(self, unfiltered_pos):
        """
        This method saves the current position in a csv file
        """
        if self.stop_saving_data == True:
            return

        # Specify the path to the folder where you want to save the data
        base_path = "/workspace/code/perception/" "src/experiments/" + FOLDER_PATH
        folder_path = base_path + "/sensor_data"
        # Ensure the directories exist
        os.makedirs(folder_path, exist_ok=True)

        # Create the csv files ONCE if it does not exist
        if self.sensor_data_csv_created is False:
            self.sensor_data_csv_file_path = create_file(folder_path)
            self.sensor_data_csv_created = True

        if self.carla_car is None:
            return
        self.write_csv_pos(unfiltered_pos)

    def write_csv_pos(self, unfiltered_pos):
        with open(self.sensor_data_csv_file_path, "a", newline="") as file:
            writer = csv.writer(file)
            # Check if file is empty and add first row
            if self.first_line_written == False:
                writer.writerow(
                    [
                        "Time",
                        "Sensor",
                        "pos x",
                        "pos y",
                        "pos z",
                        "vel",
                        "orientation x",
                        "orientation y",
                        "orientation z",
                        "orientation w",
                        "ang vel z",
                        "lin acc x",
                        "lin acc y",
                    ]
                )
                self.first_line_written = True
            self.time = rospy.get_time()
            self.pos_to_write = [
                self.time,
                "pos",
                unfiltered_pos.pose.position.x,
                unfiltered_pos.pose.position.y,
                unfiltered_pos.pose.position.z,
                0.0,  # vel
                0.0,  # orientation
                0.0,
                0.0,
                0.0,
                0.0,  # ang vel
                0.0,  # lin acc
                0.0,
            ]
            if self.previous_pos != self.pos_to_write:
                writer.writerow(self.pos_to_write)
                self.previous_pos = self.pos_to_write

            # after each sensor measurement
            # -> save the ground truth
            self.save_ground_truth()

    def save_imu_data(self, imu_data):
        """
        This method saves the imu data in a csv file
        """
        if self.stop_saving_data == True:
            return

        # Specify the path to the folder where you want to save the data
        base_path = "/workspace/code/perception/" "src/experiments/" + FOLDER_PATH
        folder_path = base_path + "/sensor_data"
        # Ensure the directories exist
        os.makedirs(folder_path, exist_ok=True)

        # Create the csv files ONCE if it does not exist
        if self.sensor_data_csv_created is False:
            self.sensor_data_csv_file_path = create_file(folder_path)
            self.sensor_data_csv_created = True

        if self.carla_car is None:
            return
        self.write_csv_imu(imu_data)

    def write_csv_imu(self, imu_data):
        with open(self.sensor_data_csv_file_path, "a", newline="") as file:
            writer = csv.writer(file)
            # Check if file is empty and add first row
            if self.first_line_written == False:
                writer.writerow(
                    [
                        "Time",
                        "Sensor",
                        "pos x",
                        "pos y",
                        "pos z",
                        "vel",
                        "orientation x",
                        "orientation y",
                        "orientation z",
                        "orientation w",
                        "ang vel z",
                        "lin acc x",
                        "lin acc y",
                    ]
                )
                self.first_line_written = True
            self.time = rospy.get_time()
            self.imu_to_write = [
                self.time,
                "imu",
                0.0,  # pos
                0.0,
                0.0,
                0.0,  # vel
                imu_data.orientation.x,
                imu_data.orientation.y,
                imu_data.orientation.z,
                imu_data.orientation.w,
                imu_data.angular_velocity.z,
                imu_data.linear_acceleration.x,
                imu_data.linear_acceleration.y,
            ]
            if self.previous_imu != self.imu_to_write:
                writer.writerow(self.imu_to_write)
                self.previous_imu = self.imu_to_write

            # after each sensor measurement
            # -> save the ground truth
            self.save_ground_truth()

    def save_velocity(self, velocity):
        """
        This method saves the velocity in a csv file
        """
        if self.stop_saving_data == True:
            return

        # Specify the path to the folder where you want to save the data
        base_path = "/workspace/code/perception/" "src/experiments/" + FOLDER_PATH
        folder_path = base_path + "/sensor_data"
        # Ensure the directories exist
        os.makedirs(folder_path, exist_ok=True)

        # Create the csv files ONCE if it does not exist
        if self.sensor_data_csv_created is False:
            self.sensor_data_csv_file_path = create_file(folder_path)
            self.sensor_data_csv_created = True

        if self.carla_car is None:
            return
        self.write_csv_vel(velocity)

    def write_csv_vel(self, velocity):
        with open(self.sensor_data_csv_file_path, "a", newline="") as file:
            writer = csv.writer(file)
            # Check if file is empty and add first row
            if self.first_line_written == False:
                writer.writerow(
                    [
                        "Time",
                        "Sensor",
                        "pos x",
                        "pos y",
                        "pos z",
                        "vel",
                        "orientation x",
                        "orientation y",
                        "orientation z",
                        "orientation w",
                        "ang vel z",
                        "lin acc x",
                        "lin acc y",
                    ]
                )
                self.first_line_written = True
            self.time = rospy.get_time()
            self.vel_to_write = [
                self.time,
                "speed",
                0.0,  # pos
                0.0,
                0.0,
                velocity.speed,
                0.0,  # orientation
                0.0,
                0.0,
                0.0,
                0.0,  # ang vel
                0.0,  # lin acc
                0.0,
            ]
            if self.previous_vel != self.vel_to_write:
                writer.writerow(self.vel_to_write)
                self.previous_vel = self.vel_to_write

            # after each sensor measurement
            # -> save the ground truth
            self.save_ground_truth()

    def save_ground_truth(self):
        """
        This method saves the ground truth in a csv file
        """
        if self.stop_saving_data == True:
            return

        # Specify the path to the folder where you want to save the data
        base_path = "/workspace/code/perception/" "src/experiments/" + FOLDER_PATH
        folder_path = base_path + "/ground_truth"
        # Ensure the directories exist
        os.makedirs(folder_path, exist_ok=True)

        # Create the csv files ONCE if it does not exist
        if self.ground_truth_csv_created is False:
            self.ground_truth_csv_file_path = create_file(folder_path)
            self.ground_truth_csv_created = True

        self.write_csv_gt()

    def write_csv_gt(self):
        with open(self.ground_truth_csv_file_path, "a", newline="") as file:
            writer = csv.writer(file)
            # Check if file is empty and add first row
            if os.stat(self.ground_truth_csv_file_path).st_size == 0:
                writer.writerow(
                    [
                        "Time",
                        "pos x",
                        "pos y",
                        "pos z",
                        "vel",
                        "heading",
                        "ang vel z",
                        "lin acc x",
                        "lin acc y",
                    ]
                )

            carla_pos = self.carla_car.get_location()
            carla_pos.y = -carla_pos.y

            # velocity
            carla_vel_vector = self.carla_car.get_velocity()
            carla_vel = carla_vel_vector.x + carla_vel_vector.y + carla_vel_vector.z

            # orientation
            # carla_quat = quaternion_from_euler(
            # self.carla_car.get_transform().rotation.roll,
            # self.carla_car.get_transform().rotation.pitch,
            # self.carla_car.get_transform().rotation.yaw)
            carla_heading = self.carla_car.get_transform().rotation.yaw

            # angular velocity around z axis
            carla_ang_vel_z = self.carla_car.get_angular_velocity().z

            # linear acceleration
            carla_lin_acc_x = self.carla_car.get_acceleration().x
            carla_lin_acc_y = self.carla_car.get_acceleration().y

            self.gt_to_write = [
                self.time,
                carla_pos.x,
                carla_pos.y,
                carla_pos.z,
                carla_vel,
                carla_heading,
                carla_ang_vel_z,
                carla_lin_acc_x,
                carla_lin_acc_y,
            ]
            writer.writerow(self.gt_to_write)

    # Main method of the node
    def run(self):
        """
        Main loop of the node:
        - updates carla attributes
        - publishes position_debug and heading_debug
        - saves position and heading errors in csv files (if uncommented)
        :return:
        """
        # Retry connecting to the CARLA simulator up to 5 times
        for _ in range(5):
            try:
                self.world = self.client.get_world()
                break
            except RuntimeError:
                self.logwarn("Failed to connect to the CARLA simulator, retrying...")
                rospy.sleep(1)  # Wait for 1 second before retrying
        self.world.wait_for_tick()

        for actor in self.world.get_actors():
            if actor.attributes.get("role_name") == "hero":
                self.carla_car = actor
                break
        if self.carla_car is None:
            self.logwarn("Carla Hero car is none!")
            return

        # Wait for the car to be spawned
        # Otherwise we save invalid data (before car teleports
        # to start position)
        rospy.sleep(5)

        def loop():
            """
            Loop for the data saving
            """
            initialized = False
            start_time = 0
            finish_line_written = False
            while True:
                # save sensor data in csv files
                if not initialized:
                    self.loginfo("Start saving data")
                    start_time = rospy.get_time()
                    initialized = True
                    self.stop_saving_data = False

                if (
                    initialized
                    and (rospy.get_time() - start_time) <= DATA_SAVING_MAX_TIME
                ):
                    self.loginfo(
                        f"{(rospy.get_time() - start_time): .2f}"
                        + "s / "
                        + str(DATA_SAVING_MAX_TIME)
                        + "s"
                    )

                if (
                    initialized
                    and (rospy.get_time() - start_time) > DATA_SAVING_MAX_TIME
                ):
                    if finish_line_written == False:
                        self.loginfo("Finished saving sensor data")
                        finish_line_written = True
                    self.stop_saving_data = True

                rospy.sleep(int(self.control_loop_rate))

        threading.Thread(target=loop).start()
        self.spin()


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
    main function
    :param args:
    :return:
    """

    roscomp.init("save_sensor_data", args=args)
    try:
        node = SaveSensorData()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
