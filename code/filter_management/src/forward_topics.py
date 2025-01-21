#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import NavSatFix, Imu
import threading

"""
This node forwards topics onto different topics
without changing the contents of the respective messages.
"""


class ForwardTopics(CompatibleNode):

    def __init__(self):
        """
        Constructor / Setup
        :return:
        """
        super(ForwardTopics, self).__init__("forward_topics")

        self.loginfo("ForwardTopics node started")

        self.role_name = self.get_param("role_name", "hero")

        # Subscriber

        # Initialize the subscriber for the IMU data
        self.imu_subscriber = self.new_subscription(
            Imu,
            "/carla/" + self.role_name + "/IMU",
            self.forward_imu_data,
            qos_profile=1,
        )
        # Initialize the subscriber for the GPS data
        self.gps_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.forward_gps_data,
            qos_profile=1,
        )

        # Publisher

        # Initialize the publisher for the IMU data
        self.imu_publisher = self.new_publisher(Imu, "/imu/data", qos_profile=1)
        # Initialize the publisher for the GPS data
        self.gps_publisher = self.new_publisher(NavSatFix, "/gps/fix", qos_profile=1)

    def run(self):
        self.loginfo("Forward Topics node running")

        def loop():
            """
            Loop for the Forward Topics node
            """
            while True:
                pass

        threading.Thread(target=loop).start()
        self.spin()

    def forward_imu_data(self, imu_data):
        self.imu_publisher.publish(imu_data)

    def forward_gps_data(self, gps_data):
        self.gps_publisher.publish(gps_data)


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("forward_topics", args=args)

    try:
        node = ForwardTopics()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
