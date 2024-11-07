#!/usr/bin/env python
from rospy import Publisher, Subscriber

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock
from carla_msgs.msg import CarlaEgoVehicleControl


class CarlaEndpoint(CompatibleNode):
    """
    This node is responsivle for sending out data to Carla.
    This connection is time critical,
    so no other logic should be implemented in this Node.

    It receives vehicle controll commands from paf acting package,
    saves them locally and continously sends out messages to carla.

    This Node adds a Totzeitglied of 1/20Hz into the control logic.
    """

    def __init__(self):
        super().__init__("carla_endpoint")
        self.loginfo("CarlaEndpoint node started")

        self.role_name = self.get_param("role_name", "hero")

        self.control_subscription = self.new_subscription(
            CarlaEgoVehicleControl,
            f"/paf/{self.role_name}/vehicle_control_cmd",
            self.__set_vehicle_control_command,
            qos_profile=10,
        )

        self.carla_status_sub: Subscriber = self.new_subscription(
            Clock,
            "/clock",
            self.__publish_vehicle_control_commands,
            qos_profile=10,
        )

        self.control_publisher: Publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            f"/carla/{self.role_name}/vehicle_control_cmd",
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.status_pub: Publisher = self.new_publisher(
            Bool,
            f"/carla/{self.role_name}/status",
            qos_profile=QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self.__carla_command_out: CarlaEgoVehicleControl = CarlaEgoVehicleControl()
        self.__carla_command_out.reverse = False
        self.__carla_command_out.hand_brake = False
        self.__carla_command_out.manual_gear_shift = False
        self.__carla_command_out.gear = 1
        self.__carla_command_out.throttle = 0.0
        self.__carla_command_out.brake = 0.0
        self.__carla_command_out.steer = 0.0

    def run(self):
        """
        Sends out the status message True for Carla to start simulation
        :return: never returns
        """
        self.status_pub.publish(True)

        self.loginfo("CarlaEndpoint node running")
        self.spin()

    def __publish_vehicle_control_commands(self, msg: Clock) -> None:
        """
        This setup might be confusing a bit at first therefore an explanation:

        We are reacting on the clock provided by carla which seems to be most
        reliable source for ticking in the expected rate.

        However there is a floating point arithmetics bug
        in roscomp.ros_timestamp() method.
        For example 42.00__00626 will be converted to 42.626 instead of 42.0

        Because of this we are doing our own conversion here.
        """
        time_seconds = self.get_time()
        nsecs = round(time_seconds - int(time_seconds), 2)

        self.__carla_command_out.header.stamp.secs = int(time_seconds)
        self.__carla_command_out.header.stamp.nsecs = int(nsecs * 1000000000)
        self.__carla_command_out.header.seq += 1
        self.control_publisher.publish(self.__carla_command_out)

    def __set_vehicle_control_command(self, msg: CarlaEgoVehicleControl) -> None:
        self.__carla_command_out.reverse = msg.reverse
        self.__carla_command_out.hand_brake = msg.hand_brake
        self.__carla_command_out.manual_gear_shift = msg.manual_gear_shift
        self.__carla_command_out.gear = msg.gear
        self.__carla_command_out.throttle = msg.throttle
        self.__carla_command_out.brake = msg.brake
        self.__carla_command_out.steer = msg.steer


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("carla_endpoint", args=args)
    try:
        node = CarlaEndpoint()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
