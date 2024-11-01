#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaEgoVehicleControl
from rospy import Publisher, Subscriber
from std_msgs.msg import Bool, Float32


class ManualController(CompatibleNode):
    """
    This node will publish CarlaEgoVehicleControl Commands and forces
    the vehicle_controller to stop doing this.
    """

    def __init__(self):
        super().__init__("manual_controller")
        self.loginfo("Manual Controller node started")
        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)
        self.role_name = self.get_param("role_name", "ego_vehicle")

        # Publisher for Carla Vehicle Control Commands
        self.control_publisher: Publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            f"/carla/{self.role_name}/vehicle_control_cmd",
            qos_profile=10,
        )

        self.switch_control: Subscriber = self.new_subscription(
            Bool,
            f"/paf/{self.role_name}/switch_manual_override",
            self._switch_manual_override,
            qos_profile=1,
        )

        self.steer_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/manual_steer",
            self._manual_steer,
            qos_profile=1,
        )

        self.brake_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/manual_throttle",
            self._manual_throttle,
            qos_profile=1,
        )

        self._override = False  # We are in manual override or not
        self._throttle: float = 0.0
        self._steer: float = 0.0

    def run(self):
        """
        Starts the main loop of the node and send a status msg.
        :return:
        """
        self.loginfo("ManualController node running")

        def loop(timer_event=None) -> None:
            if not self._override:
                return

            message = CarlaEgoVehicleControl()
            message.reverse = self._throttle < 0
            message.hand_brake = False
            message.manual_gear_shift = False
            message.gear = 1
            message.throttle = abs(self._throttle)
            message.brake = False
            message.steer = self._steer
            message.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
            self.control_publisher.publish(message)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def _switch_manual_override(self, msg: Bool):
        self._override = msg.data

    def _manual_throttle(self, msg: Float32):
        self._throttle = msg.data

    def _manual_steer(self, msg: Float32):
        self._steer = msg.data


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("manual_controller", args=args)

    try:
        node = ManualController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
