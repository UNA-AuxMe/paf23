#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from mapping.msg import Map as MapMsg


class ObstacleDetection(CompatibleNode):

    def __init__(self):
        """
        This node handles the translation from the static main frame to the
        moving hero frame. The hero frame always moves and rotates as the
        ego vehicle does. The hero frame is used by sensors like the lidar.
        Rviz also uses the hero frame. The main frame is used for planning.
        """
        super().__init__("obstacle_extraction")
        self.loginfo("ActingObstacleExtraction node started")

        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.new_subscription(
            topic=self.get_param("~map_topic", "/paf/hero/mapping/init_data"),
            msg_type=MapMsg,
            callback=self.map_callback,
            qos_profile=1,
        )

    def map_callback(self, data: MapMsg):
        a = 1 + 1


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    roscomp.init("obstacle_extraction", args=args)

    try:
        node = ObstacleDetection()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
