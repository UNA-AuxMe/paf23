#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from mapping.msg import Map as MapMsg
from visualization_msgs.msg import Marker, MarkerArray
from mapping.msg import Entity
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from rospy import Publisher


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
            topic=self.get_param(
                "~map_topic", "/paf/hero/visualization_marker_array_lane"
            ),
            msg_type=MarkerArray,
            callback=self.map_callback,
            qos_profile=1,
        )

        self.entity_publisher: Publisher = self.new_publisher(
            Point, "/test", qos_profile=1
        )

    def map_callback(self, data):
        # self.entity_publisher.publish(data.markers[0].pose.position)

        lane_points_left = []
        lane_points_right = []

        for marker in data.markers:
            # left side of the car is positiv y
            if (marker.pose.position.y) > 0:
                lane_points_left.append(marker.pose.position)
            # right side of the car is negativ y
            else:
                lane_points_right.append(marker.pose.position)

            lane_points_left.sort(key=lambda point: point.x)
            lane_points_right.sort(key=lambda point: point.x)
            lane_points_left.reverse()
            lane_points_right.reverse()

        temp = Point()
        temp.x = -1.0
        temp.y = -1.0
        temp.z = -1.0
        self.entity_publisher.publish(temp)

        self.entity_publisher.publish(lane_points_left[0])
        self.entity_publisher.publish(lane_points_left[-1])
        self.entity_publisher.publish(lane_points_right[0])
        self.entity_publisher.publish(lane_points_right[-1])


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
