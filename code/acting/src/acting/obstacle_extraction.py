#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from mapping.msg import Map as MapMsg
from visualization_msgs.msg import Marker, MarkerArray
from mapping.msg import Entity
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point
from rospy import Publisher, Duration

from mapping_common.entity import Entity
from mapping_common.map import Map


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
            callback=self.lane_callback,
            qos_profile=1,
        )

        self.new_subscription(
            topic=self.get_param("~map_topic", "/paf/hero/mapping/init_data"),
            msg_type=MapMsg,
            callback=self.map_callback,
            qos_profile=1,
        )

        self.entity_publisher: Publisher = self.new_publisher(
            MarkerArray, "/acting/entities", qos_profile=1
        )

        self.lane_points_left = []
        self.lane_points_right = []

    def create_marker_from_entity(self, id, entity: Entity) -> Marker:
        marker = entity.to_marker()
        marker.header.frame_id = "hero"
        marker.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
        marker.ns = "m"
        marker.id = id
        marker.lifetime = Duration.from_sec(1.25 / 20.0)

        return marker

    def lane_callback(self, data):
        for marker in data.markers:
            # left side of the car is positiv y
            if (marker.pose.position.y) > 0:
                self.lane_points_left.append(marker.pose.position)
            # right side of the car is negativ y
            else:
                self.lane_points_right.append(marker.pose.position)

            self.lane_points_left.sort(key=lambda point: point.x)
            self.lane_points_right.sort(key=lambda point: point.x)
            self.lane_points_left.reverse()
            self.lane_points_right.reverse()

    def map_callback(self, data: MapMsg):
        # lane_points lists are geometry_msgs/Position
        # get approcimate Y-Coordiante of the lane

        if not (len(self.lane_points_left) == 0 and len(self.lane_points_right) == 0):
            left_lane_pos = (
                self.lane_points_left[0].y + (self.lane_points_left.pop()).y
            ) / 2
            right_lane_pos = (
                self.lane_points_right[0].y + (self.lane_points_right.pop()).y
            ) / 2

            # transform Map into markers
            map = Map.from_ros_msg(data)
            marker_array = MarkerArray()

            for id, entity in enumerate(map.entities):
                current_entity_marker = self.create_marker_from_entity(id, entity)
                # check if this entity is inbetweet the detected lane markings -> entity inside the current lane
                if (
                    current_entity_marker.pose.position.y < left_lane_pos
                    and current_entity_marker.pose.position.y > right_lane_pos
                    and current_entity_marker.pose.position.x > 2.0
                ):
                    # change marker color, to make entities visible
                    current_entity_marker.color.r = 0
                    current_entity_marker.color.g = 150
                    current_entity_marker.color.b = 0
                    marker_array.markers.append(current_entity_marker)
            # publish selected markers
            self.entity_publisher.publish(marker_array)


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
