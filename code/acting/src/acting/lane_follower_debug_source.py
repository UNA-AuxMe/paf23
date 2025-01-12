#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
import rospy
from rospy import Publisher

from nav_msgs.msg import Path
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

from helper_functions import interpolate_route
import math

from typing import Tuple, List

Z_VISUAL = 0


class LaneFollowerSource(CompatibleNode):

    def __init__(self):
        super(LaneFollowerSource, self).__init__("lane_trajectory")
        self.loginfo("Lanefollow node started")

        self.role_name = self.get_param("role_name", "hero")
        topic_name = self.get_param("output_topic_name", "paf/acting/debuglane")
        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)
        # Publisher for Dummy Trajectory
        self.trajectory_pub: Publisher = self.new_publisher(
            msg_type=Path, topic=topic_name, qos_profile=1
        )

        # Publisher for Dummy Velocity
        self.velocity_pub: Publisher = self.new_publisher(
            msg_type=Float32,
            topic=f"/paf/{self.role_name}/target_velocity",
            qos_profile=1,
        )

        # Initialize all needed "global" variables here
        self.time_set = False

        self.create_path()

    def create_path(self):
        """
        Create a sinus path. The path can be used on the devroute.
        The goal for lane following algorithm is to smooth the sinus
        and put it in between lanemarkings
        """
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "global"

        startx = 984.5
        starty = -5442.0 + 50

        route: List[Tuple[float, float]] = []
        for y_offset in range(500):
            x = startx + math.sin(y_offset / 2) * 0.8  # 5 #2 # 40
            y = starty - y_offset
            route.append((x, y))

        for wp in interpolate_route(route, 0.25):
            x, y = wp
            self.add_path_element(path_msg, x, y)
        return path_msg

    def add_path_element(self, path_msg: Path, x: float, y: float):
        pos = PoseStamped()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = "global"
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = Z_VISUAL
        # currently not used therefore zeros
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 0
        path_msg.poses.append(pos)

    def run(self):
        """
        Control loop
        :return:
        """

        def loop(timer_event=None):
            """
            Publishes different speeds, trajectories ...
            depending on the selected TEST_TYPE
            """
            # Drive constant velocity.
            # Create the sinus path.
            path_msg = self.create_path()

            self.trajectory_pub.publish(path_msg)
            self.velocity_pub.publish(rospy.get_param("drive_vel", 0.0))

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


def main(args=None):
    """
    main function
    :param args:
    :return:
    """

    roscomp.init("lanefollow_debug", args=args)
    try:
        node = LaneFollowerSource()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
