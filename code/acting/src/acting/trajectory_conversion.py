#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber


from nav_msgs.msg import Path


from geometry_msgs.msg import PoseStamped, Point
from math import sqrt

from typing import Tuple

from path_conversions import get_closest_forward_index, pose_to_position_distance

from std_msgs.msg import Float32
from scipy.spatial.transform import Rotation
import numpy as np
from numpy.typing import NDArray


class TrajectoryConversion(CompatibleNode):
    """This Node is responsible for converting the global path(trajectory) into
    the cars local frame.

    This way control will act upon this local path
    """

    role_name = "hero"

    def __init__(self):
        self.trajectory_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            callback=self.trajectory_callback,
            qos_profile=1,
        )

        self.trajectory_pub: Publisher = self.new_publisher(
            Path, "/paf/acting/trajectory", qos_profile=1
        )

        self.position_subscription: Subscriber = self.new_subscription(
            PoseStamped,
            "/paf/acting/current_pos",
            self.__set_position,
            qos_profile=1,
        )

        self.heading_subscription: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.__set_orientation,
            qos_profile=1,
        )

        self.__position: Point = None
        self.__orientation: NDArray[np.float_] = None

    def __set_position(self, pose: PoseStamped, min_diff=0.001):
        """
        Updates the current position of the vehicle
        To avoid problems when the car is stationary, new positions will only
        be accepted, if they are a certain distance from the current one
        :param data: new position as PoseStamped
        :param min_diff: minium difference between new and current point for
        the new point to be accepted
        :return:
        """
        if self.__position is None:
            pass
        else:
            dist: float = pose_to_position_distance(pose.pose.position, self.__position)
            if dist < min_diff:
                self.logdebug(
                    "New position disregarded, "
                    f"as dist ({round(dist, 3)}) to current pos "
                    f"< min_diff ({round(min_diff, 3)})"
                )
                return

        self.__position = pose.pose.position

    def __set_orientation(self, heading: Float32):
        heading = heading.data
        direction = np.array([1, 0, 0])
        direction = Rotation.from_euler("z", heading).apply(direction)
        self.__orientation = direction

    def trajectory_callback(self, path: Path):
        if self.__orientation is None or self.__position is None:
            # As long as position is not set we cannot do anything
            return
        closest_index: int = get_closest_forward_index(
            path, self.__position, self.__orientation
        )

        path.poses = path.poses[closest_index:]

        for i in range(0, len(path.poses)):
            path.poses[i].pose.position.z = self.__position.z

        # Add our position as first point in trajectory
        # stamped_pose: PoseStamped = PoseStamped()
        # stamped_pose.header = path.poses[0].header
        # stamped_pose.pose.position = self.__position
        #
        # path.poses.insert(0, stamped_pose)

        self.trajectory_pub.publish(path)


def main(args=None):
    """Start the Node

    Main entry point.
    """

    roscomp.init("trajectory_conversion", args=args)

    try:
        node = TrajectoryConversion()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
