#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
import rospy
from rospy import Subscriber

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from tf2_geometry_msgs import do_transform_point

import numpy as np
from acting.trajectory_modifier import TrajectoryModifier

from typing import Tuple, List, Set
from numpy.typing import NDArray


class LaneFollower(CompatibleNode, TrajectoryModifier):

    def __init__(self):
        CompatibleNode.__init__(self, "lane_follower")
        TrajectoryModifier.__init__(self, node=self)
        self.loginfo("Lanefollow node started")

        self.role_name = self.get_param("role_name", "hero")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # We need the current position, because lanemarks are relative to this
        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1,
        )

        # We need the lanemarkers
        self.lanemarker_sub: Subscriber = self.new_subscription(
            msg_type=MarkerArray,
            callback=self.__lanemarker_callback,
            topic="/paf/hero/visualization_marker_array_lane",
            qos_profile=1,
        )

        # Subscribers
        self.curr_behavior_sub: Subscriber = self.new_subscription(
            String,
            f"/paf/{self.role_name}/curr_behavior",
            self.__set_curr_behavior,
            qos_profile=1,
        )

        # Initialize all needed "global" variables here
        self.lane1: List[Tuple[float, float]] = []
        self.lane2: List[Tuple[float, float]] = []
        self.position: NDArray = np.zeros(2)

        self.current_behaviour = ""

        self.lane_dir_hist = []

    def cluster(self, points: List[Tuple[float, float]]):
        """
        Cluster a list of 2D points into two.
        We assume they were created by lanemarkings and represent 2 distinct clusters,
        which are "lines" in the 2D space which are not colliding.

        We tried several other clustering methods such as DBSCAN.
        But this custom crafted clustering worked best for our specific case.
        """

        def distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
            return np.linalg.norm(np.array(p1) - np.array(p2))

        lane1: Set[Tuple[float, float]] = set()

        lane1.add(points[0])
        threshhold = 1

        # First pass: Dynamically adjust the threshold distance.
        # Add all points to lane1 which are not "too far from any point."
        # Do this until 1/3 of all points are in lane1.
        loop_break = iter(range(100, -1, -1))
        while len(lane1) < (1 / 3) * len(points) and next(loop_break):
            start_size = len(lane1)
            for point in points:
                for p in lane1:
                    if distance(p, point) < threshhold:
                        lane1.add(point)
                        break

            growth = len(lane1) != start_size
            threshhold = threshhold + 0.1 if not growth else threshhold - 0.1

        # Add more points to lane1.
        # Do not increase threshold but rather "explore"
        for _ in range(10):
            threshhold = threshhold * 0.9
            for point in points:
                for p in lane1:
                    if distance(p, point) < threshhold:
                        lane1.add(point)
                        break

        labels: List[int] = []
        for point in points:
            if point in lane1:
                labels.append(0)
            else:
                labels.append(1)
        return labels

    def calculate_l1_to_l2(self):
        """
        Calculate the direction vector from lane1 to lane2.
        --------
                --------
                        --------            lane2
                       ^
                      |
                     |   l1_to_l2 vector
                    |
        --------
                --------
                        --------            lane1
        """
        lane1, lane2 = (self.lane1, self.lane2)

        tow_l2 = np.array([0.0, 0.0])
        x_mean = 0.0

        mean_dist = 0.0
        comps = 0
        for i in range(len(lane1)):
            for j in range(len(lane2)):
                l1 = np.array(lane1[i])
                l2 = np.array(lane2[j])
                tow_l2 = tow_l2 + (l2 - l1)
                x_mean += lane1[i][0]
                x_mean += lane2[j][0]
                mean_dist += abs(lane1[i][0] - lane2[j][0])
                comps += 1
        x_mean = x_mean / (2 * comps)
        mean_dist = mean_dist / (comps)
        print("Mean:", x_mean, "MeanDist: ", mean_dist)
        tow_l2 = tow_l2 * 1 / np.linalg.norm(tow_l2)  # normalize

        return tow_l2, mean_dist

    def calc_tow(self):
        """
        Calculate the direction vector from lane1 to lane2.
        --------
                --------
                        --------            lane2
                       ^
                      |
                     |   l1_to_l2 vector
                    |
        --------
                --------
                        --------            lane1

        Here we do this by finding the closes point in lane2 for each
        point in lane1. The vector is then the mean of the difference of these pairs.
        """
        lane1, lane2 = (self.lane1, self.lane2)

        tow = np.zeros((2))
        for l1_p in lane1:
            l1_np = np.array(l1_p)
            min_dist = 10e99
            min_index = 0
            for i, l2_p in enumerate(lane2):
                l2_np = np.array(l2_p)
                d = np.linalg.norm(l1_np - l2_np)
                if d < min_dist:
                    min_dist = d
                    min_index = i

            tow += l1_np - np.array(lane2[min_index])

        mean_dist = tow / len(lane1)
        tow = tow / np.linalg.norm(tow)

        # print(tow, "tow")
        # if len(self.lane_dir_hist) and np.dot(tow, sum(self.lane_dir_hist)) < 0:
        #    print(len(lane1), "Lane1:", lane1)
        #    print(len(lane2), "Lane2:", lane2)
        return (
            -tow,
            mean_dist,
        )  # in th calculation we do l1 - l2 but we want l2 - l1 (towards l2)

    def running_average_tow(self, tow: NDArray):
        """
        Add an entry to the running average of lanedir,
        which is also the l1_to_l2 vector.
        Returns the average of last 20 entries.
        """

        # First detected if the indices swapped. Meaning its not left to
        # right but right to left. This is important for average.
        if len(self.lane_dir_hist):
            switched_lanes = (
                np.dot(np.mean(np.array(self.lane_dir_hist), axis=0), tow) < 0
            )
        else:
            switched_lanes = False

        self.lane_dir_hist.append(tow if not switched_lanes else -tow)
        if len(self.lane_dir_hist) >= 20:
            self.lane_dir_hist.pop(0)
        lane_dir = sum(self.lane_dir_hist)
        lane_dir = lane_dir / np.linalg.norm(lane_dir)

        lane_dir = lane_dir if not switched_lanes else -lane_dir
        return lane_dir

    def modify_path(self, positions: NDArray) -> bool:
        lane1, lane2 = (self.lane1, self.lane2)

        # For testing only
        rospy.set_param("activated", True)
        self.current_behaviour = "Cruise"

        if (
            self.current_behaviour != "Cruise"  # Only modify if in Cruise
            or len(lane1 + lane2) == 0  # And there are lanes.
            or not rospy.get_param("activated", False)  # And we are activated.
        ):
            return False

        tow, mean_dist = self.calc_tow()
        lane_dir = self.running_average_tow(tow)

        for i, traj_pos in enumerate(positions):
            if np.linalg.norm(traj_pos - self.position) > 20:
                # Only modify point which are 20 meters around us.
                continue

            dist_mean: float = 0.0
            for lane_pos in lane1 + lane2:
                sign = 1 if lane_pos in lane1 else -1
                centered_lane_pos = lane_pos + (sign * lane_dir * mean_dist / 2)

                dist_mean += np.dot(lane_dir, traj_pos - centered_lane_pos)

            dist_mean /= len(lane1 + lane2)

            positions[i] = traj_pos - lane_dir * dist_mean

        return True

    def transform_point(
        self, x: float, y: float, z: float, source_frame: str, target_frame: str
    ) -> Tuple[int, int, int]:
        try:
            # Lookup the transform from 'hero' to 'global'
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # Target frame
                source_frame,  # Source frame
                rospy.Time(0),  # Get the latest available transform
                rospy.Duration(1.0),  # Timeout duration
            )
            point = PointStamped()
            point.point.x, point.point.y, point.point.z = (x, y, z)
            transformed_point: PointStamped = do_transform_point(point, transform)
            return (
                transformed_point.point.x,
                transformed_point.point.y,
                transformed_point.point.z,
            )
        except Exception as e:
            print(e)
            return (x, y, z)  # ?? bad but whatever

    def __current_position_callback(self, data: PoseStamped):
        """
        Subsriber callback to our position.
        """
        agent = data.pose.position
        self.position = np.array([agent.x, agent.y])

    def __lanemarker_callback(self, msg: MarkerArray):
        points: List[Tuple[float, float]] = []
        marker: Marker
        for marker in msg.markers:
            x, y, z = (
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            )
            frame: str = marker.header.frame_id
            # This will be in "hero" frame,
            # but we need x, y, z in global frame for  the trajectory
            x, y, z = self.transform_point(x, y, z, frame, "global")
            points.append((x, y))

        labels = self.cluster(points)
        unique_labels = set(labels)
        if len(unique_labels) < 2:
            self.loginfo(
                "Lanefollower wasn't able to cluster the lanes received."
                + "Ignoring this set of lanes."
            )
            return

        lane1: List[Tuple[float, float]] = []
        lane2: List[Tuple[float, float]] = []
        for i, p in enumerate(points):
            lane1.append(p) if labels[i] else lane2.append(p)
        self.lane1 = lane1
        self.lane2 = lane2

    def __set_curr_behavior(self, msg: String):
        self.current_behaviour = msg.data

    def run(self):
        """
        Run this node.
        We only react on incomming messages.
        No control loop needed.
        """
        self.spin()


def main(args=None):
    """
    main function
    :param args:
    :return:
    """

    roscomp.init("lanefollow", args=args)
    try:
        node = LaneFollower()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()
