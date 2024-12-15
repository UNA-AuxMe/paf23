#!/usr/bin/env python

import math
import ros_compatibility as roscomp
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from ros_compatibility.node import CompatibleNode
import rospy
from rospy import Publisher, Subscriber
from carla_msgs.msg import CarlaSpeedometer, CarlaEgoVehicleControl

from helper_functions import interpolate_route
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from typing import Tuple, List, Set

from helper_functions import interpolate_route


from sklearn.cluster import DBSCAN
from copy import deepcopy


class LaneFollower(CompatibleNode):

    def __init__(self):
        super(LaneFollower, self).__init__("lane_trajectory")
        self.loginfo("Lanefollow node started")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.role_name = "hero"
        # Publisher for Dummy Trajectory
        self.trajectory_pub: Publisher = self.new_publisher(
            Path, "/paf/" + self.role_name + "/trajectory", qos_profile=1
        )

        # Publisher for Dummy Velocity
        self.velocity_pub: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/target_velocity", qos_profile=1
        )

        # Subscriber of current_pos, used for Steering Debugging
        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1,
        )

        self.lanemarker_sub: Subscriber = self.new_subscription(
            msg_type=MarkerArray,
            callback=self.lanemarker_callback,
            topic="/paf/hero/visualization_marker_array_lane",
            qos_profile=1,
        )

        self.z_visual = 0
        self.x, self.y, self.z = (0, 0, 0)

        # Initialize all needed "global" variables here
        self.driveVel = 1
        self.time_set = False

        self.lane_dir_hist = []

        self.create_path()

    def create_path(self):
        # Generate Empty Trajectory
        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "global"

        startx = 984.5
        starty = -5442.0

        route: List[Tuple[float, float]] = []
        for y_offset in range(500):
            x = startx + math.sin(y_offset / 2) * 0.8  # 5 #2 # 40
            y = starty - y_offset
            route.append((x, y))

        for wp in interpolate_route(route, 0.25):
            x, y = wp
            self.add_path_element(x, y)

    def cluster(self, points: List[Tuple[float, float]]):
        # scan = DBSCAN(eps=3, min_samples=int(len(points) / 3))
        # labels = scan.fit_predict(points)
        lane1: Set[Tuple[float, float]] = set()

        def distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
            return np.linalg.norm(np.array(p1) - np.array(p2))

        lane1.add(points[0])
        threshhold = 1
        while len(lane1) < (1 / 3) * len(points):
            start = len(lane1)
            for point in points:
                for p in lane1:
                    if distance(p, point) < threshhold:
                        lane1.add(point)
                        break

            end = len(lane1)
            if end - start < 1:
                threshhold += 0.1
            else:
                threshhold -= 0.1

        # add remaining points
        for i in range(10):
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

    def lanemarker_callback(self, msg: MarkerArray):
        # self.create_path()  # reset the path
        points: List[Tuple[float, float]] = []
        marker: Marker
        for marker in msg.markers:
            x, y, z = (
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z,
            )
            frame: str = marker.header.frame_id
            # This will be in "hero" frame, but we need x, y, z in global frame for  the trajectory
            x, y, z = self.transform_point(x, y, z, frame, "global")
            points.append((x, y))

            # self.add_path_element(x, y, z)
        labels = self.cluster(points)
        print(labels)

        unique_labels = set(labels)
        if len(unique_labels) < 2:
            print("No Clusters")
            return

        lane1: List[Tuple[float, float]] = []
        lane2: List[Tuple[float, float]] = []
        for i, p in enumerate(points):
            lane1.append(p) if labels[i] else lane2.append(p)
        print(len(lane1), len(lane2), labels)

        min_points_len = min(len(lane1), len(lane2))

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
        print("Mean:", x_mean, "Dist: ", mean_dist)
        tow_l2 = tow_l2 * 1 / np.linalg.norm(tow_l2)  # normalize

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

        tow = tow / np.linalg.norm(tow)
        # print(tow, "tow")
        # if len(self.lane_dir_hist) and np.dot(tow, sum(self.lane_dir_hist)) < 0:
        #    print(len(lane1), "Lane1:", lane1)
        #    print(len(lane2), "Lane2:", lane2)

        tow_l2 = -tow
        print("new_tow", tow_l2)

        switched_lanes = False
        if tow_l2[0] < 0:
            switched_lanes = True

        self.lane_dir_hist.append(tow_l2 if not switched_lanes else -tow_l2)
        if len(self.lane_dir_hist) >= 20:
            self.lane_dir_hist.pop(0)
        lane_dir = sum(self.lane_dir_hist)
        lane_dir = lane_dir / np.linalg.norm(lane_dir)
        # normalize this direction vector

        lane_dir = lane_dir if not switched_lanes else -lane_dir
        print("lane_dir", lane_dir)
        pose: PoseStamped
        for pose in self.path_msg.poses:
            traj_pos = np.array([pose.pose.position.x, pose.pose.position.y])
            if np.linalg.norm(traj_pos - lane1[0]) > 20:
                continue  # skip these points

            dist_mean = 0.0
            """
            for i in range(min_points_len):
                l1 = np.array(lane1[i])
                l2 = np.array(lane2[i])

                d1 = traj_pos - (l1 + lane_dir * mean_dist / 2)  # should be "centered"
                d2 = traj_pos - (l2 - lane_dir * mean_dist / 2)  #

                # lane_dir_rot = np.array([lane_dir[1], -lane_dir[0]])
                # this is now actually in travel direction (or against)

                # m1 = np.dot(d1, lane_dir_rot)
                # m2 = np.dot(d2, lane_dir_rot)

                dist1 = np.dot(lane_dir, d1)  # / np.linalg.norm(d1)

                dist2 = np.dot(lane_dir, d2)  # / np.linalg.norm(d2)
                # projected length onto the lane_dir

                # print(dist1, dist2, d1, d2, l1, l2, traj_pos)

                # print(m1, m2, d1, d2, lane_dir_rot)
                dist_mean += dist1 + dist2

            dist_mean /= min_points_len * 2
            """
            # print("...", len(lane1), len(lane2))
            for lane_pos in lane1 + lane2:
                sign = 1 if lane_pos in lane1 else -1
                d = traj_pos - (lane_pos + (sign * lane_dir * mean_dist / 2))

                # if np.linalg.norm(traj_pos - lane1[0]) > 18:
                # print(d, lane_pos, traj_pos, lane_dir, mean_dist)
                dist = np.dot(lane_dir, d)
                dist_mean += dist

            dist_mean /= len(lane1 + lane2)

            traj_pos = traj_pos - lane_dir * dist_mean
            # print(traj_pos, lane_dir * dist_mean, dist_mean)
            pose.pose.position.x, pose.pose.position.y = traj_pos

            # if pose == self.path_msg.poses[0]:
            #    pose.pose.position.x, pose.pose.position.y = lane1[0]
            # if pose == self.path_msg.poses[1]:
            #    pose.pose.position.x, pose.pose.position.y = lane1[0] + 5 * lane_dir

        self.trajectory_pub.publish(self.path_msg)

        # for i in range(min_points_len):
        #    x, y = lane1[i]
        #    self.move_path(x, y, mean_dist / 2, tow_l2)
        #    x, y = lane2[i]
        #    self.move_path(x, y, mean_dist / 2, -tow_l2)

        #
        # for p in lane1:
        #    x, y = p
        #    self.nudge_path(x, y)

    def move_path(self, x: float, y: float, dist: float, towards):
        """x, y of the lane point, dist is half lane width towards is where mid lane is"""
        pose: PoseStamped
        for pose in self.path_msg.poses:
            traj_pos = np.array([pose.pose.position.x, pose.pose.position.y])
            pos = np.array([x, y])

            distance = np.linalg.norm(traj_pos - pos)

            force = towards * dist / distance

            traj_pos = traj_pos + force
            pose.pose.position.x, pose.pose.position.y = traj_pos

    def nudge_path(self, x: float, y: float):
        """Nudges the path to make it straight in between lines.
        Takes an x, y point and nudges all clos trajectory points away"""
        pose: PoseStamped
        for pose in self.path_msg.poses:
            traj_pos = np.array([pose.pose.position.x, pose.pose.position.y])
            pos = np.array([x, y])
            tow_traj = traj_pos - pos
            distance = np.linalg.norm(tow_traj)
            if (
                distance == 0 or distance > 5
            ):  # we do not divide by 0 and if more than 5 meter points can be ignored
                continue
            normed_toward_trajectory = tow_traj / distance

            force = 0.2 * normed_toward_trajectory / distance

            traj_pos = traj_pos - force

            pose.pose.position.x, pose.pose.position.y = pos

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

    def check_point(self, x: float, y: float, z: float) -> bool:
        """Check if we should accept the point"""
        if not len(self.path_msg.poses):
            return True
        last: PoseStamped = self.path_msg.poses[-1]
        l_x, l_y, l_z = (
            last.pose.position.x,
            last.pose.position.y,
            last.pose.position.z,
        )
        l = np.array([l_x, l_y, l_z])
        p = np.array([x, y, self.z_visual])
        ret = np.linalg.norm(l - p) > 8
        print(l, p, ret)
        return ret

    def add_path_element(self, x: float, y: float):
        # if not self.check_point(x, y, z):
        #    return

        pos = PoseStamped()
        pos.header.stamp = rospy.Time.now()
        pos.header.frame_id = "global"
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = self.z_visual
        # currently not used therefore zeros
        pos.pose.orientation.x = 0
        pos.pose.orientation.y = 0
        pos.pose.orientation.z = 0
        pos.pose.orientation.w = 0
        self.path_msg.poses.append(pos)

    # ALL SUBSCRIBER-FUNCTIONS HERE
    def __current_position_callback(self, data: PoseStamped):
        agent = data.pose.position
        self.x = agent.x
        self.y = agent.y
        self.z = agent.z

    def run(self):
        """
        Control loop
        :return:
        """
        self.checkpoint_time = rospy.get_time()

        def loop(timer_event=None):
            """
            Publishes different speeds, trajectories ...
            depending on the selected TEST_TYPE
            """
            # Drive const. velocity on fixed straight steering

            self.velocity_pub.publish(self.driveVel)

        self.new_timer(0.05, loop)
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
