#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped

from sim.msg import (
    VehicleCtrl,
    VehicleInfo,
    Entity,
    Entities,
    Vector2D,
    VisPath,
    MultiPath,
)
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist

from typing import Tuple, List
import math
import numpy as np


class DWAPlanner:
    def __init__(self):
        rospy.init_node("dwaplanner")

        self.ctrl_pub = rospy.Publisher("vehicle_ctrl", VehicleCtrl, queue_size=10)
        self.multi_path_pub = rospy.Publisher("multi_path", MultiPath, queue_size=10)

        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback)

        rospy.Subscriber("info", VehicleInfo, self._info_callback)
        rospy.Subscriber("entities", Entities, self._entities_callback)

        rospy.Subscriber("/paf/acting/trajectory", Path, self.__set_path)
        self.trajectory_pub = rospy.Publisher(
            "/paf/local/trajectory", Path, queue_size=10
        )

        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=10
        )

        self.car_position: Tuple[float, float] = (0, 0)
        self.car_theta: float = 0.0

        self.goal_sent = False

        self.obstacles: List[Entities] = []
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rate_hz = 20
        duration = rospy.Duration(1.0 / rate_hz)
        self.dt = duration.to_sec()
        rospy.timer.Timer(duration, lambda _: self.update())

    def update(self):
        car_x, car_y = self.car_position
        self.publish_transform(car_x, car_y, "car_base")

    def _cmd_vel_callback(self, msg: Twist):
        path = Path()
        for i in range(1, 4):
            pose = PoseStamped()
            vx, vy = (msg.linear.x, msg.linear.y)
            car_px, car_py = self.car_position

            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z

            # Transform to global frame (if necessary)
            v_x = linear_x * math.cos(angular_z) - linear_y * math.sin(angular_z)
            v_y = linear_x * math.sin(angular_z) + linear_y * math.cos(angular_z)

            pose.pose.position.x = car_px + v_x * i
            pose.pose.position.y = car_py + v_y * i
            path.poses.append(pose)

        self.trajectory_pub.publish(path)

    def __set_path(self, msg: Path):
        pass

    def _info_callback(self, msg: VehicleInfo):
        self.car_position = (msg.x, msg.y)
        self.car_theta = msg.theta

    def _entities_callback(self, msg: Entities):
        self.obstacles = msg.entities
        if not self.goal_sent and len(msg.entities):
            entity: Entity = msg.entities[0]
            px, py = self.car_to_world(entity.x, entity.y)
            self.publish_goal(px, py)
            self.publish_transform(px, py, "goal")
            self.goal_sent = True

    def car_to_world(self, x_obj_local, y_obj_local):
        # Compute the object's world coordinates
        x_car, y_car = self.car_position
        theta_car = self.car_theta
        x_obj_world = (
            x_car
            + x_obj_local * math.cos(theta_car)
            - y_obj_local * math.sin(theta_car)
        )
        y_obj_world = (
            y_car
            + x_obj_local * math.sin(theta_car)
            + y_obj_local * math.cos(theta_car)
        )

        return x_obj_world, y_obj_world

    def publish_transform(self, x: float, y: float, name: str):
        transform = TransformStamped()

        # Fill in the header information
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"  # Parent frame
        transform.child_frame_id = name  # Child frame

        # Set the translation (x, y, z)
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        # Set the rotation (quaternion)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

    def publish_goal(self, x: float, y: float):
        pose = PoseStamped()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = (0, 0, 0, 1)

        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        self.goal_pub.publish(pose)


if __name__ == "__main__":
    dwa = DWAPlanner()
    rospy.spin()
