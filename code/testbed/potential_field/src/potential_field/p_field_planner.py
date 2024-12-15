#!/usr/bin/env python


import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from sim.msg import (
    VehicleCtrl,
    VehicleInfo,
    Entity,
    Entities,
    Vector2D,
    VisPath,
    MultiPath,
)


from typing import Tuple, List


class PotentialField:
    multi_path: List[List[Tuple[float, float]]] = []

    def __init__(self):
        rospy.init_node("potential_field")
        self.ctrl_pub = rospy.Publisher("vehicle_ctrl", VehicleCtrl, queue_size=10)
        self.multi_path_pub = rospy.Publisher("multi_path", MultiPath, queue_size=10)

        rospy.Subscriber("info", VehicleInfo, self._info_callback)
        rospy.Subscriber("entities", Entities, self._entities_callback)

        rospy.Subscriber("/paf/planning/trajectory", Path, self.__set_path)
        self.trajectory_pub = rospy.Publisher(
            "/paf/acting/trajectory", Path, queue_size=10
        )

        self.car_position: Tuple[float, float] = (0, 0)
        self.car_theta: float = 0.0

        self.obstacles: List[Entities] = []

        rate_hz = 20
        duration = rospy.Duration(1.0 / rate_hz)
        self.dt = duration.to_sec()
        rospy.timer.Timer(duration, lambda _: self.update())

    def force_vector(self, x: float, y: float) -> Tuple[float, float]:
        cumulative = np.zeros((2))
        for obstacle in self.obstacles:
            distance = self._distance(x, y, obstacle)
            direction = self._direction_towards(x, y, obstacle)
            if distance == 0:
                continue
            cumulative += 2 * direction * 1 / distance
        return cumulative

    def update(self):
        car_x, car_y = self.car_position
        force_x, force_y = self.force_vector(0, 0)  # car is at 0,0 for this
        w_f_x, w_f_y = self._car_to_world(force_x, force_y)
        path: List[Tuple[float, float]] = [
            self.car_position,
            (w_f_x, w_f_y),
        ]
        self.multi_path.append(path)
        self.show_multi_path(self.multi_path)
        self.multi_path = []

    def control(self, steer: float, throttle: float, brake: float) -> None:
        msg = VehicleCtrl()
        msg.steering = steer
        msg.throttle = throttle
        msg.brake = brake

    def show_multi_path(self, multi_path: List[List[Tuple[float, float]]]) -> None:
        msg = MultiPath()
        for vp in multi_path:
            vis_path = VisPath()
            for p in vp:
                vis_path.points.append(Vector2D(x=p[0], y=p[1]))

            msg.paths.append(vis_path)
        self.multi_path_pub.publish(msg)

    def _info_callback(self, msg: VehicleInfo):
        self.car_position = (msg.x, msg.y)
        self.car_theta = msg.theta

    def _entities_callback(self, msg: Entities):
        self.obstacles = msg.entities

    def _distance(self, x: float, y: float, ent: Entity):
        pos1 = np.array([x, y])
        pos2 = self._car_to_world(ent.x, ent.y)
        return np.linalg.norm(pos1 - pos2)

    def _direction_towards(self, x: float, y: float, ent: Entity):
        pos1 = np.array([x, y])
        pos2 = self._car_to_world(ent.x, ent.y)
        d: float = np.linalg.norm(pos2 - pos1)
        if d == 0.0:
            return pos2 - pos1
        return (pos2 - pos1) / d

    def _car_to_world(self, x: float, y: float):
        car_x, car_y = self.car_position
        car_phi = self.car_theta
        translation = np.array([car_x, car_y])
        rotation_matrix = np.array(
            [[np.cos(car_phi), -np.sin(car_phi)], [np.sin(car_phi), np.cos(car_phi)]]
        )
        car_position = np.array([x, y])
        world_position = (
            rotation_matrix @ car_position + translation
        )  # Rotate and then translate
        return world_position

    def __set_path(self, msg: Path):
        pose: PoseStamped
        for pose in msg.poses:
            x, y = (pose.pose.position.x, pose.pose.position.y)
            force = self.force_vector(x, y)
            pose.pose.position.x = x - force[0]
            pose.pose.position.y = y - force[1]

            self.multi_path.append(
                [(x, y), (pose.pose.position.x, pose.pose.position.y)]
            )

        self.multi_path.append(
            [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        )

        self.trajectory_pub.publish(msg)


if __name__ == "__main__":

    pf = PotentialField()
    rospy.spin()
