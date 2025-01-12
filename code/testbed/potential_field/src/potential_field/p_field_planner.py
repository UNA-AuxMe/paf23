#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
import rospy

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


from acting.trajectory_modifier import TrajectoryModifier


from typing import Tuple, List
from numpy.typing import NDArray


class PotentialField(CompatibleNode, TrajectoryModifier):
    multi_path: List[List[Tuple[float, float]]] = []

    def __init__(self):
        CompatibleNode.__init__(self, "potential_field")
        TrajectoryModifier.__init__(self, self)
        self.ctrl_pub = self.new_publisher(
            msg_type=VehicleCtrl, topic="vehicle_ctrl", qos_profile=10
        )
        self.multi_path_pub = self.new_publisher(
            msg_type=MultiPath, topic="multi_path", qos_profile=10
        )

        self.new_subscription(
            msg_type=VehicleInfo,
            topic="info",
            callback=self._info_callback,
            qos_profile=10,
        )
        self.new_subscription(
            msg_type=Entities,
            topic="entities",
            callback=self._entities_callback,
            qos_profile=10,
        )

        self.car_position: Tuple[float, float] = (0, 0)
        self.car_theta: float = 0.0

        self.obstacles: List[Entities] = []

        rate_hz = 20
        self.dt = 1.0 / rate_hz
        self.new_timer(self.dt, lambda _: self.update())

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

    def modify_path(self, positions: NDArray) -> bool:

        for i, position in enumerate(positions):
            x, y = position
            force = self.force_vector(x, y)
            positions[i] -= force

            self.multi_path.append([(x, y), (positions[i][0], positions[i][1])])

        self.multi_path.append([(position[0], position[1]) for position in positions])

        return True


if __name__ == "__main__":
    roscomp.init("pot_field")

    pf = PotentialField()
    pf.spin()
