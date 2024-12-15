import pygame
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from acting.helper_functions import interpolate_route


class Trajectory:
    startx = 984.5
    starty = -5442.0

    lanechange_trajectory = [
        (startx, starty),
        (startx - 0.5, starty - 10),
        (startx - 0.5, starty - 20),
        (startx - 0.4, starty - 21),
        (startx - 0.3, starty - 22),
        (startx - 0.2, starty - 23),
        (startx - 0.1, starty - 24),
        (startx, starty - 25),
        (startx + 0.1, starty - 26),
        (startx + 0.2, starty - 27),
        (startx + 0.3, starty - 28),
        (startx + 0.4, starty - 29),
        (startx + 0.5, starty - 30),
        (startx + 0.6, starty - 31),
        (startx + 0.7, starty - 32),
        (startx + 0.8, starty - 33),
        (startx + 0.9, starty - 34),
        (startx + 1.0, starty - 35),
        (startx + 1.0, starty - 50),
        (startx + 1.0, starty - 51),
        (startx + 0.9, starty - 52),
        (startx + 0.8, starty - 53),
        (startx + 0.7, starty - 54),
        (startx + 0.6, starty - 55),
        (startx + 0.5, starty - 56),
        (startx + 0.4, starty - 57),
        (startx + 0.3, starty - 58),
        (startx + 0.2, starty - 59),
        (startx + 0.1, starty - 60),
        (startx, starty - 61),
        (startx - 0.1, starty - 62),
        (startx - 0.2, starty - 63),
        (startx - 0.3, starty - 64),
        (startx - 0.4, starty - 65),
        (startx - 0.5, starty - 66),
        (startx - 0.5, starty - 100),
    ]
    ninety_trajectory = [
        (984.5, -5442.0),
        (984.5, -5563.5),
        (985.0, -5573.2),
        (986.3, -5576.5),
        (987.3, -5578.5),
        (988.7, -5579.0),
        (990.5, -5579.8),
        (1000.0, -5580.2),
        (1040.0, -5580.0),
        (1070.0, -5580.0),
    ]

    offset_x: float = -startx
    offset_y: float = -starty

    z_visual: float = 0

    def __init__(self, start_x: float, start_y: float, screen_factor: float):
        self.screen_factor = screen_factor
        current_trajectory = [
            (x + self.offset_x + start_x, y + self.offset_y + start_y)
            for x, y in self.lanechange_trajectory
        ]
        self.current_trajectory = interpolate_route(current_trajectory, 0.25)
        self.trajectory_pub = rospy.Publisher(
            "/paf/planning/trajectory", Path, queue_size=10
        )

    def publish_trajectory(self):
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "global"

        # clear old waypoints
        msg.poses.clear()
        for wp in self.current_trajectory:
            pos = PoseStamped()
            pos.header.stamp = rospy.Time.now()
            pos.header.frame_id = "global"
            pos.pose.position.x = wp[0]
            pos.pose.position.y = wp[1]
            pos.pose.position.z = self.z_visual
            # currently not used therefore zeros
            (
                pos.pose.orientation.x,
                pos.pose.orientation.y,
                pos.pose.orientation.z,
                pos.pose.orientation.w,
            ) = (0, 0, 0, 0)
            msg.poses.append(pos)
        self.trajectory_pub.publish(msg)

    def draw(self, screen):
        # Most important is to swap to y, x for drawing
        screen_trajectory = [
            (y * self.screen_factor, x * self.screen_factor)
            for x, y in self.current_trajectory
        ]
        pygame.draw.lines(screen, (0, 255, 255), False, screen_trajectory, 2)
