#!/usr/bin/env python
import ros_compatibility as roscomp
import rospy
from geometry_msgs.msg import PoseStamped
from ros_compatibility.node import CompatibleNode
from nav_msgs.msg import Path
import random


class TestPath(CompatibleNode):

    def __init__(self):
        """
        This node handles the translation from the static main frame to the
        moving hero frame. The hero frame always moves and rotates as the
        ego vehicle does. The hero frame is used by sensors like the lidar.
        Rviz also uses the hero frame. The main frame is used for planning.
        """
        super(TestPath, self).__init__("TestPath")
        self.loginfo("TestPath node started")
        self.local_trajectory = Path()

        self.publisher = self.new_publisher(
            msg_type=Path,
            topic="test_trajectory",
            qos_profile=1,
        )

        self.rate = 0.5
        self.new_timer(0.5, self.generate_trajectory)

    def generate_trajectory(self, timer_event=None) -> Path:
        path_msg = Path()
        path_msg.header = rospy.Header()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "hero"
        path_msg.poses = []
        curve = 0
        factor = 1
        if random.uniform(-1, 1) < 0:
            factor = -1

        for i in range(0, 10):
            pos = PoseStamped()
            pos.header.stamp = rospy.Time.now()
            pos.header.frame_id = "hero"
            curve = curve + 0.01 * i * random.uniform(0, 1)
            pos.pose.position.x = i
            pos.pose.position.y = curve * factor
            pos.pose.position.z = 0
            pos.pose.orientation.w = 1
            path_msg.poses.append(pos)

        self.local_trajectory = path_msg
        self.publisher.publish(path_msg)


if __name__ == "__main__":
    name = "path_test_node"
    roscomp.init(name)
    node = TestPath()
    node.spin()
