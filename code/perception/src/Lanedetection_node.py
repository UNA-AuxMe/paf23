#!/usr/bin/env python

from ros_compatibility.node import CompatibleNode

from rospy.numpy_msg import numpy_msg

# import rospy

import numpy as np
import ros_compatibility as roscomp

import cv2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header

from cv_bridge import CvBridge

# for the lane detection model
import torch
from PIL import Image
import torchvision.transforms as t
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler


class Lanedetection_node(CompatibleNode):
    """YOLOP:
    Model for Lanedetection and Driveable Area Detection

    subscribes to camera image and publishes lane masks and driveable area for further planning
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # load model
        self.model = torch.hub.load("hustvl/yolop", "yolop", pretrained=True)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.bridge = CvBridge()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

        self.role_name = self.get_param("role_name", "hero")
        self.setup_camera_subscriptions("Center")
        self.setup_lane_publisher()
        self.setup_dist_array_subscription()
        self.setup_overlay_publisher()

    def run(self):
        self.spin()
        pass

    def setup_camera_subscriptions(self, side):
        """
        sets up a subscriber to the selected camera angle

        Args:
            side (String): Camera angle specified in launch file
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.image_handler,
            topic=f"/carla/{self.role_name}/{side}/image",
            qos_profile=1,
        )

    def setup_dist_array_subscription(self):
        """
        sets up a subscription to the lidar
        depth image of the selected camera angle
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_dist_array,
            topic="/paf/hero/Center/dist_array",
            qos_profile=1,
        )

    def setup_lane_publisher(self):
        """sets up a publisher for the lane mask
        topic: /Center/lane_mask
        """
        self.lane_mask_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Center/lane_mask",
            qos_profile=1,
        )

    def setup_overlay_publisher(self):
        """sets up a publisher for an graphical output with original image, highlighted lane mask
        and Driveable Area
        topic: /Center/Lane_detect_Overlay
        """
        self.lane_overlay_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Center/Lane_detect_Overlay",
            qos_profile=1,
        )

    def image_handler(self, ImageMsg):
        """
        Callback function for image subscriber
        applies lane detection and Driveable area detection to given ImageMsg
        """
        # free up cuda memory
        if self.device == "cuda":
            torch.cuda.empty_cache()

        image, original_image = self.preprocess_image(ImageMsg)
        original_h, original_w, _ = original_image.shape
        with torch.no_grad():
            det_out, da_seg_out, ll_seg_out = self.detect_lanes(image)

        # convert probabilities to numpy array
        da_seg_out = da_seg_out.sigmoid().squeeze().cpu().numpy()  # (2, 288, 800)
        ll_seg_out = ll_seg_out.sigmoid().squeeze().cpu().numpy()  # (2, 288, 800)

        # scale output to original image
        da_seg_resized = cv2.resize(
            da_seg_out[1, :, :],
            (original_w, original_h),
            interpolation=cv2.INTER_LINEAR,
        )
        ll_seg_resized = cv2.resize(
            ll_seg_out[1, :, :],
            (original_w, original_h),
            interpolation=cv2.INTER_LINEAR,
        )
        """
        # dynamic scaling of the array
        ll_seg_scaled = (
            (ll_seg_resized - np.min(ll_seg_resized))
            / (np.max(ll_seg_resized) - np.min(ll_seg_resized))
            * 255
        ).astype(np.uint8)

        # dynamic scaling of the array
        da_seg_scaled = (
            (da_seg_resized - np.min(da_seg_resized))
            / (np.max(da_seg_resized) - np.min(da_seg_resized))
            * 255
        ).astype(np.uint8)
        """
        # Dynamic threshold for binarization
        lane_threshold = np.mean(ll_seg_resized) + 0.07
        driveable_area_threshold = np.mean(da_seg_resized)

        # Apply dynamic binarization
        ll_seg_binary = (ll_seg_resized > lane_threshold).astype(
            np.uint8
        )  # 1 for lane, 0 otherwise
        da_seg_binary = (da_seg_resized > driveable_area_threshold).astype(
            np.uint8
        )  # 1 for drivable area, 0 otherwise

        # Scale binary mask to match the visualization range [0, 255] for the overlay
        ll_seg_scaled = (ll_seg_binary * 255).astype(np.uint8)
        # da_seg_scaled = (da_seg_binary * 255).astype(np.uint8)

        """# convert original image to numpy
        overlay = np.array(original_image)

        # lane mask = blue
        overlay[:, :, 0] = cv2.addWeighted(overlay[:, :, 0], 1.0, ll_seg_scaled, 1, 0)

        # driveable area = green
        overlay[:, :, 1] = cv2.addWeighted(overlay[:, :, 1], 1.0, da_seg_scaled, 0.5, 0)
        """
        overlay = self.create_top_down_bounding_boxes(ll_seg_scaled, self.dist_arrays)
        ros_lane_mask = self.bridge.cv2_to_imgmsg(ll_seg_scaled)
        ros_lane_overlay = self.bridge.cv2_to_imgmsg(overlay)

        # publish
        self.lane_mask_publisher.publish(ros_lane_mask)
        self.lane_overlay_publisher.publish(ros_lane_overlay)

    def handle_dist_array(self, dist_array):
        """
        This function overwrites the current depth image from
        the lidar distance node with the latest depth image.

        Args:
            dist_array (image msg): Depth image frim Lidar Distance Node
        """
        # callback function for lidar depth image
        # since frequency is lower than image frequency
        # the latest lidar image is saved
        dist_array = self.bridge.imgmsg_to_cv2(
            img_msg=dist_array, desired_encoding="passthrough"
        )
        self.dist_arrays = dist_array

    def preprocess_image(self, image):
        """
        Preprocesses the image to be fed into the model

        Args:
            image (ImgMsg): Image from camera

        Returns:
            np.array: Preprocessed image
        """

        cv_image = self.bridge.imgmsg_to_cv2(img_msg=image, desired_encoding="rgb8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        pil_image = Image.fromarray(cv_image)
        preprocess = t.Compose(
            [
                t.Resize((288, 800)),
                t.ToTensor(),
                t.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ]
        )
        input_image = preprocess(pil_image).unsqueeze(dim=0)
        return input_image, cv_image

    def detect_lanes(self, image):
        """
        Detects lanes in the image

        Args:
            image (np.array): preprocessed Image from camera

        Returns:
            np.array: Lane mask
        """
        det_out, da_seg_out, ll_seg_out = self.model(image)
        return det_out, da_seg_out, ll_seg_out

    def create_top_down_bounding_boxes(self, lane_mask, distance_array):
        lidar_points = self.filter_lidar_by_lane_mask(lane_mask, distance_array)
        clusters = self.cluster_lidar_points(lidar_points)
        image = self.create_bounding_box_top_down(clusters)
        return image

    def filter_lidar_by_lane_mask(self, lane_mask, distance_array):
        """Filter the lidar points by lane mask and remove non-lane points."""
        # Step 1: Create a boolean mask for the points that lie inside the lane mask
        lane_mask_bool = lane_mask.astype(
            bool
        )  # Lane mask as boolean (True for lane, False for outside)

        # Step 2: Create a copy of the distance_array and set all points outside the lane mask to [0.0, 0.0, 0.0]
        filtered_lidar_points = distance_array.copy()  # Create a copy to modify
        filtered_lidar_points[~lane_mask_bool] = [
            0.0,
            0.0,
            0.0,
        ]  # Set points outside lane mask to [0.0, 0.0, 0.0]

        # Step 3: Remove points that are [0.0, 0.0, 0.0]
        valid_lidar_points = filtered_lidar_points[
            np.all(filtered_lidar_points != [0.0, 0.0, 0.0], axis=2)
        ]
        # Step 4: Extract only x and y coordinates for top-down view
        # valid_lidar_points_xy = valid_lidar_points[:, :, :2]  # Only take x and y
        valid_lidar_points = valid_lidar_points[:, :2]
        return valid_lidar_points

    def cluster_lidar_points(self, lidar_points):
        """Cluster lidar points using DBSCAN and create a 2D bounding box."""
        # 1. Extract the 2D coordinates (x, y) from the lidar points

        if lidar_points.shape[0] == 0:
            return []

        # 2. Standardize the coordinates for DBSCAN clustering
        scaler = StandardScaler()
        valid_points_scaled = scaler.fit_transform(lidar_points[:, :2])

        # 3. Apply DBSCAN for clustering
        db = DBSCAN(eps=0.5, min_samples=10).fit(valid_points_scaled)
        labels = db.labels_

        # 4. Find the bounding box around the largest cluster (or all clusters)
        clusters = []
        unique_labels = np.unique(labels)

        for label in unique_labels:
            if label == -1:
                continue  # Ignore noise points

            # Extract points corresponding to the current cluster
            cluster_points = lidar_points[labels == label]

            # Calculate the bounding box (min/max x, y coordinates)
            min_x, min_y = np.min(cluster_points, axis=0)
            max_x, max_y = np.max(cluster_points, axis=0)

            # Store the cluster bounding box
            clusters.append((min_x, min_y, max_x, max_y))
        return clusters

    def create_bounding_box_top_down(self, clusters):
        """Create a top-down view with bounding boxes
        drawn around the detected lane clusters."""
        # Initialize a blank image for the top-down view
        length_image = 30  # meters
        width_image = 5  # meters
        top_down_image = np.zeros((600, 800), dtype=np.uint8)

        # Draw bounding boxes for each cluster
        for min_x, min_y, max_x, max_y in clusters:
            # Convert coordinates to fit the top-down view scale (optional, if needed)
            x_min = int(min_x)
            y_min = int(min_y)
            x_max = int(max_x)
            y_max = int(max_y)
            y_min = int(y_min / width_image * top_down_image.shape[0])
            y_max = int(y_max / width_image * top_down_image.shape[0])
            x_min = int(x_min / length_image * top_down_image.shape[1])
            x_max = int(x_max / length_image * top_down_image.shape[1])
            # Draw the bounding box (white rectangle)
            cv2.rectangle(top_down_image, (x_min, y_min), (x_max, y_max), 255, 5)

        return top_down_image


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = Lanedetection_node("Lanedetection_node")
    node.run()
    print("Lanedetection_node started")