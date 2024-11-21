#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import torch
from torchvision.models.segmentation import (
    DeepLabV3_ResNet101_Weights,
    deeplabv3_resnet101,
)
from torchvision.models.detection.faster_rcnn import (
    FasterRCNN_MobileNet_V3_Large_320_FPN_Weights,
    FasterRCNN_ResNet50_FPN_V2_Weights,
    fasterrcnn_resnet50_fpn_v2,
    fasterrcnn_mobilenet_v3_large_320_fpn,
)
import torchvision.transforms as t
import cv2
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header, Float32MultiArray
from cv_bridge import CvBridge
from torchvision.utils import draw_bounding_boxes, draw_segmentation_masks
import numpy as np
from ultralytics import NAS, YOLO, RTDETR, SAM, FastSAM
import asyncio
import rospy
from visualization_msgs.msg import Marker, MarkerArray

# Carla-Farben
carla_colors = [
    [0, 0, 0],  # 0: None
    [70, 70, 70],  # 1: Buildings
    [190, 153, 153],  # 2: Fences
    [72, 0, 90],  # 3: Other
    [220, 20, 60],  # 4: Pedestrians
    [153, 153, 153],  # 5: Poles
    [157, 234, 50],  # 6: RoadLines
    [128, 64, 128],  # 7: Roads
    [244, 35, 232],  # 8: Sidewalks
    [107, 142, 35],  # 9: Vegetation
    [0, 0, 255],  # 10: Vehicles
    [102, 102, 156],  # 11: Walls
    [220, 220, 0],  # 12: TrafficSigns
]

# COCO-Klassen → Carla-Klassen Mapping
coco_to_carla = [
    0,  # 0: N/A -> None
    4,  # 1: Person -> Pedestrians
    10,  # 2: Bicycle -> Vehicles
    10,  # 3: Car -> Vehicles
    10,  # 4: Motorbike -> Vehicles
    1,  # 5: Airplane -> Buildings
    10,  # 6: Bus -> Vehicles
    1,  # 7: Train -> Buildings
    10,  # 8: Truck -> Vehicles
    1,  # 9: Boat -> Buildings
    12,  # 10: Traffic Light -> TrafficSigns
    11,  # 11: Fire Hydrant -> Walls
    0,  # 12: N/A -> None
    0,  # 13: N/A -> None
    0,  # 14: N/A -> None
    5,  # 15: Parking Meter -> Poles
    3,  # 16: Bench -> Other
    9,  # 17: Bird -> Vegetation
    9,  # 18: Cat -> Vegetation
    9,  # 19: Dog -> Vegetation
    9,  # 20: Horse -> Vegetation
    9,  # 21: Sheep -> Vegetation
    9,  # 22: Cow -> Vegetation
    9,  # 23: Elephant -> Vegetation
    9,  # 24: Bear -> Vegetation
    9,  # 25: Zebra -> Vegetation
    9,  # 26: Giraffe -> Vegetation
    0,  # 27: N/A -> None
    3,  # 28: Backpack -> Other
    3,  # 29: Umbrella -> Other
    3,  # 30: Handbag -> Other
    3,  # 31: Tie -> Other
    3,  # 32: Suitcase -> Other
    3,  # 33: Frisbee -> Other
    3,  # 34: Skis -> Other
    3,  # 35: Snowboard -> Other
    3,  # 36: Sports Ball -> Other
    3,  # 37: Kite -> Other
    3,  # 38: Baseball Bat -> Other
    3,  # 39: Baseball Glove -> Other
    3,  # 40: Skateboard -> Other
    3,  # 41: Surfboard -> Other
    3,  # 42: Tennis Racket -> Other
    3,  # 43: Bottle -> Other
    3,  # 44: Wine Glass -> Other
    3,  # 45: Cup -> Other
    3,  # 46: Fork -> Other
    3,  # 47: Knife -> Other
    3,  # 48: Spoon -> Other
    3,  # 49: Bowl -> Other
    9,  # 50: Banana -> Vegetation
    9,  # 51: Apple -> Vegetation
    3,  # 52: Sandwich -> Other
    9,  # 53: Orange -> Vegetation
    9,  # 54: Broccoli -> Vegetation
    9,  # 55: Carrot -> Vegetation
    3,  # 56: Hot Dog -> Other
    3,  # 57: Pizza -> Other
    3,  # 58: Donut -> Other
    3,  # 59: Cake -> Other
    3,  # 60: Chair -> Other
    1,  # 61: Couch -> Buildings
    1,  # 62: Potted Plant -> Vegetation
    1,  # 63: Bed -> Buildings
    1,  # 64: Dining Table -> Buildings
    1,  # 65: Toilet -> Buildings
    0,  # 66: N/A -> None
    0,  # 67: N/A -> None
    0,  # 68: N/A -> None
    0,  # 69: N/A -> None
    1,  # 70: TV -> Buildings
    3,  # 71: Laptop -> Other
    3,  # 72: Mouse -> Other
    3,  # 73: Remote -> Other
    3,  # 74: Keyboard -> Other
    3,  # 75: Cell Phone -> Other
    1,  # 76: Microwave -> Buildings
    1,  # 77: Oven -> Buildings
    1,  # 78: Toaster -> Buildings
    1,  # 79: Sink -> Buildings
]


def get_carla_color(coco_class):
    carla_class = coco_to_carla[coco_class]
    return carla_colors[carla_class]


class VisionNode(CompatibleNode):
    """
    VisionNode:

    The Vision-Node provides advanced object-detection features.
    It can handle different camera angles, easily switch between
    pretrained models and distances of objects.

    Advanced features are limited to ultralytics models and center view.
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # dictionary of pretrained models
        self.model_dict = {
            "frcnn_resnet50_fpn_v2": (
                fasterrcnn_resnet50_fpn_v2(
                    weights=FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
                ),
                FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT,
                "detection",
                "pyTorch",
            ),
            "frcnn_mobilenet_v3_large_320_fpn": (
                fasterrcnn_mobilenet_v3_large_320_fpn(
                    weights=FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT
                ),
                FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT,
                "detection",
                "pyTorch",
            ),
            "deeplabv3_resnet101": (
                deeplabv3_resnet101(weights=DeepLabV3_ResNet101_Weights.DEFAULT),
                DeepLabV3_ResNet101_Weights.DEFAULT,
                "segmentation",
                "pyTorch",
            ),
            "yolov8n": (YOLO, "yolov8n.pt", "detection", "ultralytics"),
            "yolov8s": (YOLO, "yolov8s.pt", "detection", "ultralytics"),
            "yolov8m": (YOLO, "yolov8m.pt", "detection", "ultralytics"),
            "yolov8l": (YOLO, "yolov8l.pt", "detection", "ultralytics"),
            "yolov8x": (YOLO, "yolov8x.pt", "detection", "ultralytics"),
            "yolo_nas_l": (NAS, "yolo_nas_l.pt", "detection", "ultralytics"),
            "yolo_nas_m": (NAS, "yolo_nas_m.pt", "detection", "ultralytics"),
            "yolo_nas_s": (NAS, "yolo_nas_s.pt", "detection", "ultralytics"),
            "rtdetr-l": (RTDETR, "rtdetr-l.pt", "detection", "ultralytics"),
            "rtdetr-x": (RTDETR, "rtdetr-x.pt", "detection", "ultralytics"),
            "yolov8x-seg": (YOLO, "yolov8x-seg.pt", "segmentation", "ultralytics"),
            "yolo11n-seg": (YOLO, "yolo11n-seg.pt", "segmentation", "ultralytics"),
            "sam_l": (SAM, "sam_l.pt", "detection", "ultralytics"),
            "FastSAM-x": (FastSAM, "FastSAM-x.pt", "detection", "ultralytics"),
        }

        # general setup
        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        self.center = self.get_param("center")
        self.back = self.get_param("back")
        self.left = self.get_param("left")
        self.right = self.get_param("right")

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.depth_images = []
        self.dist_arrays = None

        # publish / subscribe setup
        if self.center:
            self.setup_camera_subscriptions("Center")
        if self.back:
            self.setup_camera_subscriptions("Back")
        if self.left:
            self.setup_camera_subscriptions("Left")
        if self.right:
            self.setup_camera_subscriptions("Right")

        # self.setup_rainbow_subscription()
        self.setup_dist_array_subscription()
        self.setup_camera_publishers()
        self.setup_object_distance_publishers()
        self.setup_traffic_light_publishers()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

        # model setup
        model_info = self.model_dict[self.get_param("model")]
        self.model = model_info[0]
        self.weights = model_info[1]
        self.type = model_info[2]
        self.framework = model_info[3]
        self.save = True

        # print configuration of Vision-Node
        print("Vision Node Configuration:")
        print("Device -> ", self.device)
        print(f"Model -> {self.get_param('model')},")
        print(f"Type -> {self.type}, Framework -> {self.framework}")
        torch.cuda.memory.set_per_process_memory_fraction(0.1)

        # pyTorch and CUDA setup
        if self.framework == "pyTorch":
            for param in self.model.parameters():
                param.requires_grad = False
                self.model.to(self.device)

        # ultralytics setup
        if self.framework == "ultralytics":
            self.model = self.model(self.weights)

        self.marker_publisher = self.new_publisher(
            msg_type=MarkerArray,
            topic=f"/paf/{self.role_name}/visualization_marker_array",
            qos_profile=1,
        )

    def setup_camera_subscriptions(self, side):
        """
        sets up a subscriber to the selected camera angle

        Args:
            side (String): Camera angle specified in launch file
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
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

    def setup_camera_publishers(self):
        """
        sets up a publisher to the selected camera angle
        multiple publishers are used since the vision node could handle
        multiple camera angles at the same time if enough
        resources are available
        """

        if self.center:
            self.publisher_center = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Center/segmented_image",
                qos_profile=1,
            )
        if self.back:
            self.publisher_back = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Back/segmented_image",
                qos_profile=1,
            )
        if self.left:
            self.publisher_left = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Left/segmented_image",
                qos_profile=1,
            )
        if self.right:
            self.publisher_right = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Right/segmented_image",
                qos_profile=1,
            )

    def setup_object_distance_publishers(self):
        """
        sets up a publisher to publish a list of objects
        and their distances
        """

        self.distance_publisher = self.new_publisher(
            msg_type=Float32MultiArray,
            topic=f"/paf/{self.role_name}/{self.side}/object_distance",
            qos_profile=1,
        )

    def setup_traffic_light_publishers(self):
        """
        sets up a publisher for traffic light detection
        """

        self.traffic_light_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/{self.side}/segmented_traffic_light",
            qos_profile=1,
        )

    def handle_camera_image(self, image):
        """
        This function handles a new camera image and publishes the
        calculated visualization according to the correct camera angle

        Args:
            image (image msg): Image from camera scubscription, should be rgb
        """

        # free up cuda memory
        if self.device == "cuda":
            torch.cuda.empty_cache()

        if self.framework == "pyTorch":
            vision_result = self.predict_torch(image)

        if self.framework == "ultralytics":
            vision_result = self.predict_ultralytics(image)

        # publish vision result to rviz
        img_msg = self.bridge.cv2_to_imgmsg(vision_result, encoding="bgr8")
        img_msg.header = image.header

        # publish img to corresponding angle topic
        header_id = rospy.resolve_name(img_msg.header.frame_id)
        if (
            "Center" in header_id
            or "Back" in header_id
            or "Left" in header_id
            or "Right" in header_id
        ):
            side = header_id.split("/")[2]
            if side == "Center":
                self.publisher_center.publish(img_msg)
            if side == "Back":
                self.publisher_back.publish(img_msg)
            if side == "Left":
                self.publisher_left.publish(img_msg)
            if side == "Right":
                self.publisher_right.publish(img_msg)

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

    def predict_torch(self, image):
        """
        This function takes in an image from a camera and predicts a
        PyTorch model on the image. Depending on the the type of prediction
        a visualization function creates either a segmentation mask or bounding
        boxes for the image. The resulting image is returned.

        Args:
            image (image msg): image from a camera subscription

        Returns:
            image: visualization of prediction for rviz
        """

        # set model in evaluation mode
        self.model.eval()

        # preprocess image
        cv_image = self.bridge.imgmsg_to_cv2(
            img_msg=image, desired_encoding="passthrough"
        )
        cv_image_bgr = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        preprocess = t.Compose(
            [
                t.ToTensor(),
                t.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ]
        )
        input_image = preprocess(cv_image_bgr).unsqueeze(dim=0)

        # get prediction
        input_image = input_image.to(self.device)
        prediction = self.model(input_image)

        # apply visualition
        if self.type == "detection":
            vision_result = self.apply_bounding_boxes(cv_image, prediction[0])
        if self.type == "segmentation":
            vision_result = self.create_mask(cv_image, prediction["out"])

        return vision_result

    def predict_ultralytics(self, image):
        """
        This function takes in an image from a camera, predicts
        an ultralytics model on the image and looks for lidar points
        in the bounding boxes.

        This function also implements a visualization
        of what has been calculated for RViz.

        Args:
            image (image msg): image from camera subsription

        Returns:
            (cv image): visualization output for rvizw
        """

        # preprocess image
        cv_image = self.bridge.imgmsg_to_cv2(
            img_msg=image, desired_encoding="passthrough"
        )
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # run model prediction
        output = self.model(cv_image, half=True, verbose=False)

        # handle distance of objects

        # set up lists for visualization of distance outputs
        distance_output = []
        c_boxes = []
        c_labels = []

        boxes = output[0].boxes
        masks = output[0].masks.data
        markers = MarkerArray()  # Array of markers for visualization in ROS

        c_colors = []
        for i, box in enumerate(boxes):
            cls = box.cls.item()  # Klassenindex des Objekts
            pixels = box.xyxy[0]  # obere linke und untere rechte Pixelkoordinaten

            # track_id = (
            #    box.track_id if hasattr(box, "track_id") else None
            # )  # Tracking-ID could be used

            if self.dist_arrays is None:
                continue

            # crop bounding box area out of depth image
            distances = np.asarray(
                self.dist_arrays[
                    int(pixels[1]) : int(pixels[3]) : 1,
                    int(pixels[0]) : int(pixels[2]) : 1,
                    ::,
                ]
            )

            # set all 0 (black) values to np.inf (necessary if
            # you want to search for minimum)
            # these are all pixels where there is no
            # corresponding lidar point in the depth image
            condition = distances[:, :, 0] != 0
            non_zero_filter = distances[condition]
            distances_copy = distances.copy()
            distances_copy[distances_copy == 0] = np.inf

            # only proceed if there is more than one lidar
            # point in the bounding box
            if len(non_zero_filter) > 0:
                """
                !Watch out:
                The calculation of min x and min abs y is currently
                only for center angle
                For back, left and right the values are different in the
                coordinate system of the lidar.
                (Example: the closedt distance on the back view should the
                max x since the back view is on the -x axis)
                """

                # copy actual lidar points
                obj_dist_min_x = self.min_x(dist_array=distances_copy)
                obj_dist_min_abs_y = self.min_abs_y(dist_array=distances_copy)
                # absolut distance to object for visualization
                abs_distance = np.sqrt(
                    obj_dist_min_x[0] ** 2
                    + obj_dist_min_x[1] ** 2
                    + obj_dist_min_x[2] ** 2
                )

                # append class index, min x and min abs y to output array
                distance_output.append(float(cls))
                distance_output.append(float(obj_dist_min_x[0]))
                distance_output.append(float(obj_dist_min_abs_y[1]))
                # Just publish markers with closest point to object. No calculating of
                # the real object size atm
                markers.markers.append(self.get_marker(obj_dist_min_x, i, cls))

            else:
                # fallback values for bounding box if
                # no lidar points where found
                obj_dist_min_x = (np.inf, np.inf, np.inf)
                obj_dist_min_abs_y = (np.inf, np.inf, np.inf)
                abs_distance = np.inf

            # add values for visualization
            c_boxes.append(torch.tensor(pixels))
            c_labels.append(
                f"Class: {cls},"
                f"Meters: {round(abs_distance, 2)},"
                f"({round(float(obj_dist_min_x[0]), 2)},"
                f"{round(float(obj_dist_min_abs_y[1]), 2)})"
            )
            c_colors.append(get_carla_color(int(cls)))

        # publish list of distances of objects for planning
        self.distance_publisher.publish(Float32MultiArray(data=distance_output))
        self.marker_publisher.publish(markers)
        # transform image
        transposed_image = np.transpose(cv_image, (2, 0, 1))
        image_np_with_detections = torch.tensor(transposed_image, dtype=torch.uint8)

        # proceed with traffic light detection
        if 9 in output[0].boxes.cls:
            asyncio.run(self.process_traffic_lights(output[0], cv_image, image.header))

        # draw bounding boxes and distance values on image
        c_boxes = torch.stack(c_boxes)
        box_image = draw_bounding_boxes(
            image_np_with_detections,
            c_boxes,
            c_labels,
            colors="blue",
            width=3,
            font_size=12,
        )
        scaled_masks_list = []
        for i, mask in enumerate(masks.cpu().numpy()):
            # probably need to cut something off here
            scaled_masks_list.append(
                cv2.resize(
                    mask,
                    (cv_image.shape[1], cv_image.shape[0]),
                    interpolation=cv2.INTER_LINEAR,
                ),
            )
        scaled_masks = np.array(scaled_masks_list)
        mask_image = draw_segmentation_masks(
            box_image, torch.from_numpy(scaled_masks > 0), alpha=0.6, colors=c_colors
        )
        np_box_img = np.transpose(mask_image.detach().numpy(), (1, 2, 0))
        box_img = cv2.cvtColor(np_box_img, cv2.COLOR_BGR2RGB)
        # markers = self.get_markers(scaled_masks, self.dist_arrays)

        return box_img

    def get_marker(self, point, id, obj_class):
        c_color = get_carla_color(int(obj_class))
        marker = Marker()
        marker.header.frame_id = "hero"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "min_values"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = c_color[0]
        marker.color.g = c_color[1]
        marker.color.b = c_color[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = -point[2]
        marker.lifetime = rospy.Duration(0.5)
        return marker

    def min_x(self, dist_array):
        """
        Calculate min x (distance forward)

        Args:
            dist_array (np array): numpy array containing all
            lidar point in one bounding box

        Returns:
            np.array: 1x3 numpy array of min x lidar point
        """

        min_x_sorted_indices = np.argsort(dist_array[:, :, 0], axis=None)
        x, y = np.unravel_index(min_x_sorted_indices[0], dist_array.shape[:2])
        return dist_array[x][y].copy()

    def min_abs_y(self, dist_array):
        """
        Calculate min abs y (distance sideways)

        Args:
            dist_array (np array): numpy array containing all
            lidar point in one bounding box

        Returns:
            np.array: 1x3 numpy array of min abs y lidar point
        """

        abs_distance_copy = np.abs(dist_array.copy())
        min_y_sorted_indices = np.argsort(abs_distance_copy[:, :, 1], axis=None)
        x, y = np.unravel_index(min_y_sorted_indices[0], abs_distance_copy.shape[:2])
        return dist_array[x][y].copy()

    # you can add similar functions to support other camera angles here

    async def process_traffic_lights(self, prediction, cv_image, image_header):
        indices = (prediction.boxes.cls == 9).nonzero().squeeze().cpu().numpy()
        indices = np.asarray([indices]) if indices.size == 1 else indices

        max_y = 360  # middle of image
        min_prob = 0.30

        for index in indices:
            box = prediction.boxes.cpu().data.numpy()[index]

            if box[4] < min_prob:
                continue

            if (box[2] - box[0]) * 1.5 > box[3] - box[1]:
                continue  # ignore horizontal boxes

            if box[1] > max_y:
                continue

            box = box[0:4].astype(int)
            segmented = cv_image[box[1] : box[3], box[0] : box[2]]

            traffic_light_y_distance = box[1]

            traffic_light_image = self.bridge.cv2_to_imgmsg(segmented, encoding="rgb8")
            traffic_light_image.header = image_header
            traffic_light_image.header.frame_id = str(traffic_light_y_distance)
            self.traffic_light_publisher.publish(traffic_light_image)

    def create_mask(self, input_image, model_output):
        """
        function to create segmentation mask for pyTorch models

        Args:
            input_image (np.array): original image
            model_output (np.array): model output

        Returns:
            np.array: image with segmentation mask
        """

        output_predictions = torch.argmax(model_output, dim=0)
        for i in range(21):
            output_predictions[i] = output_predictions[i] == i

        output_predictions = output_predictions.to(dtype=torch.bool)

        transposed_image = np.transpose(input_image, (2, 0, 1))
        tensor_image = torch.tensor(transposed_image)
        tensor_image = tensor_image.to(dtype=torch.uint8)
        segmented_image = draw_segmentation_masks(
            tensor_image, output_predictions, alpha=0.6
        )
        cv_segmented = segmented_image.detach().cpu().numpy()
        cv_segmented = np.transpose(cv_segmented, (1, 2, 0))
        return cv_segmented

    def apply_bounding_boxes(self, input_image, model_output):
        """
        function to draw bounding boxes for pyTorch models

        Args:
            input_image (np.array): original image
            model_output (np.array): model output

        Returns:
            np.array: image with segmentation mask
        """

        transposed_image = np.transpose(input_image, (2, 0, 1))
        image_np_with_detections = torch.tensor(transposed_image, dtype=torch.uint8)
        boxes = model_output["boxes"]
        labels = [self.weights.meta["categories"][i] for i in model_output["labels"]]

        box = draw_bounding_boxes(
            image_np_with_detections,
            boxes,
            labels,
            colors="blue",
            width=3,
            font_size=24,
        )

        return np.transpose(box.detach().numpy(), (1, 2, 0))

    def run(self):
        self.spin()
        pass


if __name__ == "__main__":
    roscomp.init("VisionNode")
    node = VisionNode("VisionNode")
    node.run()
