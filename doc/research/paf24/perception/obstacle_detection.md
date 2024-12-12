Summary: 
- Object detection
- Distance calculation
- Publishing of outputs

Object detection:
Sensors:
4 Cameras: center, (back, left, right not used)
Depth, Segmentation, RGB Camera? -> FusionCamera
Lidar
Radar (not used)

Distance calculation:
vision node receives depth-images from lidar_distance_node for specified camera angle
min x and min abs y distance within each bounding box (only ultralytics models)

Lidar sensor 3D values are transformed to 2D values to compare to camera images
Put both images on each other to calculate distances (distance in meters as values for pixels??, grayscale images)
lidar sensor flickers -> higher spin rate for full image leads to lower resolution

Publishing of outputs:
Class_Index
Min_X
Min_Abs_y

Cyclists are not detected
open car doors are mostly detected but still hit
standing car in front of our car is getting hit
Construction site sign not detected

/carla/hero/speed 9 subscribers -> Node /CollisionCheck (aus Planning.launch)->
/paf/hero/collision 3 subscribers, 1 publisher, std_msgs/Float32MultiArray
/paf/hero/Center/Object_distance 1 subscriber std_msgs/Float32MultiArray
/paf/hero/emergency 1 subscriber std_msgs/Bool
/paf/hero/oncoming 1 subscriber Gegenverkehr std_msgs/Float32

published distances: Type of detected item, min_x, min_y distances




    Are all obstacles detected? No
    Does successful object detection depend on:
        Speed of the car? Unknown yet
        Size of the obstacle? Unknown yet
        Kind of obstacle? 
        Dynamic/static obstacle? No cyclists/construction site signs are not always detected
    Which sensors are contributing to object detection 4 cameras (only front camera is used?), lidar, (radar - not used?)
    Do multiple sensors need to confirm detected obstacles? No, front camera and lidar images are compared for 1 distance result

