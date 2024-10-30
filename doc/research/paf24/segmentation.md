Already implemented solutions:

https://github.com/una-auxme/paf/blob/c3011ee70039e199e106c54aa162a8f52be241a6/code/perception/launch/perception.launch?plain=1#L59-L61

Carla Sensor:
https://carla.readthedocs.io/en/0.8.4/cameras_and_sensors/
```
camera = carla.sensor.Camera('MyCamera', PostProcessing='SemanticSegmentation')
camera.set(FOV=90.0)
camera.set_image_size(800, 600)
camera.set_position(x=0.30, y=0, z=1.30)
camera.set_rotation(pitch=0, yaw=0, roll=0)

carla_settings.add_sensor(camera)
```

#Questions:
Is the already implemented solution using the "Sementic Sensor"?
How to convert the Carla Sementic Sensor into our code? (https://github.com/una-auxme/paf/blob/main/code/agent/src/agent/agent.py)
How the yolo8x-seg and deeplabv3_resnet101 was trained?