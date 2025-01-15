# Comparison of the YOLO11n, YOLO11l and YOLO11x model

## Time of execution

### Manually reading out the execution times

For a first slight comparison a time delta is calculated around the function call for the image processing of ultralytics.

To print the time in ms the following code is used:

~~~python
time_start = time_ns()
if self.framework == "ultralytics":
    vision_result = self.predict_ultralytics(image)
time_delta = (time_ns() - time_start) / 1_000_000
print(f"{time_ns()}:\t{time_delta} ms")
~~~

The red results in ms are:

| YOLO | 11n   |   11l  |  11x  |
| :--: | :---: | :----: | :---: |
|  ms  | 36-76 | 40-120 | 32-95 |

### Calculating a mean time

The code for this

~~~python
time_start = time_ns()
if self.framework == "ultralytics":
    vision_result = self.predict_ultralytics(image)
time_delta = time_ns() - time_start
if time_delta / 1_000_000 < 100:
    self.benchTime = True
if self.benchTime is True:
    self.timeArray.append(time_delta)
    b = np.array(self.timeArray)
    c = np.mean(b) / 1_000_000
    print(f"{time_ns()}:\t{c} ms")
~~~

Results as mean over an array of times:

| YOLO | 11n   |   11l  |  11x  |
| :--: | :---: | :----: | :---: |
| ms   | 66 |    86  | 86 |

### Conclusion

The difference in time between the models is comprehensible, so that the biggest one with the best recognition can be used. This would be good to that effect that, with the YOLOv11x traffic lights get detected way earlier; about 10-20 meters.
**But** even the smallest net need 66ms on average to classify an image. This means the node can only run at $\frac{1}{66ms} \approx 15.2 Hz$. The bigger ones only at $\frac{1}{86ms} \approx 11.6 Hz$.
This is slower than the desired $20 Hz$.

## Troubleshooting

The question now is: why does it take so long. According to the [website of YOLO](https://docs.ultralytics.com/de/models/yolo11/) the processing of the images should take between 1.5 and 12.5 milliseconds (667 to 80Hz).
