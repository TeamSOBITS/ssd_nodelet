# SSD Nodelet

SSD(Single Shot MultiBox Detector)による物体検出（Nodelet実装による高速化）


![Face Detect Result](doc/test.png)

## How To Use

```bash
$ roslaunch ssd_nodelet face_detect.launch  <- face detect mode
$ roslaunch ssd_nodelet object_detect.launch  <- object detect mode
```

### Publications:
 * /rosout [rosgraph_msgs/Log]
 * /ssd_object_detect/detect_result [sensor_msgs/Image]
 * /ssd_object_detect/objects_name [sobit_common_msg/StringArray]
 * /ssd_object_detect/objects_num [std_msgs/UInt8]
 * /ssd_object_detect/objects_rect [sobit_common_msg/BoundingBoxes]

### Subscriptions:
 * /camera/rgb/image_raw [sensor_msgs/Image]
 * /ssd_object_detect/detect_ctrl [std_msgs/Bool]
