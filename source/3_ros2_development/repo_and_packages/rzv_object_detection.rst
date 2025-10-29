RZ/V Object Detection
------------------------------

A ROS2 package for performing object detection on Renesas RZ/V2H platform.

Implements various detection models and provides nodes for both static image and camera-based detection.

Overview
"""""""""""

This package provides ROS2 nodes for:

- General object detection using YOLOX Pascal VOC model
- Hand detection using YOLOX and Gold YOLOX models
- Rock-Paper-Scissors hand detection using YOLOv8 models
- Real-time camera-based detection
- Static image-based detection

For more details about the rzv_object_detection package, refer to the `README.md in the rzv_object_detection package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_object_detection/-/blob/master/README.md?ref_type=heads>`_.

RZ/V ROS2 Package Dependencies
""""""""""""""""""""""""""""""""

This package depends on the following RZ/V ROS2 packages:

- foxglove_keypoint_publisher

For other dependencies, refer to the `package.xml file <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_object_detection/-/blob/master/package.xml?ref_type=heads>`_.

License
""""""""""

This package is licensed under the Apache License 2.0.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the rzv_object_detection package.
