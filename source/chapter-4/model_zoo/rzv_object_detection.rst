.. _rzv_object_detection_package:

rzv_object_detection
--------------------

A ROS 2 package for performing object detection on the Renesas RZ/V2H platform. Implements various detection models and provides nodes for both static image and camera-based detection.

Overview
^^^^^^^^

This package provides ROS 2 nodes for:

- General object detection using YOLOX Pascal VOC model.
- Hand detection using YOLOX and Gold YOLOX models.
- Rock-Paper-Scissors hand detection using YOLOv8 models.
- Real-time camera-based detection.
- Static image-based detection.

Features
^^^^^^^^

- Support for multiple detection models:

  - YOLOX Pascal VOC (20 classes)
  - YOLOX Hand Detection
  - Gold YOLO Hand Detection
  - YOLOv8 RPS Hand Detection

- Configurable detection parameters.
- Integration with Foxglove Studio for visualization.
- Multi-threaded processing support.

Node Parameters
^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter Name
     - Data Type
     - Default
     - Description
   * - ``model_path``
     - string
     - ""
     - Path to the model file.
   * - ``model_type``
     - string
     - "yolox_pascal_voc"
     - Type of model to use (``yolox_pascal_voc``, ``yolox_hand``, ``gold_yolox_hand``, ``yolov8_rps``).
   * - ``processing_queue_size``
     - int
     - 5
     - Size of the image processing queue.
   * - ``confidence_threshold``
     - float
     - 0.5
     - Threshold for detection confidence.
   * - ``iou_threshold``
     - float
     - 0.45
     - Threshold for IoU in NMS.
   * - ``class_names``
     - string[]
     - []
     - Optional custom class names.
   * - ``processing_threads``
     - int
     - 1
     - Number of processing threads.

Topics
^^^^^^

**Subscriptions**

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Topic
     - Type
     - Description
   * - ``/image_raw``
     - ``sensor_msgs/msg/Image``
     - Input images for object detection.

**Publishers**

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Topic
     - Type
     - Description
   * - ``bounding_box``
     - ``geometry_msgs/msg/PoseArray``
     - Detection results as pose arrays.
   * - ``object_detect``
     - ``std_msgs/msg/String``
     - Detected object name.

Launch Files
^^^^^^^^^^^^

**Camera-based Detection**

.. code-block:: bash

    # Hand detection using camera input
    ros2 launch rzv_object_detection camera_hand_detection_yolox.launch.py

    # RPS hand detection using camera input
    ros2 launch rzv_object_detection camera_rps_hand_detection_yolov8.launch.py

**Static Image Detection**

.. code-block:: bash

    # Pascal VOC object detection on static image using YOLOX
    ros2 launch rzv_object_detection static_object_detection_yolox.launch.py

    # Hand detection on static image using YOLOX
    ros2 launch rzv_object_detection static_hand_detection_yolox.launch.py

    # Hand detection on static image using Gold YOLO
    ros2 launch rzv_object_detection static_hand_detection_gold_yolo.launch.py

    # RPS hand detection on static image using YOLOv8
    ros2 launch rzv_object_detection static_rps_hand_detection_yolov8.launch.py

Usage Examples
^^^^^^^^^^^^^^

.. code-block:: bash

    # Run with custom model type and thresholds
    ros2 run rzv_object_detection yolox_object_detection \
        --ros-args -p model_type:=yolox_hand \
        -p confidence_threshold:=0.6 -p iou_threshold:=0.4

    # Run with specific model path
    ros2 run rzv_object_detection yolox_object_detection \
        --ros-args -p model_path:=/path/to/custom/drpai/model

    # Launch with custom camera device
    ros2 launch rzv_object_detection camera_hand_detection_yolox.launch.py \
        video_device:=/dev/video1

Dependencies
^^^^^^^^^^^^

- ROS 2
- OpenCV
- ``rzv_model_utils_ros2``
- ``rzv_model`` and related AI model packages
- ``image_publisher`` (for static images)
- ``v4l2_camera`` (for camera input)
- ``foxglove_keypoint_publisher`` (for visualization)

License
^^^^^^^

This package is licensed under Apache 2.0.
