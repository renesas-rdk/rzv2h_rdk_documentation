Model Zoo
---------

This section describes the AI model packages available for the RZ/V2H RDK platform.

These packages provide a unified C++ framework for deploying AI models optimized with DRP-AI acceleration,
enabling high power-efficiency inference on the RZ/V2H architecture.

Overview
^^^^^^^^

The RZ/V2H RDK AI model ecosystem is organized into the following layers:

- **Base Framework** (``rzv_model``): Core C++ library providing the ``BaseModel`` class, DRP-AI runtime integration, and shared utilities for model loading, preprocessing, inference, and postprocessing.
- **Model-Specific Packages**: Each AI model family (YOLOX - ``rzv_yolox``, YOLOv8 - ``rzv_yolov8``, HRNetV2 - ``rzv_hrnetv2``, RTMPose - ``rzv_rtmpose``, MediaPipe - ``rzv_mediapipe``, Gold-YOLO - ``rzv_gold_yolo``) is implemented as a separate package that extends the base framework with task-specific postprocessing logic.
- **ROS 2 Application Packages** (``rzv_object_detection``, ``rzv_pose_estimation``): ROS 2 nodes that combine the model packages with camera/image input and visualization output.
- **ROS 2 Utilities** (``rzv_model_utils_ros2``): Helper functions for integrating AI models into ROS 2 applications, including model configuration loading, message encoding, and diagnostics.

Available Packages
^^^^^^^^^^^^^^^^^^

**Core Packages**

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `rzv_model <https://github.com/renesas-rdk/rzv_model>`_
     - Base framework providing ``BaseModel`` class, DRP-AI runtime integration, and shared utilities for model loading, preprocessing, inference, and postprocessing.
     - Apache 2.0
   * - `rzv_model_utils_ros2 <https://github.com/renesas-rdk/rzv_model_utils_ros2>`_
     - ROS 2 utility library for model configuration loading, message encoding, and inference diagnostics.
     - Apache 2.0

**Object Detection Model Packages**

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `rzv_yolox <https://github.com/renesas-rdk/rzv_yolox>`_
     - YOLOX object detection with axis-aligned bounding boxes. Extends ``BaseModel`` with YOLOX-specific postprocessing.
     - Apache 2.0
   * - `rzv_yolov8 <https://github.com/renesas-rdk/rzv_yolov8>`_
     - YOLOv8 detection (axis-aligned and oriented bounding boxes) with DFL decoding and multi-threaded postprocessing.
     - AGPLv3
   * - `rzv_gold_yolo <https://github.com/renesas-rdk/rzv_gold_yolo>`_
     - Gold-YOLO object detection with axis-aligned bounding boxes.
     - Apache 2.0

**Pose Estimation Model Packages**

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `rzv_hrnetv2 <https://github.com/renesas-rdk/rzv_hrnetv2>`_
     - HRNetV2-based pose estimation with heatmap decoding for keypoint detection.
     - Apache 2.0
   * - `rzv_rtmpose <https://github.com/renesas-rdk/rzv_rtmpose>`_
     - RTMPose lightweight real-time pose estimation.
     - Apache 2.0
   * - `rzv_mediapipe <https://github.com/renesas-rdk/rzv_mediapipe>`_
     - MediaPipe hand landmark detection with handedness classification.
     - Apache 2.0

**ROS 2 Application Packages**

The following packages can be used as examples of how to integrate model packages into ROS 2 applications.
You can also refer to the source code of these packages for implementation details and best practices.

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `rzv_object_detection <https://github.com/renesas-rdk/rzv_object_detection>`_
     - ROS 2 nodes for camera-based and static image object detection using YOLOX, YOLOv8, and Gold-YOLO models.
     - Apache 2.0
   * - `rzv_pose_estimation <https://github.com/renesas-rdk/rzv_pose_estimation>`_
     - ROS 2 nodes for hand landmark estimation with a two-stage pipeline (detection + landmark).
     - Apache 2.0

.. toctree::
   :maxdepth: 1
   :caption: Model Zoo Contents

   rzv_model
   api_reference
   rzv_object_detection
   rzv_pose_estimation