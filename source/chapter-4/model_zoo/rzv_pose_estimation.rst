.. _rzv_pose_estimation_package:

rzv_pose_estimation
^^^^^^^^^^^^^^^^^^^

A ROS 2 package for performing hand landmark estimation on the Renesas RZ/V2H platform. This package combines hand detection with landmark estimation to provide detailed hand pose analysis.

Overview
""""""""

This package provides ROS 2 nodes for:

- Hand detection and landmark estimation.
- Support for multiple landmark estimation models.
- Real-time camera-based estimation.
- Static image-based estimation.
- Smooth landmark tracking using EMA filtering.

Features
""""""""

- Two-stage pipeline:

  #. Hand detection using YOLOX models.
  #. Landmark estimation using various models.

- Support for multiple landmark models:

  - MediaPipe Hand Landmark model
  - HRNetV2 Hand Landmark model
  - RTMPose Hand model

- EMA-based landmark smoothing for temporal stability.
- Integration with Foxglove Studio for visualization.
- Multi-threaded processing support.

Node Parameters
"""""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 25 15 15 45

   * - Parameter Name
     - Data Type
     - Default
     - Description
   * - ``landmark_model_path``
     - string
     - ""
     - Path to the landmark model file.
   * - ``hand_detect_model_path``
     - string
     - ""
     - Path to the hand detection model file.
   * - ``processing_queue_size``
     - int
     - 1
     - Size of the image processing queue.
   * - ``confidence_threshold``
     - float
     - 0.5
     - Threshold for detection confidence.
   * - ``processing_threads``
     - int
     - 1
     - Number of processing threads.
   * - ``landmark_model_type``
     - string
     - "hrnetv2_hand_landmark"
     - Type of landmark model (``hrnetv2_hand_landmark``, ``rtmpose_hand``, ``mediapipe_hand_landmark``).
   * - ``smoothing_enabled``
     - bool
     - true
     - Whether to enable landmark smoothing.
   * - ``smoothing_factor``
     - double
     - 0.7
     - Smoothing factor for EMA (0-1, higher = more smoothing).

Topics
""""""

**Subscriptions**

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Topic
     - Type
     - Description
   * - ``/image_raw``
     - ``sensor_msgs/msg/Image``
     - Input images for pose estimation.

**Publishers**

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Topic
     - Type
     - Description
   * - ``bounding_box``
     - ``geometry_msgs/msg/PoseArray``
     - Hand detection results as pose arrays.
   * - ``hand_landmarks``
     - ``geometry_msgs/msg/PoseArray``
     - Hand landmark results as pose arrays.

Launch Files
""""""""""""

.. code-block:: bash

    # Hand landmark estimation using camera input
    ros2 launch rzv_pose_estimation camera_hand_landmark_estimation.launch.py

    # Hand landmark estimation on static image
    ros2 launch rzv_pose_estimation static_hand_landmark_estimation.launch.py

Model Performance Comparison
""""""""""""""""""""""""""""

.. list-table::
   :header-rows: 1
   :widths: 25 20 15 15 25

   * - Model Type
     - Accuracy
     - Speed
     - Memory
     - Notes
   * - ``hrnetv2_hand_landmark``
     - High
     - Medium
     - Medium
     - Best balance of accuracy and speed.
   * - ``rtmpose_hand``
     - Medium-High
     - Fast
     - Low
     - Good for resource-constrained systems.
   * - ``mediapipe_hand_landmark``
     - Medium
     - Fast
     - Low
     - Provides additional hand orientation data.

Usage Examples
""""""""""""""

.. code-block:: bash

    # Run with custom model type
    ros2 run rzv_pose_estimation hand_landmark_estimation \
        --ros-args -p landmark_model_type:=mediapipe_hand_landmark

    # Run with custom model paths
    ros2 run rzv_pose_estimation hand_landmark_estimation \
        --ros-args -p landmark_model_path:=/path/to/landmark/model \
        -p hand_detect_model_path:=/path/to/detection/model

    # Run with modified smoothing
    ros2 run rzv_pose_estimation hand_landmark_estimation \
        --ros-args -p smoothing_enabled:=false -p smoothing_factor:=0.5

    # Launch with custom camera device
    ros2 launch rzv_pose_estimation camera_hand_landmark_estimation.launch.py \
        video_device:=/dev/video1

Dependencies
""""""""""""

- ROS 2
- OpenCV
- ``rzv_model`` and related AI model packages
- ``image_publisher`` (for static images)
- ``v4l2_camera`` (for camera input)
- ``foxglove_keypoint_publisher`` (for visualization)

License
"""""""

This package is licensed under Apache 2.0.
