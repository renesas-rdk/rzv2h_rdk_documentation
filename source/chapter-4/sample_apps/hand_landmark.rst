.. _hand_landmark:

Static / Camera-based Hand Landmark Estimation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: ../../images/landmark.png
   :align: center
   :alt: Hand Landmark Demo
   :width: 600px

   Hand Landmark Demo

The RZ/V Pose Estimation package provides the following features:

- Demonstrates the usage of the AI library (DRP-AI) wrapped in a ROS 2 node.
- Supports hand detection and landmark estimation.
- Supports real-time camera-based estimation.
- Supports static image-based estimation.
- Supports smooth landmark tracking.
- Supports a two-stage pipeline: hand detection using YOLOX models, followed by landmark estimation using various models.
- Supports multiple landmark models:

  #. MediaPipe Hand Landmark model
  #. HRNetV2 Hand Landmark model
  #. RTMPose Hand model

- Supports EMA-based landmark smoothing.
- Integrates with Foxglove Studio for visualization.
- Supports multi-threaded processing.

.. _required_ros2_packages_hand_landmark:

The following RZ/V ROS 2 packages are used:

- rzv_pose_estimation
- rzv_model
- foxglove_keypoint_publisher

Quick setup instructions:

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>` with the :ref:`required packages <required_ros2_packages_hand_landmark>` for this application.

#. **Optional:** Connect a compatible USB camera to the RZ/V2H RDK board for hand detection and landmark estimation.

#. Launch the Static / Camera-based Hand Landmark Estimation application.

   Load the workspace environment:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source <path/to>/install/setup.bash

   For hand landmark estimation on a static image, use:

   .. code-block:: bash

      ros2 launch rzv_pose_estimation static_hand_landmark_estimation.launch.py

   For hand landmark estimation using camera input, use:

   .. code-block:: bash

      ros2 launch rzv_pose_estimation camera_hand_landmark_estimation.launch.py

#. For visualization using Foxglove Studio, refer to the :ref:`Foxglove Visualization <foxglove_visualization>` section for setup instructions.

   The input layout file for Foxglove Studio is located at
   ``rzv_pose_estimation/config/foxglove/landmark_estimation.json`` inside the ROS 2 workspace.

For more details about the Static / Camera-based Hand Landmark Estimation application, refer to the
`README.md in the rzv_pose_estimation package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_pose_estimation/-/blob/master/README.md?ref_type=heads>`_.

- v1.0.0 (2025-10-31): Initial release of the Static / Camera-based Hand Landmark Estimation sample application.