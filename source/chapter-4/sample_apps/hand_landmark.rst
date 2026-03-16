.. _hand_landmark:

Static / Camera-based Hand Landmark Estimation
--------------------------------------------------

.. figure:: ../../images/landmark.png
    :align: center
    :alt: Hand Landmark Demo
    :width: 600px

    Hand Landmark Demo

Key Features
^^^^^^^^^^^^^

The RZ/V Pose Estimation package enables:

- Demonstrates the usage of AI library (DRP-AI) that is wrapped in ROS 2 node.
- Hand detection and landmark estimation.
- Real-time camera-based estimation.
- Static image-based estimation.
- Smooth landmark tracking.
- Two-stage pipeline: Hand detection using YOLOX models -> Landmark estimation using various models.
- Support for multiple landmark models:

  1. MediaPipe Hand Landmark model
  2. HRNetV2 Hand Landmark model
  3. RTMPose Hand model
- EMA-based landmark smoothing.
- Integration with Foxglove Studio for visualization.
- Multi-threaded processing support.

.. _required_ros2_packages_hand_landmark:

RZ/V ROS 2 Packages Used
^^^^^^^^^^^^^^^^^^^^^^^^^

- rzv_pose_estimation
- rzv_model
- foxglove_keypoint_publisher

Quick Setup Instructions
^^^^^^^^^^^^^^^^^^^^^^^^

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>` with the :ref:`required packages <required_ros2_packages_hand_landmark>` for this application.

#. **Optional:** Connect a compatible USB camera to the RZ/V2H RDK board for hand detection and landmark estimation.

#. Launch the Static / Camera-based Hand Landmark Estimation application.

   Load the workspace environment:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source <path/to>/install/setup.bash

   For hand landmark estimation on static image, use:

   .. code-block:: bash

      ros2 launch rzv_pose_estimation static_hand_landmark_estimation.launch.py

   For hand landmark estimation using camera input, use:

   .. code-block:: bash

      ros2 launch rzv_pose_estimation camera_hand_landmark_estimation.launch.py

#. For visualization using Foxglove Studio, refer to the :ref:`Foxglove Visualization <foxglove_visualization>` section for setup instructions.

   The input layout file for Foxglove Studio is located at: ``rzv_pose_estimation/config/foxglove/landmark_estimation.json`` inside the ROS 2 workspace.

Application Details
^^^^^^^^^^^^^^^^^^^

For more details about the Static / Camera-based Hand Landmark Estimation application, refer to the `README.md in rzv_pose_estimation package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_pose_estimation/-/blob/master/README.md?ref_type=heads>`_.

Changelog
^^^^^^^^^

- v1.0.0 (2025-10-31): Initial release of the Static / Camera-based Hand Landmark Estimation sample application.
