.. _hand_landmark:

Static / Camera-based Hand Landmark Estimation
--------------------------------------------------

.. figure:: ../images/landmark.png
    :align: center
    :alt: Hand Landmark Demo
    :width: 600px

    Hand Landmark Demo

Key features
^^^^^^^^^^^^^

The RZ/V Pose Estimation package enables:

- Demonstrates the usage of AI library (DRP-AI) that is wrapped in ROS2 node.
- Hand detection and landmark estimation
- Real-time camera-based estimation
- Static image-based estimation
- Smooth landmark tracking
- Two-stage pipeline: Hand detection using YOLOX models -> Landmark estimation using various models
- Support for multiple landmark models:

  1. MediaPipe Hand Landmark model
  2. HRNetV2 Hand Landmark model
  3. RTMPose Hand model
- EMA-based landmark smoothing
- Integration with Foxglove Studio for visualization
- Multi-threaded processing support

.. _required_ros2_packages_hand_landmark:

RZ/V ROS2 Packages Used
^^^^^^^^^^^^^^^^^^^^^^^^

- rzv_pose_estimation
- rzv_model
- foxglove_keypoint_publisher

Quick Setup Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Prepare the cross compiled ROS2 workspace with the required packages mentioned above.

- Setup the RZ/V2H RDK board as per :ref:`RZ/V2H RDK board setup <quick_setup_rdk_guide>`.
- Setup the host machine for cross-compilation as per :ref:`Common docker environment setup <docker_sdk_setup>`.
- Collect all :ref:`required packages <required_ros2_packages_hand_landmark>` in the ``ros2_ws/src/`` directory inside the cross-compile docker container.
- Cross-compile the ROS2 workspace using :ref:`cross-build the ROS2 Application using Yocto SDK <requirements_ros2_cross_build>`.
- Deploy the ``install`` directory to the RZ/V2H RDK board using :ref:`Deploying the ROS2 Application <ros2_deployment>` or using the ``scp`` command.

2. Install the required dependencies on the RZ/V2H RDK board.

.. code-block:: bash

   $ rosdep install --from-paths <path/to>install/*/share -y -r --ignore-src

Please replace ``<path/to>install/`` with the actual path to the ``install/`` directory on your RZ/V2H RDK board.

3. **Optional:** Connect a compatible USB camera to the RZ/V2H RDK board for hand detection and landmark estimation.

4. Launch the Static / Camera-based Hand Landmark Estimation application.

- Load the workspace environment:

.. code-block:: bash

   $ source /opt/ros/jazzy/setup.bash
   $ source <path/to>/install/setup.bash

- For hand landmark estimation on static image, use:

.. code-block:: bash

   $ ros2 launch rzv_pose_estimation static_hand_landmark_estimation.launch.py

- For hand landmark estimation using camera input, use:

.. code-block:: bash

   $ ros2 launch rzv_pose_estimation camera_hand_landmark_estimation.launch.py

5. For visualization using Foxglove Studio, refer to the :ref:`FoxGlove Visualization <foxglove_visualization>` section for setup instructions.

The input layout file for FoxGlove Studio is located at: ``rzv_pose_estimation/config/foxglove/landmark_estimation.json`` inside the ROS2 workspace.

Application Details
^^^^^^^^^^^^^^^^^^^^^

For more details about the Static / Camera-based Hand Landmark Estimation application, refer to the `README.md in rzv_pose_estimation package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_pose_estimation/-/blob/master/README.md?ref_type=heads>`_ section.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the Static / Camera-based Hand Landmark Estimation sample application.