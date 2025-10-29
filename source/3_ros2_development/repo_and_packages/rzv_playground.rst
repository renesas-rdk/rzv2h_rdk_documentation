RZ/V Play Ground
------------------------------

Collection of demonstration and teleoperation launch files for robotic arm and hand systems on the RZ/V platform.

Overview
"""""""""""

This package provides comprehensive launch files for various robotic systems, featuring:

- Hand pose teleoperation: Vision-based control using MediaPipe hand tracking
- Teleoperation twist control: Keyboard/game-pad control for both cartesian and joint space
- Multi-robot support: Agilex Piper Arm, SO ARM101, and various hand configurations
- Visualization: Foxglove Studio integration for real-time monitoring
- Safety features: Mock hardware mode for testing without physical robots

For more details about the rzv_playground package, refer to the `README.md in the rzv_playground package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_playground/-/blob/master/README.md?ref_type=heads>`_.

RZ/V ROS2 Package Dependencies
""""""""""""""""""""""""""""""""

This package depends on the following RZ/V ROS2 packages:

- rzv_object_detection
- rzv_pose_estimation
- foxglove_keypoint_publisher
- arm_hand_control
- agilex_piper_arm_bringup
- piper_arm_inspire_hand_bringup
- piper_arm_ruiyan_hand_bringup
- so_arm101_bringup

For other dependencies, refer to the `package.xml file <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_playground/-/blob/master/package.xml?ref_type=heads>`_.

License
""""""""""

This package is licensed under the Apache License 2.0.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the rzv_playground package.
