RZ/V Demo RPS
-----------------------

This ROS 2 package enables hand using rock-paper-scissors gesture recognition.

It captures hand gestures through a vision-based recognition system and translates them into control commands to interact with games.

Beside that, also providing launch files and configurations for demonstrating Rock-Paper-Scissor on Renesas RZ/V platforms.

1. Overview

This package provides node for controlling robotic hands. It supports:

- Rock-Paper-Scissors Controller: Detects Rock Paper Scissors gestures in real time, executes the game logic, and sends commands to control the robotic hand accordingly.
- Compatible with the Inspire RH56 Dexhand and Ruiyan RH2 robotic hands.
- RPS Object detection and interpretation
- Simultaneous control of virtual and physical dexterous hands
- Visualization through Foxglove Studio

For more details about the rzv_demo_rps package, refer to the `README.md in the rzv_demo_rps package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_demo_rps/-/blob/main/README.md?ref_type=heads>`_.

2. RZ/V ROS2 Package Dependencies

This package depends on the following RZ/V ROS2 packages:

- arm_hand_control
- rzv_object_detection
- foxglove_keypoint_publisher
- inspire_rh56_urdf
- ruiyan_rh2_urdf

For other dependencies, refer to the `package.xml file <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_demo_rps/-/blob/main/package.xml?ref_type=heads>`_.

3. License

This package is licensed under the Apache License 2.0.

4. CHANGELOG

- v1.0.0 (2025-31-10): Initial release of the rzv_demo_rps package.
