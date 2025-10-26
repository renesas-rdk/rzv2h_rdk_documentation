Arm Hand Control
-----------------

A ROS 2 package for controlling robotic hands through gesture recognition and landmark tracking.

Overview
"""""""""""

This package provides nodes for controlling robotic hands, particularly the Inspire RH56 Dexhand. It supports:

- Hand gesture interpretation (predefined gestures)
- Hand landmark interpretation (from hand tracking algorithms)
- Direct control of the Inspire RH56 Dexhand hardware
- Pick-and-place operations via action server

For more details about the arm_hand_control package, refer to the `README.md in the arm_hand_control package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/arm_hand_control/-/blob/master/README.md?ref_type=heads>`_.

Dependencies
""""""""""""""

- ROS 2
- sensor_msgs
- std_msgs
- geometry_msgs
- control_msgs
- trajectory_msgs
- tf2
- tf2_geometry_msgs
- yaml-cpp
- ament_index_cpp


License
""""""""""

This package is licensed under the MIT License.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the arm_hand_control package.