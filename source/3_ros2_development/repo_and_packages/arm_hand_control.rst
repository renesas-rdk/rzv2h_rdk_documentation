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

RZ/V ROS2 Package Dependencies
""""""""""""""""""""""""""""""""

This package depends on the following RZ/V ROS2 packages:

- inspire_rh56_dexhand

For other dependencies, refer to the `package.xml file <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/arm_hand_control/-/blob/master/package.xml?ref_type=heads>`_.

License
""""""""""

This package is licensed under the MIT License.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the arm_hand_control package.