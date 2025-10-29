RZ/V Demo Dexhand
------------------------------

This package provides launch files and configurations for demonstrating dexterous hand control on Renesas RZ/V platforms.

It integrates vision-based pose estimation with both virtual and physical hand control.

Overview
"""""""""""

This package provide launch files to run the dexterous hand demo using hand landmark estimation from camera input.

It supports controlling both virtual and physical dexterous hands, with visualization through Foxglove Studio.

For more details about the rzv_demo_dexhand package, refer to the `README.md in the rzv_demo_dexhand package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_demo_dexhand/-/blob/update-readme-and-package/README.md?ref_type=heads>`_.

RZ/V ROS2 Package Dependencies
""""""""""""""""""""""""""""""""

This package depends on the following RZ/V ROS2 packages:

- arm_hand_control
- foxglove_keypoint_publisher
- inspire_rh56_urdf
- rzv_pose_estimation
- ruiyan_rh2_urdf

For other dependencies, refer to the `package.xml file <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_demo_dexhand/-/blob/update-readme-and-package/package.xml?ref_type=heads>`_.

License
""""""""""

This package is licensed under the Apache License 2.0.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the rzv_demo_dexhand package.
