RZ/V Pose Estimation
------------------------------

A ROS2 package for performing hand landmark estimation on Renesas RZ/V2H platform.

This package combines hand detection with landmark estimation to provide detailed hand pose analysis.

1. Overview

This package provides ROS2 nodes for:

- Hand detection and landmark estimation
- Support for multiple landmark estimation models
- Real-time camera-based estimation
- Static image-based estimation
- Smooth landmark tracking

For more details about the rzv_pose_estimation package, refer to the `README.md in the rzv_pose_estimation package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_pose_estimation/-/blob/master/README.md?ref_type=heads>`_.

2. RZ/V ROS2 Package Dependencies

This package depends on the following RZ/V ROS2 packages:

- foxglove_keypoint_publisher

For other dependencies, refer to the `package.xml file <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_pose_estimation/-/blob/master/package.xml?ref_type=heads>`_.

3. License

This package is licensed under the Apache License 2.0.

4. CHANGELOG

- v1.0.0 (2025-31-10): Initial release of the rzv_pose_estimation package.
