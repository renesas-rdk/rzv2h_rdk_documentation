Foxglove Keypoint Publisher
----------------------------

A ROS2 package that publishes keypoint poses (landmarks or bounding boxes) as Foxglove image annotations for visualization purposes.

1. Overview

This package provides a ROS2 node that:

- Subscribes to pose array messages containing key-points (like hand landmarks or bounding boxes)
- Converts these poses to Foxglove image annotations for visualization
- Supports configurable visualization parameters through YAML files
- Includes test configurations for:

  - Hand landmarks
  - Bounding boxes
  - Body poses

For more details about the foxglove_keypoint_publisher package, refer to the `README.md in the foxglove_keypoint_publisher package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/foxglove_keypoint_publisher/-/blob/master/README.md?ref_type=heads>`_.

2. RZ/V ROS2 Package Dependencies

There is no direct dependency on other RZ/V ROS2 packages.

For other dependencies, refer to the `package.xml file <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/foxglove_keypoint_publisher/-/blob/master/package.xml?ref_type=heads>`_.

3. License

This package is licensed under the Apache License 2.0.

4. CHANGELOG

- v1.0.0 (2025-31-10): Initial release of the foxglove_keypoint_publisher package.
