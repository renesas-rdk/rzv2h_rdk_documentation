.. _static_object_detection:

Static Object Detection
^^^^^^^^^^^^^^^^^^^^^^^

.. figure:: ../../images/static_objects.png
   :align: center
   :alt: Static Object Detection Demo
   :width: 600px

   Static Object Detection Demo

The RZ/V Static Object Detection package provides the following features:

- Demonstrates the usage of the AI library (DRP-AI) wrapped in a ROS 2 node.
- Supports the YOLOX Pascal VOC model (20 classes) for static object detection.
- Supports multi-threaded processing.
- Supports visualization through Foxglove Studio.

.. _required_ros2_packages_object_detection:

The following RZ/V ROS 2 packages are used:

- rzv_model
- rzv_object_detection
- foxglove_keypoint_publisher

Quick setup instructions:

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>` with the :ref:`required packages <required_ros2_packages_object_detection>` for this application.

#. Launch the Object Detection application.

   Load the workspace environment:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source <path/to>/install/setup.bash

   For static object detection, use:

   .. code-block:: bash

      # Pascal VOC object detection on static image
      ros2 launch rzv_object_detection static_object_detection_yolox.launch.py

      # Hand detection on static image using YOLOX
      ros2 launch rzv_object_detection static_hand_detection_yolox.launch.py

      # Hand detection on static image using Gold YOLOX
      ros2 launch rzv_object_detection static_hand_detection_gold_yolox.launch.py

#. For visualization using Foxglove Studio, refer to the :ref:`Foxglove Visualization <foxglove_visualization>` section for setup instructions.

   The input layout file for Foxglove Studio is located at
   ``rzv_object_detection/config/foxglove/objects_detection.json`` inside the ROS 2 workspace.

For more details about the Static Object Detection application, refer to the
`README.md in the rzv_object_detection package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_object_detection/-/blob/master/README.md?ref_type=heads>`_.

- v1.0.0 (2025-10-31): Initial release of the Static Object Detection sample application.