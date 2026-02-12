.. _static_object_detection:

Static Object Detection
----------------------------

.. figure:: ../../images/static_objects.png
    :align: center
    :alt: Static Object Detection Demo
    :width: 600px

    Static Object Detection Demo

Key features
^^^^^^^^^^^^^

The RZ/V Static Object Detection package enables:

- Demonstrates the usage of AI library (DRP-AI) that is wrapped in ROS2 node.
- Support YOLOX Pascal VOC model (20 classes) for static object detection.
- Multi-threaded processing support.
- Visualization through Foxglove Studio.

.. _required_ros2_packages_object_detection:

RZ/V ROS2 Packages Used
^^^^^^^^^^^^^^^^^^^^^^^^

- rzv_model
- rzv_object_detection
- foxglove_keypoint_publisher

Quick Setup Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Prepare the cross compiled ROS2 workspace with the required packages mentioned above.

- Setup the RZ/V2H RDK board as per :ref:`RZ/V2H RDK board setup <quick_setup_rdk_guide>`.
- Setup the host machine for cross-compilation as per :ref:`Common docker environment setup <docker_sdk_setup>`.
- Collect all :ref:`required packages <required_ros2_packages_object_detection>` in the ``ros2_ws/src/`` directory inside the cross-compile docker container.
- Cross-compile the ROS2 workspace using :ref:`cross-build the ROS2 Application using Yocto SDK <requirements_ros2_cross_build>`.
- Deploy the ``install`` directory to the RZ/V2H RDK board using :ref:`Deploying the ROS2 Application <ros2_deployment>` or using the ``scp`` command.

2. Install the required dependencies on the RZ/V2H RDK board.

.. code-block:: bash

   $ rosdep install --from-paths <path/to>install/*/share -y -r --ignore-src

Please replace ``<path/to>install/`` with the actual path to the ``install/`` directory on your RZ/V2H RDK board.

3. Launch the Object Detection application.

- Load the workspace environment:

.. code-block:: bash

   $ source /opt/ros/jazzy/setup.bash
   $ source <path/to>/install/setup.bash

- For static object detection, use:

.. code-block:: bash

   # Pascal VOC object detection on static image
   $ ros2 launch rzv_object_detection static_object_detection_yolox.launch.py

   # Hand detection on static image using YOLOX
   $ ros2 launch rzv_object_detection static_hand_detection_yolox.launch.py

   # Hand detection on static image using Gold YOLOX
   $ ros2 launch rzv_object_detection static_hand_detection_gold_yolox.launch.py

4. For visualization using Foxglove Studio, refer to the :ref:`Foxglove Visualization <foxglove_visualization>` section for setup instructions.

The input layout file for Foxglove Studio is located at: ``rzv_object_detection/config/foxglove/objects_detection.json`` inside the ROS2 workspace.

Application Details
^^^^^^^^^^^^^^^^^^^^^

For more details about the Static Object Detection application, refer to the `README.md in rzv_object_detection package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_object_detection/-/blob/master/README.md?ref_type=heads>`_ section.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the Static Object Detection sample application.