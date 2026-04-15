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

Quick hardware setup instructions
""""""""""""""""""""""""""""""""""

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>`.

Quick software setup instructions
"""""""""""""""""""""""""""""""""

.. note::

   All subsequent operations must be executed inside :ref:`the cross-compilation Docker container <development_guide>`, which was set up in the :ref:`common setup step <sample_apps_prerequisites>`.

#. Clone the required source from GitHub by using the ``vcs`` tool inside the Docker container.

   Get the ``ros2_demo_workspace`` repository first:

   .. code-block:: bash

      cd ~/ros2_ws
      git clone https://github.com/renesas-rdk/ros2_demo_workspace.git

   Import the repositories by using the ``vcs`` command:

   .. code-block:: bash

      vcs import < ./ros2_demo_workspace/vcs_manifests/static_object_detection.target.lock.repos

   It will clone all required repositories to the ``./src`` folder.

#. Cross-compile the ROS 2 workspace and deploy it to the RZ/V2H RDK board.

   Update the APT repository list in the target sysroot.

   .. code-block:: bash

      rzv2h-chroot apt update

   Install the dependencies to the target board first:

   .. code-block:: bash

      sysroot-rosdep-install

   It will take time if you run this command for the first time.

   Cross-build the application:

   .. code-block:: bash

      cross-colcon-build

   Deploy the binaries to the target board:

   .. code-block:: bash

      scp -r install ubuntu@board_ip:~/ros2_ws/

   .. note::

      Replace ``board_ip`` with the actual IP address of your board. Ensure that the ``ros2_ws`` directory exists at ``/home/ubuntu`` on the target board before running the ``scp`` command.

Start the application
"""""""""""""""""""""

#. Install the required dependencies on the RZ/V2H RDK board.

   .. code-block:: bash

      cd /home/ubuntu/ros2_ws
      source /opt/ros/jazzy/setup.bash
      rosdep install --from-paths ./install/*/share -y -r --ignore-src

   The ``/home/ubuntu/ros2_ws`` directory is the location where you copied the cross-compiled workspace on the board.

#. Launch the Object Detection application.

   Load the workspace environment:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source ./install/setup.bash

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
`README.md in the rzv_object_detection package <https://github.com/renesas-rdk/rzv_object_detection>`_.

- v1.0.0 (2026-03-31): Initial release of the Static Object Detection sample application.