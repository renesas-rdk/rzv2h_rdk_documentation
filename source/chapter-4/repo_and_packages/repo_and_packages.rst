Repositories and Packages
-------------------------

This section provides a quick reference for the ROS 2 packages available for development on the Renesas RZ/V2H RDK platform.

.. seealso::

   All packages are available in the `Renesas RZ/V2H RDK GitHub Repository <https://github.com/renesas-rdk>`_.

.. _package_reference:

Application Packages
^^^^^^^^^^^^^^^^^^^^

TODO: add complete packages, update this section to link to the actual applications and their descriptions.

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `rzv_demo_dexhand <https://github.com/renesas-rdk/rzv_demo_dexhand>`_
     - Launch files and configurations for vision-based dexterous hand control demo.
     - Apache 2.0
   * - `rzv_demo_rps <https://github.com/renesas-rdk/rzv_demo_rps>`_
     - Rock-Paper-Scissors gesture recognition and robotic hand game demo.
     - Apache 2.0
   * - `rzv_playground <https://github.com/renesas-rdk/rzv_playground>`_
     - Collection of teleoperation and demonstration launch files for robotic arm and hand systems.
     - Apache 2.0

AI / Perception Packages
^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `rzv_object_detection <https://github.com/renesas-rdk/rzv_object_detection>`_
     - Object detection (YOLOX, YOLOv8) for camera and static image input on RZ/V2H.
     - Apache 2.0
   * - `rzv_pose_estimation <https://github.com/renesas-rdk/rzv_pose_estimation>`_
     - Hand landmark detection and pose estimation on RZ/V2H.
     - Apache 2.0

Visualization Packages
^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `foxglove_keypoint_publisher <https://github.com/renesas-rdk/foxglove_keypoint_publisher>`_
     - Publishes keypoint poses as Foxglove image annotations for visualization.
     - Apache 2.0

Hand Control Packages
^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `arm_hand_control <https://github.com/renesas-rdk/arm_hand_control>`_
     - Hand gesture interpretation and robotic hand control via landmark tracking.
     - MIT
   * - `inspire_rh56_hand <https://github.com/renesas-rdk/inspire_rh56_hand>`_
     - Collection of packages (bringup, description, ros2_control, URDF, utils) for the Inspire RH56 hand.
     - Apache 2.0
   * - `ruiyan_rh2_hand <https://github.com/renesas-rdk/ruiyan_rh2_hand>`_
     - Collection of packages (controller, dexhand, URDF) for the Ruiyan RH2 hand.
     - Apache 2.0

Robotic Arm Packages
^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 55 20

   * - Package
     - Description
     - License
   * - `agilex_piper_arm <https://github.com/renesas-rdk/agilex_piper_arm>`_
     - Collection of packages (bringup, description, controller, ros2_control, utils, MuJoCo sim) for the AgileX Piper arm.
     - Apache 2.0
   * - `so_arm101 <https://github.com/renesas-rdk/so_arm101>`_
     - Collection of packages (bringup, description, ros2_control, utils) for the SO-ARM101 arm.
     - Apache 2.0
