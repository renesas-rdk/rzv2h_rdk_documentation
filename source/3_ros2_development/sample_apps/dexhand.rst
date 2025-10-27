Vision Based Dexterous Hand
----------------------------

.. note::

   Available for :ref:`FoxGlove <foxglove_visualization>` simulation environment without real robotic hardware!

TODO: Add one image showing the dexterous hand manipulation using vision.

Key features
^^^^^^^^^^^^^

The RZ/V Demo DexHand package enables:

- Hand landmark estimation and interpretation.
- Simultaneous control of virtual and physical dexterous hands.
- Visualization through Foxglove Studio.
- Support for multiple dexterous hand models.
- Support for running two AI models simultaneously on the DRP-AI IP: one for hand detection and another for hand landmark estimation.
- Support for multiple AI models for both hand detection and hand landmark estimation.

.. _required_ros2_packages_dexhand:

RZ/V ROS2 Packages Used
^^^^^^^^^^^^^^^^^^^^^^^^

- arm_hand_control
- foxglove_keypoint_publisher
- rzv_demo_dexhand
- rzv_pose_estimation
- rzv_model

**Option 1:** Using Insprire RH6 hand

- inspire_rh56_urdf
- inspire_rh56_dexhand

**Option 2:** Using Ruiyan RH2 hand

- ruiyan_rh2_controller
- ruiyan_rh2_urdf
- ruiyan_rh2_dexhand

Quick Setup Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Prepare the cross compiled ROS2 workspace with the required packages mentioned above.

- Setup the RZ/V2H RDK board as per :ref:`RZ/V2H RDK board setup <quick_setup_rdk_guide>`.
- Setup the host machine for cross-compilation as per :ref:`Common docker environment setup <docker_sdk_setup>`.
- Collect all :ref:`required packages <required_ros2_packages_dexhand>` in the ``ros2_ws/src/`` directory inside the cross-compile docker container.
- Cross-compile the ROS2 workspace using :ref:`cross-build the ROS2 Application using Yocto SDK <requirements_ros2_cross_build>`.
- Deploy the ``install`` directory to the RZ/V2H RDK board using :ref:`Deploying the ROS2 Application <ros2_deployment>` or using the ``scp`` command.

2. Install the required dependencies on the RZ/V2H RDK board.

.. code-block:: bash

   $ rosdep install --from-paths <path/to>install/*/share -y -r --ignore-src

Please replace ``<path/to>install/`` with the actual path to the ``install/`` directory on your RZ/V2H RDK board.

3. **Optional**: Connect the dexterous hand to the RZ/V2H RDK board if you want to control the real hand.

TODO: Add instructions about setup script to configure serial port permissions as well as CAN port.

4. Connect a compatible USB camera to the RZ/V2H RDK board for hand detection and landmark estimation.

5. Launch the Vision Based Dexterous Hand application.

- Load the workspace environment:

.. code-block:: bash

   $ source /opt/ros/jazzy/setup.bash
   $ source <path/to>/install/setup.bash

- For real dexterous hand control, use:

.. code-block:: bash

   # For Inspire RH56 hand
   $ ros2 launch rzv_demo_dexhand demo_physical_inspire_rh56_hand.launch.py

   # For Ruiyan RH2 hand
   $ ros2 launch rzv_demo_dexhand demo_physical_ruiyan_rh2_hand.launch.py

- For virtual hand control (without real dexterous hand), use:

.. code-block:: bash

   $ ros2 launch rzv_demo_dexhand demo_virtual_hands.launch.py

6. For simulation using Foxglove Studio, refer to the :ref:`FoxGlove Visualization <foxglove_visualization>` section for setup instructions.

The input layout file for FoxGlove Studio is located at: ``rzv_demo_dexhand/config/foxglove/demo_dexhand.json`` inside the ROS2 workspace.

Application Details
^^^^^^^^^^^^^^^^^^^^^

For more details about the Vision Based Dexterous Hand application, refer to the `README.md in rzv_demo_dexhand package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_demo_dexhand/-/blob/master/README.md?ref_type=heads>`_ section.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the Vision Based Dexterous Hand sample application.