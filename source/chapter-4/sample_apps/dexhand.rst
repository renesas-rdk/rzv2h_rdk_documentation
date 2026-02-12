.. _dexhand:

Vision Based Dexterous Hand
----------------------------

.. note::

   Available for :ref:`Foxglove <foxglove_visualization>` simulation environment without real robotic hardware!

.. figure:: ../../images/demo_dexhand.jpg
    :align: center
    :alt: DexHand Demo
    :width: 600px

    Dexterous Hand Demo

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

**Base package**

- arm_hand_control
- foxglove_keypoint_publisher
- rzv_demo_dexhand
- rzv_pose_estimation
- rzv_model

**Option 1:** Using Inspire RH56 hand

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

.. note::

    For cross-compilation by using Yocto SDK, please install ``ruiyan_rh2_controller/rh6_ctrl/lib/libRyhandArm64.so`` to the SDK sysroot using the following command:

    .. code-block:: bash

        $ sudo cp ruiyan_rh2_controller/rh6_ctrl/lib/libRyhandArm64.so $ROS2_SDK_SYSROOT/usr/lib/


2. Install the required dependencies on the RZ/V2H RDK board.

.. code-block:: bash

   $ rosdep install --from-paths <path/to>install/*/share -y -r --ignore-src

Please replace ``<path/to>install/`` with the actual path to the ``install/`` directory on your RZ/V2H RDK board.

3. **Optional**: Connect the dexterous hand to the RZ/V2H RDK board if you want to control the real hand.

.. note::

    Before using the RuiYan RH2 Dexhand, ensure that the hand is properly initialized using the provided setup script located in the ``ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh`` or in the ``install/ruiyan_rh2_dexhand/share/ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh`` after installation.

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

   # For Inspire RH56 hand
   $ ros2 launch rzv_demo_dexhand demo_virtual_inspire_rh56_hands.launch.py

   # For Ruiyan RH2 hand
   $ ros2 launch rzv_demo_dexhand demo_virtual_ruiyan_rh2_hands.launch.py

6. Based on your hand gesture shown in front of the camera, the dexterous hand will mimic your hand movements.

.. note::

   The common setup uses a fixed USB camera placed in front of the user and
   pointing **upward toward the hand**. The camera captures the palm from below,
   so that the **hand appears from bottom to top** in the image, the **wrist is
   at the bottom**, and the **fingers point upward**.

   When your hand is positioned correctly within the camera view, the **robot hand
   will mimic your gestures accurately**. The robot hand only interprets motion
   along the **vertical (bottom-to-top) direction**.

   Refer to the top image for the correct orientation between the camera and the user's hand.

7. For simulation using Foxglove Studio, refer to the :ref:`Foxglove Visualization <foxglove_visualization>` section for setup instructions.

The input layout file for Foxglove Studio is located at: ``rzv_demo_dexhand/config/foxglove/demo_dexhand.json`` inside the ROS2 workspace.

Application Details
^^^^^^^^^^^^^^^^^^^^^

For more details about the Vision Based Dexterous Hand application, refer to the `README.md in rzv_demo_dexhand package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_demo_dexhand/-/blob/master/README.md?ref_type=heads>`_ section.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the Vision Based Dexterous Hand sample application.