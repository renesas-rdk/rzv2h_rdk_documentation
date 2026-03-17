Vision Based Dexterous Hand
^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _dexhand:

.. note::

   Available for :ref:`Foxglove <foxglove_visualization>` simulation environment without real robotic hardware!

.. figure:: ../../images/demo_dexhand.jpg
   :align: center
   :alt: DexHand Demo
   :width: 600px

   Dexterous Hand Demo

The RZ/V Demo DexHand package provides the following features:

- Supports hand landmark estimation and interpretation.
- Supports simultaneous control of virtual and physical dexterous hands.
- Supports visualization through Foxglove Studio.
- Supports multiple dexterous hand models.
- Supports running two AI models simultaneously on the DRP-AI IP: one for hand detection and another for hand landmark estimation.
- Supports multiple AI models for both hand detection and hand landmark estimation.

.. _required_ros2_packages_dexhand:

The following RZ/V ROS 2 packages are used.

**Base packages**

- arm_hand_control
- foxglove_keypoint_publisher
- rzv_demo_dexhand
- rzv_pose_estimation
- rzv_model

**Option 1: Using Inspire RH56 hand**

- inspire_rh56_urdf
- inspire_rh56_dexhand

**Option 2: Using Ruiyan RH2 hand**

- ruiyan_rh2_controller
- ruiyan_rh2_urdf
- ruiyan_rh2_dexhand

Quick setup instructions:

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>` with the :ref:`required packages <required_ros2_packages_dexhand>` for this application.

#. **Optional**: Connect the dexterous hand to the RZ/V2H RDK board if you want to control the real hand.

   .. note::

      Before using the Ruiyan RH2 Dexhand, ensure that the hand is properly initialized using the provided setup script located in ``ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh`` or in ``install/ruiyan_rh2_dexhand/share/ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh`` after installation.

#. Connect a compatible USB camera to the RZ/V2H RDK board for hand detection and landmark estimation.

#. Launch the Vision Based Dexterous Hand application.

   Load the workspace environment:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source <path/to>/install/setup.bash

   For real dexterous hand control, use:

   .. code-block:: bash

      # For Inspire RH56 hand
      ros2 launch rzv_demo_dexhand demo_physical_inspire_rh56_hand.launch.py

      # For Ruiyan RH2 hand
      ros2 launch rzv_demo_dexhand demo_physical_ruiyan_rh2_hand.launch.py

   For virtual hand control (without a real dexterous hand), use:

   .. code-block:: bash

      # For Inspire RH56 hand
      ros2 launch rzv_demo_dexhand demo_virtual_inspire_rh56_hands.launch.py

      # For Ruiyan RH2 hand
      ros2 launch rzv_demo_dexhand demo_virtual_ruiyan_rh2_hands.launch.py

#. Based on the hand gesture shown in front of the camera, the dexterous hand mimics the observed hand movement.

   .. note::

      The common setup uses a fixed USB camera placed in front of the user and
      pointing **upward toward the hand**. The camera captures the palm from below,
      so the **hand appears from bottom to top** in the image, the **wrist is at
      the bottom**, and the **fingers point upward**.

      When the hand is positioned correctly within the camera view, the **robot hand
      mimics the gestures accurately**. The robot hand interprets motion only along
      the **vertical (bottom-to-top) direction**.

      Refer to the image above for the correct orientation between the camera and the user's hand.

#. For simulation using Foxglove Studio, refer to the :ref:`Foxglove Visualization <foxglove_visualization>` section for setup instructions.

   The input layout file for Foxglove Studio is located at
   ``rzv_demo_dexhand/config/foxglove/demo_dexhand.json`` inside the ROS 2 workspace.

For more details about the Vision Based Dexterous Hand application, refer to the
`README.md in the rzv_demo_dexhand package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_demo_dexhand/-/blob/master/README.md?ref_type=heads>`_.

- v1.0.0 (2025-10-31): Initial release of the Vision Based Dexterous Hand sample application.