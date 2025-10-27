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

Application Details
^^^^^^^^^^^^^^^^^^^^^

For more details about the Vision Based Dexterous Hand application, refer to the `README.md in rzv_demo_dexhand package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_demo_dexhand/-/blob/master/README.md?ref_type=heads>`_ section.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the Vision Based Dexterous Hand sample application.