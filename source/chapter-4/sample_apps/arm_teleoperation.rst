.. _arm_teleoperation:

Vision Based Robotic Arm Teleoperation
--------------------------------------

.. note::

   Available for :ref:`Foxglove <foxglove_visualization>` and :ref:`MuJoCo Visualization <mujoco_visualization>` simulation environment without real robotic hardware!

.. figure:: ../../images/arm_teleop.png
   :alt: Arm Teleoperation Demo
   :width: 600px
   :align: center

   Arm Teleoperation Demo.

Key Features
^^^^^^^^^^^^

The RZ/V Demo Arm Teleoperation package enables:

- Detection of hand landmarks via camera input to control the arm and gripper for grasping tasks.
- Support for running two AI models simultaneously on the DRP-AI IP.
- Mapping of hand landmarks to robotic arm and hand joint commands.
- Control of AgileX Piper Arm (6 DOFs) with dexterous robotic hands (Inspire RH56).
- Simultaneous control of virtual and physical AgileX Piper Arm.
- Visualization through Foxglove Studio and MuJoCo.

.. _required_ros2_packages_teleop_arm:

RZ/V ROS 2 Packages Used
^^^^^^^^^^^^^^^^^^^^^^^^^

TODO: update link to each packages.

- agilex_piper_arm_bringup
- agilex_piper_controller
- agilex_piper_utils
- agilex_piper_arm_description
- agilex_piper_ros2_control
- arm_hand_control
- cartesian_controllers
- rzv_model
- rzv_pose_estimation
- foxglove_keypoint_publisher
- rzv_playground

**Optional**: With Inspire RH56 hand support:

- inspire_rh56_description
- inspire_rh56_dexhand
- inspire_rh56_hand_bringup
- inspire_rh56_hand_utils
- inspire_rh56_hand_ros2_control
- piper_arm_inspire_hand_bringup

RZ/V ROS 2 Host PC Packages Used
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. note::

   These packages are required on the **host PC** only if you want to use :ref:`MuJoCo simulation <mujoco_visualization>`.

- agilex_piper_arm_description
- agilex_piper_mujoco
- cartesian_controllers
- mujoco
- mujoco_ros2_control
- mujoco_sim_ros2

Install ROS 2 Jazzy on the host PC as per the `ROS 2 Jazzy installation guide <https://docs.ros.org/en/jazzy/Installation.html>`_.

Quick Setup Instructions
^^^^^^^^^^^^^^^^^^^^^^^^

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>` with the :ref:`required packages <required_ros2_packages_teleop_arm>` for this application.

#. **Optional**: Connect the AgileX Piper Arm and Inspire RH56 hand to the RZ/V2H RDK board if you want to control the real arm and hand.

#. Connect a compatible USB camera to the RZ/V2H RDK board for hand detection and landmark estimation.

   - The common setup is that the camera is fixed in one position and faces upward.
   - The USB camera's field of view should capture the user's hand, and the user's hand must remain within the camera's frame.

#. Launch the Vision Based Robotic Arm Teleoperation application.

   Load the workspace environment:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source <path/to>/install/setup.bash

   For real AgileX Piper Arm and Inspire RH56 Hand control:

   .. code-block:: bash

      ros2 launch rzv_playground hand_palm_pose_teleop_inspire_hand.launch.py use_mock_hardware:=false

   For real AgileX Piper Arm with compatible Gripper:

   .. code-block:: bash

      ros2 launch rzv_playground hand_palm_pose_teleop_piper_gripper.launch.py use_mock_hardware:=false

   For virtual hand control with Foxglove (without real arm):

   .. code-block:: bash

      ros2 launch rzv_playground hand_palm_pose_teleop_inspire_hand.launch.py use_mock_hardware:=true

   For virtual hand control with MuJoCo (without real arm):

   .. code-block:: bash

      ros2 launch rzv_playground hand_palm_pose_teleop_piper_gripper.launch.py \
         bringup_launch_file:=agilex_piper_mujoco_cartesian_control.launch.py

   Make sure to check the correct CAN interface and serial port parameters in the launch files before running the above commands.

#. Visualize the robotic arm and hand movements by following the instructions below:

   - Move your hand **up or down**, the Piper arm will move **up or down** accordingly.
   - Move your hand **forward or backward**, the Piper arm will move **forward or backward**.
   - Move your hand **left or right**, the Piper arm will move **left or right**.
   - **Close your thumb**, the robotic hand or gripper will switch to the **grasping position**.
   - If the system **cannot detect your hand** after a certain period, the Piper arm will **reset to its initial position**.

#. Set up visualization.

   For Foxglove Studio, refer to the :ref:`Foxglove Visualization <foxglove_visualization>` section for setup instructions.
   The input layout file for Foxglove Studio is located at ``rzv_playground/config/foxglove/*.json`` inside the ROS 2 workspace.

   For MuJoCo simulation, refer to the :ref:`MuJoCo Visualization <mujoco_visualization>` section for setup instructions.
   After setting up the MuJoCo environment, visualize the robotic arm and hand movements in the MuJoCo simulator on your host PC:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source <path/to>/install/setup.bash
      ros2 launch agilex_piper_mujoco bringup_mujoco_cartesian_motion_controller.launch.py

   .. note::

      Make sure to set up the MuJoCo environment on your host PC as described in the :ref:`MuJoCo Visualization <mujoco_visualization>` section before running the above command.

Application Details
^^^^^^^^^^^^^^^^^^^

For more details about the Vision Based Robotic Arm Teleoperation application, refer to the `README.md in rzv_playground package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_playground/-/blob/master/README.md?ref_type=heads>`_.

Changelog
^^^^^^^^^

- v1.0.0 (2025-10-31): Initial release of the Vision Based Robotic Arm Teleoperation sample application.