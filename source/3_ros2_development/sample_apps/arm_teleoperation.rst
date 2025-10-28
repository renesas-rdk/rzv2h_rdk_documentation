Vision Based Robotic Arm Teleoperation
--------------------------------------------------------

.. note::

   Available for :ref:`FoxGlove <foxglove_visualization>` and :ref:`MuJoCo Visualization <mujoco_visualization>` simulation environment without real robotic hardware!

Key features
^^^^^^^^^^^^^

The RZ/V Demo Arm Teleoperation package enables:

- Detect hand landmarks via camera input and control the arm and gripper for grasping tasks.
- Support for running two AI models simultaneously on the DRP-AI IP.
- Mapping of hand landmarks to robotic arm and hand joint commands.
- Control of AglieX Piper Arm (6 DOFs) with dexterous robotic hands (Inspire RH56).
- Simultaneous control of virtual and physical AglieX Piper Arm.
- Visualization through Foxglove Studio and MuJoCo.

.. _required_ros2_packages_teleop_arm:

RZ/V ROS2 Packages Used
^^^^^^^^^^^^^^^^^^^^^^^^

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

**Optional**: With Inspire RH56 hand support

- inspire_rh56_urdf
- inspire_rh56_dexhand
- inspire_rh56_hand_bringup
- inspire_rh56_hand_utils
- piper_arm_inspire_hand_bringup

RZ/V ROS2 Host PC Packages Used
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Optional**: Those packages are required on the **host PC** if you want to use MuJoCo simulation:

- agilex_piper_arm_description
- agilex_piper_mujoco
- cartesian_controllers
- mujoco
- mujoco_ros2_control
- mujoco_sim_ros2

Please install the ROS2 Jazzy on the host PC as per `ROS2 Jazzy installation guide <https://docs.ros.org/en/jazzy/Installation.html>`_.

Quick Setup Instructions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Prepare the cross compiled ROS2 workspace with the required packages mentioned above.

- Setup the RZ/V2H RDK board as per :ref:`RZ/V2H RDK board setup <quick_setup_rdk_guide>`.
- Setup the host machine for cross-compilation as per :ref:`Common docker environment setup <docker_sdk_setup>`.
- Collect all :ref:`required packages <required_ros2_packages_teleop_arm>` in the ``ros2_ws/src/`` directory inside the cross-compile docker container.
- Cross-compile the ROS2 workspace using :ref:`cross-build the ROS2 Application using Yocto SDK <requirements_ros2_cross_build>`.
- Deploy the ``install`` directory to the RZ/V2H RDK board using :ref:`Deploying the ROS2 Application <ros2_deployment>` or using the ``scp`` command.

2. Install the required dependencies on the RZ/V2H RDK board.

.. code-block:: bash

   $ rosdep install --from-paths <path/to>install/*/share -y -r --ignore-src

Please replace ``<path/to>install/`` with the actual path to the ``install/`` directory on your RZ/V2H RDK board.

3. **Optional**: Connect the AglieX Piper Arm and Inspire RH56 hand to the RZ/V2H RDK board if you want to control the real arm and hand.

4. Connect a compatible USB camera to the RZ/V2H RDK board for hand detection and landmark estimation.

   - The common setup is that the camera is fixed in one position and faces upward.

   - The USB camera's field of view should capture the user's hand, and the user's hand must remain within the camera's frame.

5. Launch the Vision Based Robotic Arm Teleoperation application.

- Load the workspace environment:

.. code-block:: bash

   $ source /opt/ros/jazzy/setup.bash
   $ source <path/to>/install/setup.bash

- For real Agilex Piper Arm and Inspire RH56 Hand control, use:

.. code-block:: bash

   $ ros2 launch rzv_playground hand_palm_pose_teleop_inspire_hand.launch.py use_mock_hardware:=false

- For real Agilex Piper Arm with compatible Gripper, use:

.. code-block:: bash

   $ ros2 launch rzv_playground hand_palm_pose_teleop_piper_gripper.launch.py use_mock_hardware:=false

- For virtual hand control with FoxGlove (without real arm), use:

.. code-block:: bash

   $ ros2 launch rzv_playground hand_palm_pose_teleop_inspire_hand.launch.py use_mock_hardware:=false

- For virtual hand control with MuJoCo (without real arm), use:

.. code-block:: bash

   $ ros2 launch rzv_playground hand_palm_pose_teleop_piper_gripper.launch.py \
      bringup_launch_file:=agilex_piper_mujoco_cartesian_control.launch.py

Make sure to check the correct CAN interface and serial port parameters in the launch files before running the above commands.

6. For simulation using Foxglove Studio, refer to the :ref:`FoxGlove Visualization <foxglove_visualization>` section for setup instructions.

The input layout file for FoxGlove Studio is located at: ``rzv_playground/config/foxglove/*.json`` inside the ROS2 workspace.

For MuJoCo simulation, refer to the :ref:`MuJoCo Visualization <mujoco_visualization>` section for setup instructions.

After setting up the MuJoCo environment, you can visualize the robotic arm and hand movements in the MuJoCo simulator on your host PC:

.. code-block:: bash

   $ source /opt/ros/jazzy/setup.bash
   $ source <path/to>/install/setup.bash
   $ ros2 launch agilex_piper_mujoco bringup_mujoco_cartesian_motion_controller.launch.py

Make sure to set up the MuJoCo environment on your host PC as described in the :ref:`MuJoCo Visualization <mujoco_visualization>` section before running the above command.

Application Details
^^^^^^^^^^^^^^^^^^^^^

For more details about the Vision Based Robotic Arm Teleoperation application, refer to the `README.md in rzv_playground package <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/rzv_playground/-/blob/master/README.md?ref_type=heads>`_ section.

CHANGELOG
"""""""""""""

- v1.0.0 (2025-31-10): Initial release of the Vision Based Robotic Arm Teleoperation sample application.