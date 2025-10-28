.. _mujoco_visualization:

MuJoCo Visualization
--------------------------

`MuJoCo <https://mujoco.org/>`_ is a simulation platform designed for robotics development, particularly in the context of ROS2.

It provides a virtual environment where developers can test and validate their robotic applications without the need for physical hardware.

**MuJoCo Setup**

Use the following steps to visualize a ROS application in MuJoCo.

1. Install the ROS2 Jazzy on the host PC as per `ROS2 Jazzy installation guide <https://docs.ros.org/en/jazzy/Installation.html>`_.
2. Install MuJoCo and the necessary ROS2 MuJoCo packages on your Host PC.

The required packages are, please install them into your **Host PC's ROS2 Jazzy workspace**:

- `mujoco_sim_ros2 <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/pc_host_ros_package/mujoco_sim_ros2>`_
- `mujoco_ros2_control <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/pc_host_ros_package/mujoco_ros2_control>`_
- `mujoco <https://github.com/google-deepmind/mujoco/releases/tag/3.3.0>`_

Additional, some other ROS2 packages are also required on the **Host PC's ROS2 Jazzy workspace** in order to run the MuJoCo simulation for the RZ/V Demo Arm Teleoperation application:

- `agilex_piper_arm_description <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/robots/agilex_piper_arm/agilex_piper_arm_description.git>`_
- `agilex_piper_mujoco <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/pc_host_ros_package/agilex_piper_mujoco.git>`_
- `cartesian_controllers <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/cartesian_controllers.git>`_

For other applications, please refer to the respective application documentation for any additional packages that may be required.

Those packages can be cloned into the ``src/`` directory of your :ref:`ROS2 Jazzy workspace <common_ros2_workspace_structure>` on the Host PC.

3. Build the ROS2 workspace on the Host PC.

.. code-block:: bash

   $ cd <path-to-your-ROS2-Jazzy-workspace>
   $ rosdep update
   $ rosdep install --from-paths src --ignore-src -r -y
   $ colcon build --symlink-install

4. Load the workspace environment:

.. code-block:: bash

   $ source /opt/ros/jazzy/setup.bash
   $ source <path-to-your-ROS2-Jazzy-workspace>/install/setup.bash

5. Launch the MuJoCo simulation, for example the RZ/V Demo Arm Teleoperation application.

.. code-block:: bash

   $ ros2 launch agilex_piper_mujoco bringup_mujoco_cartesian_motion_controller.launch.py

The MuJoCo simulator window should open, displaying the robotic arm like below:

.. figure:: images/mujoco_simulation.png
    :align: center
    :alt: MuJoCo Simulation
    :width: 600px

    MuJoCo Simulation of RZ/V Demo Arm Teleoperation Application