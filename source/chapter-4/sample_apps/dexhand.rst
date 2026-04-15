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

Quick hardware setup instructions
""""""""""""""""""""""""""""""""""

#. Complete the :ref:`Prerequisites for Running Sample Applications <sample_apps_prerequisites>`.

#. **Optional**: Connect the dexterous hand to the RZ/V2H RDK board if you want to control the real hand.

   .. note::

      Before using the Ruiyan RH2 Dexhand, ensure that the hand is properly initialized using the provided setup script located in ``ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh`` or in ``install/ruiyan_rh2_dexhand/share/ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh`` after installation.

#. Connect a compatible USB camera to the RZ/V2H RDK board for hand detection and landmark estimation.

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

      vcs import < ./ros2_demo_workspace/vcs_manifests/vision_based_dexterous_dand.target.lock.repos

   It will clone all required repositories to the ``./src`` folder.

#. Cross-compile the ROS 2 workspace and deploy it to the RZ/V2H RDK board.

   Update the APT repository list in the target sysroot.

   .. code-block:: bash

      rzv2h-chroot apt update

   Install the dependencies to the target board first:

   .. code-block:: bash

      sysroot-rosdep-install

   It will take time if you run this command for the first time.

   Copy the hand control library to the sysroot:

   .. code-block:: bash

      sudo cp /home/ubuntu/ros2_ws/src/ruiyan_rh2_controller/rh6_ctrl/lib/libRyhandArm64.so $V2H_SYSROOT/usr/lib/aarch64-linux-gnu/

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


#. Launch the Vision Based Dexterous Hand application.

   Load the workspace environment:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      source ./install/setup.bash

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
`README.md in the rzv_demo_dexhand package <https://github.com/renesas-rdk/rzv_demo_dexhand>`_
- v1.0.0 (2026-03-31): Initial release of the Vision Based Dexterous Hand sample application.