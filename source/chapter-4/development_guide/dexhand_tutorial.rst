Tutorial with Vision Based Dexterous Hand
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This tutorial explains the complete ROS 2 application workflow on the Renesas RZ/V2H RDK platform using the :ref:`Vision Based Dexterous Hand <dexhand>` application as an example.

You do not need to understand the internal implementation details of the application for this tutorial.

This page focuses only on the practical development workflow after the environment is already prepared. It covers:

- building the application in the Docker-based development environment,
- deploying the built files to the target device,
- running the application on the target device, and
- remotely debugging the application from VS Code.

.. note::

   This tutorial assumes that the :ref:`software environment and hardware environment <requirements_ros2_cross_build>` are already set up.

   In particular, the following are assumed to be ready:

   - the Docker-based ROS 2 development environment,
   - the cross-compilation environment,
   - the VS Code Dev Container environment,
   - the target board connection, and
   - the required runtime dependencies on the target device.

Tutorial Scenario
"""""""""""""""""

In this tutorial, the example application is the :ref:`Vision Based Dexterous Hand <dexhand>` demo.

The tutorial uses the following example launch file for the virtual hand workflow:

.. code-block:: bash

   ros2 launch rzv_demo_dexhand demo_virtual_inspire_rh56_hands.launch.py

You can replace it with another launch file later if needed, such as:

.. code-block:: bash

   ros2 launch rzv_demo_dexhand demo_virtual_ruiyan_rh2_hands.launch.py

or a physical hand launch file if your hardware is connected.

Before You Begin
""""""""""""""""

Make sure the required source packages for the Dexterous Hand application are already available in your workspace under ``src/`` in the Docker container.

Example workspace location in the Docker container:

.. code-block:: bash

   /home/ubuntu/ros2_ws

The workspace should contain the ROS 2 source packages and the VS Code configuration files used in this tutorial:

.. code-block:: text

   ros2_ws/
   ├── build                           # Generated after cross-building the workspace
   ├── install                         # Generated after cross-building the workspace
   ├── log                             # Generated after cross-building the workspace
   ├── src                             # Must contain the required source packages for the Dexterous Hand application
   │   ├── arm_hand_control
   │   ├── foxglove_keypoint_publisher
   │   ├── rzv_demo_dexhand
   │   ├── rzv_model
   │   ├── rzv_pose_estimation
   │   ├── inspire_rh56_dexhand
   │   ├── inspire_rh56_urdf
   │   ├── ruiyan_rh2_controller
   │   ├── ruiyan_rh2_dexhand
   │   └── ruiyan_rh2_urdf
   └── .vscode

At minimum, make sure the base packages required by the Dexterous Hand application are available under ``src/``.

Add the optional packages according to the hand model you want to use.

Step 1: Open the Workspace in VS Code
"""""""""""""""""""""""""""""""""""""

Open the Docker container in VS Code.

If the current workspace is not ``/home/ubuntu/ros2_ws``, open it manually:

#. In VS Code, select **File > Open Folder**.
#. Open the following directory inside the Docker container:

   .. code-block:: bash

      /home/ubuntu/ros2_ws

Make sure ``/home/ubuntu/ros2_ws`` is the active workspace before continuing.

Step 2: Configure ``settings.json``
"""""""""""""""""""""""""""""""""""

Open the workspace configuration file:

.. code-block:: bash

   code .vscode/settings.json

If the ``code`` command is not available inside the container, open the file directly from the VS Code Explorer.

For this tutorial, update the key variables in ``.vscode/settings.json`` as follows:

.. code-block:: json

   {
     "TARGET_IP": "192.168.0.10",     // Replace with the actual target IP address
     "NODE_PACKAGE_NAME": "rzv_pose_estimation",
     "NODE_EXECUTABLE_NAME": "hand_landmark_estimation",
     "LAUNCH_PACKAGE_NAME": "rzv_demo_dexhand",
     "LAUNCH_FILE_NAME": "demo_virtual_inspire_rh56_hands.launch.py"
   }

Replace ``192.168.0.10`` with the actual IP address of your RZ/V2H RDK board.

If you are not sure about the purpose of a macro, refer to the comments in the file or the :ref:`workspace settings <workspace_settings>` section for more details.

.. note::

   The exact executable name used for debugging depends on which node inside the launch file you want to run or debug.

   If you are not sure, first run the launch file normally on the target, then inspect the launched processes and update ``NODE_EXECUTABLE_NAME`` to match the specific executable you want to debug.

For this tutorial:

- ``LAUNCH_PACKAGE_NAME`` and ``LAUNCH_FILE_NAME`` are used to start the Dexterous Hand application.
- ``NODE_PACKAGE_NAME`` and ``NODE_EXECUTABLE_NAME`` are used when you want to debug one specific node.

Step 3: Install Sysroot Dependencies and Build the Workspace
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

Before building the workspace, install the required dependencies into the target sysroot.

From the ROS 2 workspace inside the Docker container, run:

.. code-block:: bash

   sysroot-rosdep-install

This command installs the dependencies required by the packages in the current workspace into the sysroot used for cross-compilation.

After that, build the workspace by using the cross-build command.

For a release build, run:

.. code-block:: bash

   cross-colcon-build

Other build options are also supported. Refer to :ref:`Build the Application <build_deploy>` for more details.

After a successful build, confirm that the ``install`` directory exists.

Step 4: Deploy the Built Files to the Target Device
""""""""""""""""""""""""""""""""""""""""""""""""""""

Deploy the generated ``install`` directory to the target board.

If you use the provided VS Code workflow, run the task:

- Press ``Ctrl+Shift+P`` to open the Command Palette.
- Run **Tasks: Run Task**.
- Then choose **ROS2: Deploy to Target**.

Or you can use the **Task Buttons** extension:

- Click the **Deploy** button in the VS Code status bar to start the deployment process.

Step 5: Install Runtime Dependencies on the Target
"""""""""""""""""""""""""""""""""""""""""""""""""""

On the target device, install the required runtime dependencies if they are not already installed.

.. code-block:: bash

   cd /home/ubuntu/ros2_ws
   source /opt/ros/jazzy/setup.bash
   rosdep install --from-paths ./install/*/share -y -r --ignore-src

If needed, install ``gdbserver`` for remote debugging:

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install gdbserver

Step 6: Run the Dexterous Hand Application on the Target
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""

This step describes two ways to run the Dexterous Hand application on the target:

- Run directly on the target over SSH
- Run remotely from VS Code on the host

Run Directly on the Target
~~~~~~~~~~~~~~~~~~~~~~~~~~

Connect a compatible USB camera to the RZ/V2H RDK board before running the application.

Then log in to the target:

.. code-block:: bash

   ssh ubuntu@192.168.0.10

Load the ROS 2 environment:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source /home/ubuntu/ros2_ws/install/setup.bash

Run the virtual hand example:

.. code-block:: bash

   ros2 launch rzv_demo_dexhand demo_virtual_inspire_rh56_hands.launch.py

If you want to use the Ruiyan RH2 virtual hand example instead:

.. code-block:: bash

   ros2 launch rzv_demo_dexhand demo_virtual_ruiyan_rh2_hands.launch.py

If you want to run a physical hand example:

.. code-block:: bash

   # Inspire RH56
   ros2 launch rzv_demo_dexhand demo_physical_inspire_rh56_hand.launch.py

.. code-block:: bash

   # Ruiyan RH2
   ros2 launch rzv_demo_dexhand demo_physical_ruiyan_rh2_hand.launch.py

.. note::

   For Ruiyan RH2 physical hand control, initialize the hand first if required by your setup.

   Example:

   .. code-block:: bash

      /home/ubuntu/ros2_ws/install/ruiyan_rh2_dexhand/share/ruiyan_rh2_dexhand/setup/ruiyan_rh2_init.sh

Run Remotely from VS Code
~~~~~~~~~~~~~~~~~~~~~~~~~

Instead of manually logging in and typing commands, you can use the provided VS Code tasks.

For the Dexterous Hand example, set the following in ``settings.json``:

.. code-block:: json

   {
     "TARGET_IP": "192.168.0.10",
     "LAUNCH_PACKAGE_NAME": "rzv_demo_dexhand",
     "LAUNCH_FILE_NAME": "demo_virtual_inspire_rh56_hands.launch.py"
   }

Then run the task:

- **ROS2: Launch Package LaunchFile**

This is equivalent to running:

.. code-block:: bash

   ros2 launch rzv_demo_dexhand demo_virtual_inspire_rh56_hands.launch.py

on the target device.

.. tip::

   This method is useful when you want to launch the application from the host development environment without manually logging in to the target device.

Step 7: Verify That the Application Is Running
"""""""""""""""""""""""""""""""""""""""""""""""

After launching the application on the target device, verify that ROS 2 nodes are running.

In another terminal on the target:

.. code-block:: bash

   source /opt/ros/jazzy/setup.bash
   source /home/ubuntu/ros2_ws/install/setup.bash
   ros2 node list
   ros2 topic list

This step is useful before remote debugging because it helps identify which executable you may want to debug.

Step 8: Prepare for Remote Debugging
"""""""""""""""""""""""""""""""""""""

To debug a node inside the launch file, you need to identify the specific executable to debug.

For launch-based debugging, the following variables are required in ``settings.json``:

.. code-block:: json

   {
     "TARGET_IP": "192.168.0.10",
     "LAUNCH_PACKAGE_NAME": "rzv_demo_dexhand",
     "LAUNCH_FILE_NAME": "demo_virtual_inspire_rh56_hands.launch.py",
     "NODE_PACKAGE_NAME": "<package_of_target_node>",
     "NODE_EXECUTABLE_NAME": "<target_executable>"
   }

Replace:

- ``<package_of_target_node>`` with the package that contains the executable you want to debug
- ``<target_executable>`` with the executable name filtered by the debug launch workflow

If you want to debug a node started directly by ``ros2 run``, only these are needed:

.. code-block:: json

   {
     "TARGET_IP": "192.168.0.10",
     "NODE_PACKAGE_NAME": "<package_of_target_node>",
     "NODE_EXECUTABLE_NAME": "<target_executable>"
   }

Step 9: Start Remote Debugging from VS Code
""""""""""""""""""""""""""""""""""""""""""""

For a node started with ``ros2 run``:

#. Open the Run and Debug view in VS Code with ``Ctrl+Shift+D``.
#. Select **GDB for ROS2 Run**.
#. Start debugging.

For a node started through the Dexterous Hand launch file:

#. Open the Run and Debug view in VS Code with ``Ctrl+Shift+D``.
#. Select **GDB for ROS2 Launch**.
#. Start debugging.

The corresponding target-side command is similar to:

.. code-block:: bash

   ros2 launch --launch-prefix 'gdbserver localhost:<debug_port>' --launch-prefix-filter '<executable_name>' rzv_demo_dexhand demo_virtual_inspire_rh56_hands.launch.py

.. note::

   The current debug task **already performs a debug build** before starting the session, so you do not need to run the build step manually.

Step 10: Typical Day-to-Day Workflow
"""""""""""""""""""""""""""""""""""""

During development, the most common workflow is:

#. Modify source files under ``src/``.
#. Install the required dependencies into the sysroot:

   .. code-block:: bash

      sysroot-rosdep-install

#. Build the workspace:

   .. code-block:: bash

      cross-colcon-build

#. Deploy the updated ``install`` directory using VS Code tasks or the Task Buttons extension.

#. Install the dependencies on the target board.

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      rosdep install --from-paths /home/ubuntu/ros2_ws/install/*/share -y -r --ignore-src

#. Run the application on the target, or start a remote run or debug session from VS Code.

   .. code-block:: bash

      ssh ubuntu@192.168.0.10
      source /opt/ros/jazzy/setup.bash
      source /home/ubuntu/ros2_ws/install/setup.bash
      ros2 launch rzv_demo_dexhand demo_virtual_inspire_rh56_hands.launch.py

#. If needed, start a remote debugging session from VS Code.

Troubleshooting
"""""""""""""""

#. Build succeeds but the target still runs old code

   Make sure the updated ``install`` directory **has been deployed again** after rebuilding.

   Check the timestamp on the target:

   .. code-block:: bash

      ls -l /home/ubuntu/ros2_ws/install

#. Launch file runs but the debug session does not attach

   Check that ``NODE_EXECUTABLE_NAME`` matches the exact executable name used by the node you want to debug.

   Also check that ``gdbserver`` is installed on the target:

   .. code-block:: bash

      which gdbserver

#. Dependencies are missing on the target

   Install them on the target:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      rosdep install --from-paths /home/ubuntu/ros2_ws/install/*/share -y -r --ignore-src

#. Build fails because sysroot dependencies are missing

   Make sure you ran ``sysroot-rosdep-install`` from the workspace before starting the cross-build.

   Example:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      sysroot-rosdep-install

#. The application does not respond correctly to hand motion

   Check the USB camera connection and make sure the hand orientation matches the expected setup.

   The common setup uses a fixed camera pointing upward toward the hand so that the hand appears from bottom to top in the image.

Summary
"""""""

In this tutorial, you used the :ref:`Vision Based Dexterous Hand <dexhand>` application to go through the complete ROS 2 workflow:

- install the required dependencies into the sysroot,
- cross-build in the development container,
- deploy to the target device,
- run on the target device, and
- debug remotely from VS Code.

After you are comfortable with this example, you can reuse the same workflow for other ROS 2 applications by changing only the package, executable, and launch file names.