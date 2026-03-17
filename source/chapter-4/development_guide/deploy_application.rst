.. _ros2_deployment:

ROS 2 Application Deployment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section provides instructions for deploying ROS 2 applications to the Renesas RZ/V2H RDK platform.

Prerequisites
"""""""""""""

Before starting deployment:

#. Complete the :ref:`Cross compilation environment setup <requirements_ros2_cross_build>` section.
#. Prepare the VS Code workspace configuration by following :ref:`ROS 2 VS Code Workspace Configuration <ros2_vscode_workspace>`.
#. Make sure the required :ref:`variables <workspace_settings>` in ``settings.json`` are configured correctly, especially ``TARGET_IP``.
#. Make sure all required packages are available in the current workspace on the host machine.

Install Dependencies into the Sysroot
""""""""""""""""""""""""""""""""""""""

Before building the application, make sure that all required dependencies are installed into the target sysroot.

Prepare the necessary ROS 2 packages in the current workspace on the host machine, then run the following command to install the required dependencies into the sysroot:

.. code-block:: bash

   sysroot-rosdep-install

.. _build_deploy:

Build the Application
"""""""""""""""""""""

Before deploying, ensure that your ROS 2 application is built using the cross-compilation environment.

There are three ways to build your application:

#. Use the command line:

   .. code-block:: bash

      cross-colcon-build

   This command builds the ROS 2 workspace using the cross-compilation toolchain with the default build configuration (``Release``).

#. Use a VS Code task:

   - Press ``Ctrl+Shift+P`` to open the Command Palette.
   - Run **Tasks: Run Task**.
   - Choose the appropriate build task:

     - **ROS2: Build Release** to build with the ``Release`` configuration
     - **ROS2: Build Debug** to build with the ``Debug`` configuration

#. Use the default build task:

   - Press ``Ctrl+Shift+B``.
   - This executes the default build task, which is configured to build with the ``Release`` configuration.

Deploy to Target
""""""""""""""""

There are two main methods to deploy your application to the RZ/V2H RDK platform:

#. Use a VS Code task:

   - Press ``Ctrl+Shift+P`` to open the Command Palette.
   - Run **Tasks: Run Task**.
   - Choose **ROS2: Deploy to Target** to execute the deployment script.

#. Use the **Task Buttons** extension:

   - Click the **Deploy** button in the VS Code status bar to start the deployment process.

Install Dependencies on the Target
"""""""""""""""""""""""""""""""""""

After deployment, install any additional dependencies on the target device.

#. Change to the workspace directory on the target device:

   .. code-block:: bash

      cd <path_to_your_ros2_ws>

   Replace ``<path_to_your_ros2_ws>`` with the actual path to your ROS 2 workspace on the RZ/V2H RDK.

#. Set up the ROS 2 environment and install dependencies using ``rosdep``:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      rosdep install --from-paths ./install/*/share -y -r --ignore-src

Run the Application
"""""""""""""""""""

You can start your application on the target device by using either ``ros2 run`` or ``ros2 launch`` through the VS Code tasks.

.. note::

   Before running the application, make sure all required dependencies are installed on the target device. Otherwise, the application may fail to start.

#. Update the following variables in ``settings.json`` to match the mode you want to use:

   - For ``ros2 run``:

     - ``NODE_PACKAGE_NAME``
     - ``NODE_EXECUTABLE_NAME``

   - For ``ros2 launch``:

     - ``LAUNCH_PACKAGE_NAME``
     - ``LAUNCH_FILE_NAME``

#. Use one of the following methods to run the application:

   - Method 1: Use the Command Palette

     - Press ``Ctrl+Shift+P`` to open the Command Palette.
     - Run **Tasks: Run Task**.
     - Choose the appropriate task:

       - **ROS2: Run Package Executable** to run using ``ros2 run``
       - **ROS2: Launch Package LaunchFile** to run using ``ros2 launch``

   - Method 2: Use the **Task Buttons** extension

     - Click **Run ExecutableFile** in the VS Code status bar to run using ``ros2 run``.
     - Click **Run LaunchFile** in the VS Code status bar to run using ``ros2 launch``.

#. Enter any custom arguments if prompted.

   For example, if your launch file requires a custom argument such as ``video_device``, enter it in the following format:

   .. code-block:: bash

      video_device:=/dev/video0

#. The application starts on the target device.