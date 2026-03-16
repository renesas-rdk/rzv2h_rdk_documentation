ROS 2 Application Remote Debugging
----------------------------------

This section describes how to remotely debug ROS 2 applications on the Renesas RZ/V2H RDK platform by using GDB.

It covers the required setup, the debugging workflow, and practical notes for running a remote debugging session from VS Code.

.. note::

   Before proceeding, make sure you have completed the previous sections for cross-compilation, deployment, and VS Code workspace configuration.

Prerequisites
^^^^^^^^^^^^^

#. Complete the :ref:`Cross compilation environment setup <requirements_ros2_cross_build>` section.

   Make sure VS Code is configured with the required extensions and workspace settings for ROS 2 development, and that the application has been successfully deployed to the RZ/V2H RDK platform.

#. Make sure the required :ref:`variables <workspace_settings>` in ``settings.json`` are configured correctly, especially: ``TARGET_IP``

#. Ensure that all runtime dependencies required by the application are installed on the target device.

#. Install ``gdbserver`` on the RZ/V2H RDK platform if it is not already available:

   .. code-block:: bash

      sudo apt-get update
      sudo apt-get install gdbserver

Debugging Workflow
^^^^^^^^^^^^^^^^^^

#. Before starting the debugging workflow, open ``settings.json`` and make sure the required variables are configured correctly for the mode you want to use.

   - For debugging with ``ros2 run``, set:

     - ``TARGET_IP``
     - ``NODE_PACKAGE_NAME``
     - ``NODE_EXECUTABLE_NAME``

   - For debugging with ``ros2 launch``, set:

     - ``TARGET_IP``
     - ``LAUNCH_PACKAGE_NAME``
     - ``LAUNCH_FILE_NAME``
     - ``NODE_PACKAGE_NAME``
     - ``NODE_EXECUTABLE_NAME``

#. Open the Run and Debug view in VS Code:

   - Press ``Ctrl+Shift+D``.
   - Select the appropriate debug configuration:

     - **GDB for ROS2 Run** to debug an application started with ``ros2 run``
     - **GDB for ROS2 Launch** to debug an application started with ``ros2 launch``

   - If prompted, enter any required runtime arguments, such as ROS 2 parameters.

#. Start the corresponding VS Code task to begin the remote debugging session.

   - For **ROS2: Debug Run (GDB)**, the following command is executed on the target device:

     .. code-block:: bash

        ros2 run --prefix 'gdbserver localhost:<debug_port>' <package_name> <executable_name>

   - For **ROS2: Debug Launch (GDB)**, the following command is executed on the target device:

     .. code-block:: bash

        ros2 launch --launch-prefix 'gdbserver localhost:<debug_port>' --launch-prefix-filter '<executable_name>' <launch_package_name> <launch_file_name>

.. note::

   The current debug task already performs a debug build before starting the session. Therefore, you do not need to run the build step manually.