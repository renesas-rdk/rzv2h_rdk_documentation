.. _cross_build_usage:

Cross compilation Usage Guide
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section describes how to use the cross-compilation commands inside the Docker container.

Prepare your workspace
""""""""""""""""""""""

If you are **cross-building any ROS 2 applications**, ensure that the ROS 2 workspace is fully set up with all necessary dependency packages.

Make sure to include all required ROS 2 packages in the ``src/`` directory of your workspace.

It is required to have the correct folder structure for the ROS 2 workspace to ensure that the cross-compilation process can locate all necessary files and dependencies.

The expected folder structure for the ROS 2 workspace is as follows:

.. code-block:: text
   :emphasize-lines: 2-5

   ros2_ws/
   ├── src/                  # Source code for ROS 2 packages
   │   ├── package_1/
   │   ├── package_2/
   │   └── ...
   ├── build/                # Build output directory (generated)
   ├── install/              # Installation directory (generated)
   └── log/                  # Log files (generated)

rzv2h-chroot
""""""""""""

.. tip::

   See the ``/usr/local/bin/rzv2h-chroot`` script in the Docker container for the implementation of the ``rzv2h-chroot`` command and for details on how it sets up the environment for running commands inside the ARM64 chroot.

``rzv2h-chroot`` is a wrapper around the standard ``chroot`` command. It allows you to run commands inside the ARM64 chroot environment with the required emulation and environment variable setup.

The basic usage is as follows:

.. code-block:: bash

   # Enter an interactive shell
   rzv2h-chroot

   # Run a specific command
   rzv2h-chroot apt update

   # Execute a command with arguments
   rzv2h-chroot bash -c "command"

**Use cases:**

- Install packages into the sysroot:

  .. code-block:: bash

     rzv2h-chroot apt-get update
     rzv2h-chroot apt-get install build-essential

- Check installed packages:

  .. code-block:: bash

     rzv2h-chroot dpkg -l

- Run a script inside the chroot:

  .. code-block:: bash

     # Copy the script into the sysroot first
     cp /path/to/your/script.sh $V2H_SYSROOT/path/to/your/script.sh

     # Run it inside the chroot
     rzv2h-chroot /bin/bash -c "source /path/to/your/script.sh"

.. important::

   #. The script already runs with ``sudo`` privileges. Do not add ``sudo`` to commands executed with ``rzv2h-chroot``. Otherwise, errors may occur.

      For example, the following command causes an error, while the second command shows the correct usage:

      .. code-block:: bash

         # This causes an error
         rzv2h-chroot sudo apt-get update

         # Correct usage without sudo
         rzv2h-chroot apt-get update

   #. If you need to run multiple commands, it is usually more efficient to enter an interactive shell with ``rzv2h-chroot`` and run the commands directly inside the chroot environment.

.. note::

   Be aware that there are two filesystem contexts inside Docker:

   #. The Docker container itself (**AMD64** architecture)
   #. The chroot environment (**ARM64** architecture)

   When using ``rzv2h-chroot <command>``, the command runs inside the ARM64 chroot environment. File paths from the Docker container are not directly accessible. You must copy files into the sysroot (``$V2H_SYSROOT``) before referencing them inside the chroot.

sysroot-rosdep-install
""""""""""""""""""""""

.. tip::

   See the ``/usr/local/bin/sysroot-rosdep-install`` script in the Docker container for the implementation of the ``sysroot-rosdep-install`` command and for details on how it uses rosdep to install dependencies into the sysroot.

``sysroot-rosdep-install`` installs ROS 2 dependencies into the sysroot using rosdep.

.. code-block:: bash

   # Use the default workspace from $ROS2_WS
   sysroot-rosdep-install

   # Specify a custom workspace path
   sysroot-rosdep-install /path/to/custom/ros2_ws

The input for this command is the path to your ROS 2 workspace, which must contain a ``src/`` directory with your ROS 2 packages. If no path is provided, the command uses the workspace defined by the ``ROS2_WS`` environment variable by default.

The command performs the following steps:

#. Validates the ROS 2 workspace structure (a ``src/`` directory is required).
#. Syncs the workspace source code to the sysroot (``/home/ubuntu/ros2_ws``).
#. Enters the chroot environment with ARM64 emulation.
#. Installs build-time dependencies using rosdep.
#. Prepares the sysroot for cross-compilation.

.. important::

   - Run this command whenever you add new packages to your ROS 2 workspace.
   - Make sure the library versions in the sysroot match those in the RZ/V2H RDK Linux image. Otherwise, runtime errors may occur on the target device.

cross-colcon-build
""""""""""""""""""

.. tip::

   See the end of the ``~/.bashrc`` file in the Docker container for the implementation of ``cross-colcon-build`` and for details on how it sets up the CMake toolchain file and other arguments.

For ROS 2 development, the ``colcon build`` command is commonly used to build packages. For cross-compilation, use ``cross-colcon-build`` instead. This command is a wrapper around ``colcon build`` with the required configuration for cross-compiling to ARM64.

.. code-block:: bash

   # Build all packages
   cross-colcon-build

   # Build a specific package and its dependencies
   cross-colcon-build --packages-up-to <name-of-pkg>

You can pass any additional arguments supported by ``colcon build`` directly to ``cross-colcon-build``.

.. note::

   If you provide CMake arguments through ``--cmake-args``, they are ignored because ``cross-colcon-build`` already includes the required CMake arguments for cross-compilation.

If you need to add extra CMake arguments, edit the ``~/.bashrc`` file in the Docker container. Locate the following section at the end of the file:

.. code-block:: bash

   # Get build type at runtime, not at function definition time
   local build_type="${CMAKE_BUILD_TYPE:-Release}"

   colcon build \
       $@ \
       --cmake-force-configure \
       --cmake-args \
           -DCMAKE_TOOLCHAIN_FILE=/home/ubuntu/toolchains/cross.cmake \
           -DPython3_EXECUTABLE=/usr/bin/python3 \
           -DPython3_ROOT_DIR=/usr \
           -DPython3_FIND_STRATEGY=LOCATION \
           -DCMAKE_BUILD_TYPE=${build_type}

Add your custom CMake arguments to the ``--cmake-args`` list.

For example, to enable verbose Makefile output and set a custom installation prefix:

.. code-block:: bash
   :emphasize-lines: 9

   # Get build type at runtime, not at function definition time
   local build_type="${CMAKE_BUILD_TYPE:-Release}"

   colcon build \
       $@ \
       --cmake-force-configure \
       --cmake-args \
           -DCMAKE_TOOLCHAIN_FILE=/home/ubuntu/toolchains/cross.cmake \
           -DCMAKE_VERBOSE_MAKEFILE=ON \
           -DPython3_EXECUTABLE=/usr/bin/python3 \
           -DPython3_ROOT_DIR=/usr \
           -DPython3_FIND_STRATEGY=LOCATION \
           -DCMAKE_BUILD_TYPE=${build_type}

Then save the file and source it again to apply the changes:

.. code-block:: bash

   source ~/.bashrc

Run ``cross-colcon-build`` again to apply the updated CMake arguments.

Development Workflow
""""""""""""""""""""

#. **Set up your ROS 2 workspace** on the host machine and mount it into the Docker container.
#. **Install dependencies** using ``sysroot-rosdep-install`` to ensure that all required libraries are available in the sysroot.
#. **Optional:** Use ``rzv2h-chroot`` to enter the chroot environment and run additional commands.
#. **Build your packages** using ``cross-colcon-build``.
#. **Deploy the built applications** to the RZ/V2H RDK. Follow the next section for deployment instructions.