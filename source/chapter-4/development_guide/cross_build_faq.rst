Cross-compilation FAQ
---------------------

General
^^^^^^^

#. What is the difference between ``cross-colcon-build`` and ``colcon build``?

   ``cross-colcon-build`` is a wrapper around ``colcon build`` that automatically sets the CMake toolchain file and other arguments required for cross-compilation to ARM64.

   Do **not** use ``colcon build`` directly inside the Docker container.
   It would attempt to build for the host architecture, typically AMD64, and the resulting binaries would not run on the RZ/V2H RDK board.

   Always use ``cross-colcon-build`` for building applications that will run on the target device.

#. Why does ``cross-colcon-build`` try to build non-ROS 2 packages for my workspace?

   This is normal behavior.

   ``cross-colcon-build`` builds all packages in the workspace that colcon detects, including non-ROS 2 packages.

   If you want to build only specific ROS 2 packages, use options such as:

   - ``--packages-select``
   - ``--packages-up-to``

   For example:

   .. code-block:: bash

      cross-colcon-build --packages-select <package_name>

   Alternatively, if a directory should always be ignored by colcon, add a ``COLCON_IGNORE`` file in that package directory.

#. Why do I see a warning about ``AMENT_PREFIX_PATH`` during cross-build?

   During cross-build, you may see a warning like this:

   .. code-block:: console

      [0.233s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/opt/rzv2h-sysroot/opt/ros/jazzy' in the environment variable AMENT_PREFIX_PATH doesn't contain any 'local_setup.*' files.

   This warning is expected and does not indicate a problem with your setup.

   The warning appears because the ROS 2 setup files are intentionally removed from the sysroot.
   This prevents hardcoded paths from being embedded in the build output, which would otherwise break execution on the target board.

   As long as your application runs correctly on the RZ/V2H RDK device, you can safely ignore this warning.

   Without this adjustment, you may see an error like the following on the target board:

   .. code-block:: console

      ubuntu@ubuntu:~$ source install/setup.bash
      not found: "/opt/rzv2h-sysroot/opt/ros/jazzy/local_setup.bash"
      ubuntu@ubuntu:~$

Build and Dependencies
^^^^^^^^^^^^^^^^^^^^^^

#. How do I rebuild only one package after modifying its source code?

   Use the ``--packages-select`` option to build only the modified package:

   .. code-block:: bash

      cross-colcon-build --packages-select <package_name>

   If the modified package has downstream dependents that also need rebuilding, use ``--packages-above`` instead:

   .. code-block:: bash

      cross-colcon-build --packages-above <package_name>

   This is significantly faster than rebuilding the entire workspace.

#. When do I need to run ``sysroot-rosdep-install`` again?

   You need to run ``sysroot-rosdep-install`` again when:

   - you add a **new package** to the ``src/`` directory,
   - you add or change a **dependency** in a package's ``package.xml``, or
   - the sysroot is missing libraries that the build requires.

   If you only modify source code, such as C++, Python, or launch files, without changing dependencies, you do **not** need to run ``sysroot-rosdep-install`` again.
   Just run ``cross-colcon-build``.

#. Why is ``sysroot-rosdep-install`` slow?

   This is expected.

   ``sysroot-rosdep-install`` installs packages into the ARM64 sysroot, and the installation process depends on ``apt``.
   Package installation speed depends on:

   - the number of required packages,
   - the number of dependency packages, and
   - network and disk performance.

   Also, ``apt`` does not significantly parallelize package installation in this workflow, so installing many dependencies may take time.

   If the command completes successfully, the slower speed does not indicate an error.

#. The build fails with a "package not found" error in CMake. What should I do?

   This typically means that a required dependency is not installed in the sysroot.

   Example error:

   .. code-block:: text

      CMake Error at CMakeLists.txt:10 (find_package):
        By not providing "FindSomePackage.cmake" in CMAKE_MODULE_PATH this
        project has asked CMake to find a package configuration file provided by
        "SomePackage", but CMake did not find one.

   To fix this:

   #. Make sure the dependency is listed in the package's ``package.xml``.
   #. Run ``sysroot-rosdep-install`` to install the missing dependency into the sysroot.
   #. Run ``cross-colcon-build`` again.

   If the dependency is not available through ``rosdep``, you may need to install it manually into the sysroot using ``rzv2h-chroot``:

   .. code-block:: bash

      rzv2h-chroot apt-get install <package_name>

#. The sysroot uses Renesas OpenCVA, not the default OpenCV from the apt repository. What should I know?

   The sysroot in the cross-compilation Docker image includes **Renesas OpenCVA**, a hardware-accelerated version of OpenCV optimized for the RZ/V2H platform.
   This is **not** the same as the standard ``libopencv-dev`` package available from the Ubuntu apt repository.

   Key differences:

   - Renesas OpenCVA is built with hardware acceleration support specific to the RZ/V2H DRP-AI and video pipeline.
   - The standard ``libopencv-dev`` from apt does not include these hardware-specific optimizations.

   .. warning::

      Do **not** install ``libopencv-dev`` from the apt repository into the sysroot.
      Running ``apt-get install libopencv-dev`` inside ``rzv2h-chroot`` will overwrite the Renesas OpenCVA libraries and break hardware acceleration on the target board.

   If your package depends on OpenCV, it will automatically link against the Renesas OpenCVA libraries already present in the sysroot.
   No additional installation is required.

   If you accidentally overwrite the OpenCV libraries, recreate the sysroot by creating a new Docker container from the original image.

Sysroot and Docker Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#. How do I restart a Docker container that was stopped?

   Use ``docker start`` to restart an existing container, then use ``docker exec`` to open a shell:

   .. code-block:: bash

      docker start ros2_cross_build_container
      docker exec -it ros2_cross_build_container bash

   Do **not** use ``docker run`` again with the same image.
   ``docker run`` creates a **new** container, and any files you created or modified inside the previous container, outside mounted volumes, will not be available in the new one.

   If you accidentally created a new container, you can still access the old one by starting it with its container name or ID.
   Use ``docker ps -a`` to list all containers.

#. Will I lose my data if the Docker container is stopped or restarted?

   It depends on where the data is stored:

   - Files inside the **mounted volume** (``$ROS2_WS``, which maps to ``/home/ubuntu/ros2_ws`` inside the container) are stored on the host machine.
     They are preserved across container stops, restarts, and even container removal.
   - Files stored **outside** the mounted volume but still inside the container, for example, changes to ``~/.bashrc``, files under ``/home/ubuntu/toolchains/``, or packages installed with ``apt``, are preserved across ``docker stop`` and ``docker start``.
     However, they are **lost** if the container is removed with ``docker rm``.

   To avoid losing important changes, keep your source code and configuration files inside the mounted workspace directory.

#. Can I run multiple cross-build Docker containers at the same time?

   You can create and run multiple Docker containers from the same image.
   However, each container has its own independent sysroot and chroot environment.

   Keep in mind that only **one chroot instance** can run at a time **within a single container**.
   If you need to run ``rzv2h-chroot`` or ``sysroot-rosdep-install`` in parallel, use separate containers.

   Also, if multiple containers mount the same host directory (``$ROS2_WS``), concurrent builds may cause file conflicts.
   It is recommended to use a separate workspace directory for each container.

#. How do I avoid library version mismatches between the sysroot and the board?

   The sysroot used for cross-compilation must match the Linux image running on the RZ/V2H RDK board.

   If the board's Linux image is updated, for example, after a firmware update or OS upgrade, the sysroot inside the Docker container may become outdated.
   This can cause runtime errors such as missing symbols or incompatible shared libraries.

   .. warning::

      **ABI mismatch** can occur when ROS 2 packages on the board are updated, for example, through ``apt upgrade``, but the sysroot used for cross-compilation is not updated accordingly.

      In this case, the application was cross-compiled against older library versions in the sysroot, but at runtime it links against newer libraries on the board.
      This mismatch can cause:

      - **Segmentation faults** or crashes at startup.
      - **Undefined symbol** errors when the application tries to call functions that have changed or been removed.
      - **Silent data corruption** if data structures have changed size or layout between versions.

      Example error:

      .. code-block:: text

         symbol lookup error: /home/ubuntu/ros2_ws/install/<package_name>/lib/<package_name>/<executable_name>: undefined symbol: _ZN8rclcpp13...

      To prevent ABI mismatch, always keep the sysroot and the board's Linux image in sync.

   **How to keep the sysroot and the board in sync:**

   - For **ROS 2 apt package updates**, install the same package version on both the board and the sysroot.

     For example, if you updated ``ros-jazzy-rclcpp`` on the board, update the same package in the sysroot:

     On the board:

     .. code-block:: bash

        sudo apt update
        sudo apt install ros-jazzy-rclcpp

     On the sysroot (inside the Docker container):

     .. code-block:: bash

        rzv2h-chroot apt update
        rzv2h-chroot apt install ros-jazzy-rclcpp

     Only update the specific packages that were changed.
     Running ``apt upgrade`` on the entire sysroot is not recommended as it can be time-consuming and may unintentionally update other packages.

   - For **Linux image updates** such as firmware updates or OS upgrades, the sysroot cannot be updated with ``apt`` alone.
     In this case:

     - Pull the latest Docker image that matches the board's Linux image version:

       .. code-block:: bash

          docker pull ghcr.io/renesas-rdk/rzv2h_ubuntu_xbuild:latest

     - Create a new container from the updated image.

   After updating the sysroot by either method, rebuild and redeploy your application:

   .. code-block:: bash

      cross-colcon-build

Deployment and Runtime
^^^^^^^^^^^^^^^^^^^^^^

#. I deployed successfully, but the application still runs the old code. Why?

   This usually means that the ``install`` directory was not deployed again after the last rebuild.

   After modifying source code, the correct workflow is:

   #. Rebuild with ``cross-colcon-build``.
   #. Deploy again using the VS Code **ROS2: Deploy to Target** task or manually with ``scp``.
   #. Restart the application on the target device.

   You can verify the deployment by checking the file timestamps on the target:

   .. code-block:: bash

      ls -l /home/ubuntu/ros2_ws/install/<package_name>/lib/<package_name>/

#. ``rosdep install`` on the board reports "package not found". What should I do?

   This can happen for several reasons:

   - Try with ``sudo apt update`` command and then run ``rosdep install`` again.
   - The board does not have **internet access**.
     ``rosdep install`` needs to download packages from the Ubuntu and ROS 2 repositories.
     Make sure the board is connected to the internet and can reach ``packages.ros.org``.
   - The package is a **build-time-only** dependency that is not available in the target repositories.
     These packages are needed in the sysroot during cross-compilation but do not need to be installed on the board.
     You can safely ignore these errors if the application runs correctly.
   - The ``rosdep`` database is outdated.
     Run ``rosdep update`` on the board before retrying.

Debugging
^^^^^^^^^

#. ``gdbserver`` is not installed on the board. How do I install it?

   Install ``gdbserver`` on the RZ/V2H RDK board:

   .. code-block:: bash

      sudo apt-get update
      sudo apt-get install gdbserver

   Verify the installation:

   .. code-block:: bash

      which gdbserver

   ``gdbserver`` is required for remote debugging from VS Code.
   Without it, the debug tasks will fail to start.

#. The debug session does not attach to the target. What should I check?

   Check the following:

   - **``NODE_EXECUTABLE_NAME``** in ``settings.json`` must match the exact name of the executable you want to debug.
     If the name does not match, ``gdbserver`` will not attach to the correct process.

     You can verify the executable name by listing running processes on the target:

     .. code-block:: bash

        ros2 node list
        ps aux | grep ros

   - **Port conflict**: the default GDB port, defined by ``TARGET_GDB_PORT`` in ``settings.json``, may already be in use.
     Check whether another ``gdbserver`` instance is still running on the target:

     .. code-block:: bash

        ps aux | grep gdbserver

     Kill any leftover ``gdbserver`` processes before starting a new debug session:

     .. code-block:: bash

        killall gdbserver

   - **Network connectivity**: make sure the host PC can reach the target on the debug port.

     .. code-block:: bash

        ping <TARGET_IP>

   - **Build type**: check the debug symbol from the build output binaries on the target to confirm that you built with the ``Debug`` configuration.

     You can check the build type by running:

     .. code-block:: bash

        file /home/ubuntu/ros2_ws/install/<package_name>/lib/<package_name>/<executable_name>

     The output for a **Debug** build should contain ``with debug_info, not stripped``, for example:

     .. code-block:: text

        <executable_name>: ELF 64-bit LSB pie executable, ARM aarch64, version 1 (GNU/Linux), dynamically linked, interpreter /lib/ld-linux-aarch64.so.1, ..., with debug_info, not stripped

     Alternatively, you can use ``readelf`` for a more definitive check:

     .. code-block:: bash

        readelf --debug-dump=info /home/ubuntu/ros2_ws/install/<package_name>/lib/<package_name>/<executable_name> | head -20

     If the output shows ``.debug_info`` section content, the binary was built with debug symbols.