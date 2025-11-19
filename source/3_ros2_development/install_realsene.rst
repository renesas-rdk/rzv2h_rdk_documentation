Intel RealSense Camera Installation
-------------------------------------

This section provides instructions for installing and configuring Intel RealSense cameras on the Renesas RZ/V2H RDK board for ROS2 applications.

**Why should we use cross-build instead of native-build?**

While we can build `librealsense2` and `realsense-ros` native, building `librealsense2` this way takes a significant amount of time. By using cross-build, we can reduce both the build time and the deployment time to the target machine.

However, cross-build has limitations when it comes to installing or uninstalling related software. Please follow this guide carefully to learn how to use it correctly.

For more detailed information, please refer to the official `RealSense ROS <https://github.com/IntelRealSense/realsense-ros>`_ and `librealsense <https://github.com/IntelRealSense/librealsense>`_ documentation.

For more information about building native, please refer to the `Install librealsense2 documentation <https://github.com/IntelRealSense/librealsense/blob/development/doc/installation.md#install-libreal>`_

When perform navtive build, follow the official documentation, starting from the **Install librealsense2** step and skip **3. Build and apply patched kernel modules for**, as we have already applied these patches in the kernel image by do the next step.

Prerequisites
^^^^^^^^^^^^^^

Before proceeding:

- Ensure that you have a working cross-compilation environment set up as described in the :ref:`Common docker environment setup <docker_sdk_setup>` section.
- Ensure that you have set up the eSDK environment as described in the :ref:`eSDK Setup <esdk_setup>`  section.
- Installing the required dependencies for building `librealsense2` inside the cross-compilation Docker container:

.. code-block:: bash

   $ sudo apt-get install -y ccache

Cross-compile and Install librealsense2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build the `librealsense2` library inside the cross-compilation Docker container:

1. Start the cross-compilation Docker container as described in the :ref:`Common docker environment setup <docker_sdk_setup>` section.
2. Inside the Docker container, clone the `librealsense` repository:

   .. code-block:: bash

      $ git clone https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/intel-realsense-camera/librealsense.git --depth 1 --single-branch -b rzv_ros2

3. Navigate to the `librealsense` directory:

    .. code-block:: bash

        $ cd librealsense

4. Create a build directory and navigate into it:

    .. code-block:: bash

        $ mkdir build && cd build

5. Configure the build using CMake with the appropriate toolchain file:

    .. code-block:: bash

         $ cmake .. -DCMAKE_TOOLCHAIN_FILE=~/toolchains/cross.cmake \
                    -DBUILD_EXAMPLES=OFF \
                    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
                    -DBUILD_UNIT_TESTS=OFF \
                    -DCMAKE_BUILD_TYPE=Release \
                    -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
                    -DENABLE_PRECOMPILED_HEADERS=OFF \
                    -DCMAKE_INSTALL_PREFIX=./tmp-install

6. Build and install the library:

    .. code-block:: bash

        $ make -j$(nproc)
        $ sudo make install

7. Copy the installed files to the Yocto SDK sysroot:

    .. code-block:: bash

        $ sudo cp -r ./tmp-install/* $ROS2_SDK_SYSROOT/usr/

8. Deploy the `librealsense2` library to the RZ/V2H RDK board:

    Compress the `tmp-install` directory:

    .. code-block:: bash

         $ sudo tar -cf librealsense.tar.bz2 -C ./tmp-install/ .

    Copy the ``librealsense.tar.bz2`` file to  the RZ/V2H RDK board using `scp` or any other file transfer method.

    On the RZ/V2H RDK board, extract the files to the root directory:

    .. code-block:: bash

         $ mkdir -p ~/librealsense_install
         $ tar -xvf librealsense.tar.bz2 -C ~/librealsense_install/
         $ sudo cp -r ~/librealsense_install/* /usr/

Setup udev rules for Intel RealSense Cameras
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install rules so the device is accessible without root:

.. code-block:: bash

    $ git clone https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/intel-realsense-camera/librealsense.git --depth 1 --single-branch -b rzv_ros2

    $ cd librealsense

    # Running the udev rules setup script
    $ sudo ./scripts/setup_udev_rules.sh

Verify device on the target board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Connect the Intel RealSense camera to the RZ/V2H RDK board via USB before running the following commands.

.. code-block:: bash

    # Check connected RealSense devices
    $ rs-enumerate-devices
    # or
    $ rs-fw-update -l   # list devices & firmware info

If nothing appears, recheck cables/ports (prefer USB 3.x), udev rules, and power.

Cross-compile and Install realsense-ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build the `realsense-ros` package inside the cross-compilation Docker container:

1. Start the cross-compilation Docker container as described in the :ref:`Common docker environment setup <docker_sdk_setup>` section.
2. Inside the Docker container, clone the `realsense-ros` repository into your ROS2 workspace `src` directory:

.. code-block:: bash

   $ cd ~/ros2_ws/src
   $ git clone https://partnergitlab.renesas.solutions/sst1/industrial/ws078/rzv_ros_package/intel-realsense-camera/realsense_ros.git --depth 1 --single-branch -b rzv_ros2

3. Navigate to your ROS2 workspace directory and cross-compile the workspace using the Yocto SDK as described in the :ref:`cross-build the ROS2 Application using Yocto SDK <requirements_ros2_cross_build>` section.
4. Deploy the `install` directory to the RZ/V2H RDK board using :ref:`Deploying the ROS2 Application <ros2_deployment>` or using the `scp` command.
5. Install the required dependencies on the RZ/V2H RDK board:

.. code-block:: bash

   $ rosdep install --from-paths <path/to>install/*/share -y -r --ignore-src

**Warning (safe and can ignore):**

The following warning will be displayed when running the above `cross-colcon-build` command. This occurs because, during cross-compilation, the build process tries to detect the camera using the `rs-enumerate-devices` command.

We have already verified this command in a previous step and confirmed that it works.

.. code-block:: text

    --- stderr: realsense2_camera
    bash: line 1: rs-enumerate-devices: command not found
    ---

(Optional) Compressed Image Transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you want to publish or subscribe to compressed image topics (e.g., for bandwidth optimization),
please install the following packages before running the RealSense node.

On the target board:

.. code-block:: bash

   sudo apt install ros-$ROS_DISTRO-image-transport
   sudo apt install ros-$ROS_DISTRO-compressed-image-transport

Running the Camera Node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Setup environment
   source install/setup.bash

Launch Directly
"""""""""""""""""""

.. code-block:: bash

   # Default launch
   ros2 run realsense2_camera realsense2_camera_node

   # Launch with pointcloud enabled
   ros2 run realsense2_camera realsense2_camera_node --ros-args -p pointcloud__neon_.enable:=true

Launch via ``rs_launch.py``
""""""""""""""""""""""""""""""

.. code-block:: bash

   # Default launch
   ros2 launch realsense2_camera rs_launch.py

   # Pointcloud
   ros2 launch realsense2_camera rs_launch.py pointcloud__neon_.enable:=true

   # With YAML
   ros2 launch realsense2_camera rs_launch.py config_file:=realsense_config.yaml

Example ``realsense_config.yaml``
"""""""""""""""""""""""""""""""""

.. code-block:: yaml

   # Enable synchronization between multiple streams (e.g., depth + color)
   enable_sync: true

   # Enable depth-to-color alignment (depth image pixels align with the color image)
   align_depth:
     enable: true

   # Perform an initial hardware reset when starting the camera node
   initial_reset: true

   depth_module:
     # Depth stream resolution and frame rate: width x height x fps
     depth_profile: 640x480x30
     # Laser emitter power level (range depends on camera model)
     laser_power: 360.0

   rgb_camera:
     # Color stream resolution and frame rate: width x height x fps
     color_profile: 640x480x30

   pointcloud__neon_:
     # Enable NEON (ARM SIMD) optimized point cloud processing
     enable: false

     # Output point cloud in ordered (2D grid) format
     # true  -> Keep pixel-to-point mapping (width × height)
     # false -> Unordered, removing invalid points to reduce data size
     ordered_pc: false

     # Allow points without texture (color) to remain in the point cloud
     # true  -> Keep all valid depth points, even without color
     # false -> Remove points without texture mapping
     allow_no_texture_points: false

   # Enable specific filters (comma-separated if multiple)
   filters: colorizer

   colorizer:
     # Preset for colorizer filter (affects contrast/brightness of depth coloring)
     visual_preset: 2
     # Color scheme for depth visualization (e.g., jet, white-to-black, etc.)
     color_scheme: 2

   # Time (in seconds) to wait before attempting to reconnect after device loss
   reconnect_timeout: 6.0

.. note::

   Setting the FPS or image size too high when pointcloud is enabled can cause performance issues.
   Please reduce the FPS or image size to suit your use case.

Update Firmware
^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Dry run / info
   rs-fw-update -l

   # Update (example; replace with your FW file)
   sudo rs-fw-update -f /path/to/Intel_RealSense_D4XX_FW.bin

.. note::

   - Use the firmware recommended for your specific model and compatible version with ``librealsense``.
   - Keep the camera on a stable USB 3.x port during updates. Do **not** disconnect power.
