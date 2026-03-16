.. _cross_build_non_ros2_apps:

Cross-build Non-ROS 2 Applications
-----------------------------------

The cross-compilation environment provided by the Renesas RDK Docker image can also be used to build non-ROS 2 applications for the RZ/V2H RDK platform.

This section describes how to cross-compile a generic CMake-based application using the provided toolchain.

Prerequisites
^^^^^^^^^^^^^

- Complete the :ref:`Cross compilation environment setup <requirements_ros2_cross_build>` section.
- Make sure the Docker container is running and accessible.

Setting up the Environment
^^^^^^^^^^^^^^^^^^^^^^^^^^

#. Start and access the Docker container:

   .. code-block:: bash

      docker exec -it ros2_cross_build_container bash

#. (Optional) Connect to the Docker container from VS Code using the **Remote - Containers** extension for a better development experience.

   Refer to the :ref:`Cross compilation environment setup <requirements_ros2_cross_build>` section for instructions on how to connect to the Docker container from VS Code.

Install Dependencies into the Sysroot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before building the application, install any required libraries into the ARM64 sysroot using ``rzv2h-chroot``.

For example, to install common development libraries:

.. code-block:: bash

   rzv2h-chroot apt update
   rzv2h-chroot apt install -y <package-name>

.. important::

   The ``rzv2h-chroot`` command already runs with ``sudo`` privileges. Do not add ``sudo`` to commands executed with ``rzv2h-chroot``.

   For more details on ``rzv2h-chroot``, refer to the :ref:`Cross compilation usage guide <cross_build_usage>`.

Build the Application
^^^^^^^^^^^^^^^^^^^^^

For CMake-based projects, use the cross-compilation toolchain file provided in the Docker container.

#. Navigate to your project directory:

   .. code-block:: bash

      cd /path/to/your/project

#. Create a build directory and configure with CMake:

   .. code-block:: bash

      mkdir build && cd build

      cmake .. -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAINS_WS/cross.cmake \
               -DCMAKE_BUILD_TYPE=Release

   The ``$TOOLCHAINS_WS/cross.cmake`` toolchain file configures the compiler and sysroot for cross-compilation targeting ARM64.

#. Build the project:

   .. code-block:: bash

      make -j$(nproc)

#. The resulting ARM64 binaries are ready to be deployed to the RZ/V2H RDK.

Deploy to Target
^^^^^^^^^^^^^^^^

Copy the built binaries to the RZ/V2H RDK using ``scp`` or another file transfer method.

.. code-block:: bash

   scp ./your_application ubuntu@<TARGET_IP>:/home/ubuntu/

Replace ``<TARGET_IP>`` with the IP address of your RZ/V2H RDK board.