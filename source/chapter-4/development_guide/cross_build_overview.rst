Cross compilation the Application for RZ/V2H RDK
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section provides the overview of cross-building applications for the Renesas RZ/V2H RDK platform.

What is Cross-building?
"""""""""""""""""""""""

Cross-building is the process of compiling software on a host system to run on a different target system.

The advantages of cross-building is that it allows developers to build applications for embedded systems without needing to compile directly on the target device, **which have limited resources causing longer build times**.

In this case, we will be cross-building applications on a development machine (Ubuntu PC 24.04 host) to run on the Renesas RZ/V2H RDK (target).

.. note::

   The primary focus of this section is cross-building ROS 2 applications.

   However, **the cross-build environment can also be used to build non-ROS applications**, provided they are compatible with the RZ/V2H RDK Linux image and the supplied toolchain.

How ROS 2 Cross-building Works
""""""""""""""""""""""""""""""

.. figure:: ../../images/cross_build_overview.png
   :alt: System Overview
   :align: center

   Cross-build system overview

The cross-build environment runs inside a **Docker container** on the host machine. This container is pre-configured with the necessary toolchains and a **sysroot** - a copy of the target device's root filesystem that provides the libraries and headers needed during compilation.

To handle architecture differences, the system uses **QEMU** to emulate the ARM64 architecture. This allows the Docker container running on an x86_64 host to execute ARM binaries, which is essential for tasks such as **installing dependencies and running configuration scripts that need to behave as if they are on the target device.**

The container also leverages the **chroot** technique to create an isolated filesystem environment that mimics the target RZ/V2H system. When entering this chroot environment, QEMU transparently intercepts ARM64 binary execution, enabling tools like ``apt-get`` and ``rosdep`` to resolve and install packages as though they are running natively on the target hardware.

The overall workflow is:

#. The **RZ/V2H RDK Linux Image** provides the root filesystem, which is used as the sysroot inside the Docker container.
#. Dependencies are resolved and installed into the sysroot through the **QEMU-emulated chroot** environment.
#. The **cross-compilation toolchain** compiles the application source code on the host, linking against the libraries in the sysroot to produce ARM64 binaries.
#. The resulting binaries are ready to be deployed to the RZ/V2H RDK target device.

.. tip::

   You can think of the ARM64 sysroot with QEMU emulation as a "virtual RZ/V2H RDK environment" inside the Docker container, which allows you to install dependencies or execute any command via ``rzv2h-chroot`` as if you were directly working on the RZ/V2H RDK.

Limitations
"""""""""""

- Command execution inside the chroot environment may be slower due to QEMU emulation. Please be patient when running commands.
- Only one chroot instance can run simultaneously within the Docker container.