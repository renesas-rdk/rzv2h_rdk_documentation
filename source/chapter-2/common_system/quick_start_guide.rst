.. _linux_kernel_and_device_tree:

Overview
--------

The RZ/V2H RDK uses Linux kernel `version 6.10 <https://github.com/Renesas-SST/linux-rz/tree/ubuntu/rz-v2h-rdk>`_ from the **Renesas Software Solutions Team (SST)**. It is based on the Yocto Project ``linux-yocto`` kernel and is customized for the RZ/V2H RDK to ensure compatibility and performance.

This section provides instructions for customizing and rebuilding various system components by using an **Ubuntu 24.04** environment.

Prerequisites
-------------

Before proceeding, ensure that the following prerequisites are met.

.. note::

   Please use the ``ubuntu/rz-v2h-rdk`` branch for the repositories listed below.

   Using files from other branches may lead to build failures or unexpected behavior.

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - **Item**
     - **Description / Link**
   * - Ubuntu 24.04 PC with Docker
     - A host machine running Ubuntu 24.04 with Docker installed is required to build, run, and manage the demo environment.

       Ensure that Docker is installed and working correctly before continuing.
   * - `linux-rz <https://github.com/Renesas-SST/linux-rz/tree/ubuntu/rz-v2h-rdk>`_
     - Linux kernel source code for the RZ/V2H RDK. You can customize and rebuild it as needed.
   * - `rz-utils <https://github.com/Renesas-SST/rz-utils/tree/ubuntu/rz-v2h-rdk>`_
     - A collection of utilities for workflows related to Renesas RZ-based devices, including the RZ/V2H RDK.
   * - SSH and rsync access
     - SSH connectivity between the build host and target board, with ``rsync`` installed on both systems, is required to copy build artifacts.

Quick Setup Guide
-----------------

Use an Ubuntu 24.04 host machine or a Docker container based on an Ubuntu 24.04 image.

Install the required packages:

.. code-block:: bash

   sudo apt update
   sudo apt install \
       build-essential \
       gcc-aarch64-linux-gnu \
       bc \
       bison \
       flex \
       libssl-dev \
       libncurses-dev \
       git \
       rsync

Create a workspace for the build (you can choose any location and name):

.. code-block:: bash

   mkdir -p ~/rzv2h_workspace
   cd ~/rzv2h_workspace

Clone the required repositories using the ``ubuntu/rz-v2h-rdk`` branch:

.. code-block:: bash

   git clone -b ubuntu/rz-v2h-rdk https://github.com/Renesas-SST/linux-rz.git --single-branch --depth 1
   git clone -b ubuntu/rz-v2h-rdk https://github.com/Renesas-SST/rz-utils.git --single-branch --depth 1

.. note::

   For this release, the ``ubuntu/rz-v2h-rdk`` branch is the only supported branch for building the Linux kernel for the RZ/V2H RDK.

   The build tool for the bootloaders (U-Boot and ATF) is not included in this release.

Set up the environment for the build
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before running the build script, complete the following steps:

1. Change to the workspace directory.
2. Update the ``rz-utils/local-build-scripts/config.ini`` file.
3. Set ``KERNEL_DIR`` to the path of your local ``linux-rz`` source tree.
4. Set ``KERNEL_MODULES_OUTPUT_DIR`` to the directory where the build system should install kernel modules.
5. Change to the local build script directory before executing the script.

For example:

.. code-block:: ini

   KERNEL_DIR=/home/<user>/rzv2h_workspace/linux-rz
   KERNEL_MODULES_OUTPUT_DIR=/home/<user>/rzv2h_workspace/output

Follow the next sections for detailed instructions on building the Linux kernel and related components for the RZ/V2H RDK.

.. _build_kernel:

Custom Linux Kernel and Device Tree
-----------------------------------

This section describes how to customize and build the Linux kernel and device tree blobs (DTBs) for the RZ/V2H RDK by using the `rz-utils <https://github.com/Renesas-SST/rz-utils/tree/ubuntu/rz-v2h-rdk>`_ repository.

Customizing and Building the Linux Kernel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the build script provided in the ``rz-utils`` repository to build the Linux kernel for the RZ/V2H RDK.

Change to the build script directory:

.. code-block:: bash

   cd ~/rzv2h_workspace/rz-utils/local-build-scripts

Modify the kernel source as needed by editing the files in the ``linux-rz`` directory.

Build the Linux kernel image, device tree blobs (DTBs), and modules:

.. code-block:: bash

   ./main_build.sh kernel all

You can also build individual kernel outputs as needed:

.. code-block:: bash

   ./main_build.sh kernel image
   ./main_build.sh kernel modules-install

The generated output files are placed in the output directories defined by the build system and ``config.ini``.

Customizing and Building the Kernel Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The purpose of customizing the kernel configuration is to enable or disable specific features, drivers, or options in the Linux kernel based on your use case and requirements.

One common reason to customize the kernel configuration is to add support for additional drivers or hardware components that are not included in the default configuration.

For example, if you want to use a specific camera sensor that requires a kernel driver not currently enabled, you would need to enable that driver in the kernel configuration.

Please refer to the `Linux Kernel Configuration Guide <https://www.kernel.org/doc/html/latest/kbuild/kconfig.html>`_ for detailed information on how to customize the kernel configuration.

To modify the kernel configuration, run the following command:

.. code-block:: bash

   ./main_build.sh kernel menuconfig

This opens the interactive kernel configuration interface, where you can enable or disable features based on your use case.

After updating the kernel configuration, save the changes and exit the menu.

**(Recommended)** If you want to manually change the default kernel configuration for the target platform, please edit the ``linux-rz/arch/arm64/configs/renesas_defconfig`` file and then run the following command to update the kernel configuration:

.. code-block:: bash

   ./main_build.sh kernel defconfig

After updating the configuration, rebuild the kernel image and related outputs:

.. code-block:: bash

   ./main_build.sh kernel all

.. tip::

   To check the specific configuration options that are enabled in the default configuration, you can run the following command **on the target board**:

   .. code-block:: bash

      zcat /proc/config.gz | grep "CONFIG_<option_name>"

.. note::

   Kernel configuration changes may affect image size, enabled drivers, boot behavior, and module compatibility. Review your changes carefully before deploying the updated kernel to the target board.

.. _modify_dts:

Customizing and Building the Device Tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the device tree sources in the ``linux-rz`` repository to modify hardware-related settings such as enabled peripherals, pin control, buses, and attached devices.

For the RZ/V2H RDK, the main device tree source file is located at
``linux-rz/arch/arm64/boot/dts/renesas/rzv2h-rdk-ver1.dts``.

The device tree overlay source files are located at
``linux-rz/arch/arm64/boot/dts/renesas/overlays/rzv2h-rdk-1.0*.dts``.

Refer to the `Device Tree Usage Guide <https://www.kernel.org/doc/html/latest/devicetree/usage-model.html>`_ for detailed information on how to modify device tree blobs (DTBs) and overlays (DTBOs).

After making changes to the device tree source files, rebuild the device tree blobs:

.. code-block:: bash

   ./main_build.sh kernel dtbs

Build output locations
----------------------

The generated kernel artifacts are typically available in the following locations on the build host:

.. list-table::
   :header-rows: 1
   :widths: 25 40 35

   * - Artifact
     - Output location
     - Purpose
   * - Kernel image
     - ``linux-rz/arch/arm64/boot/Image``
     - Linux kernel binary loaded by U-Boot during boot.
   * - DTB
     - ``linux-rz/arch/arm64/boot/dts/renesas/rzv2h-rdk-ver1.dtb``
     - Hardware description used by the kernel for the base board configuration.
   * - DTBO
     - ``linux-rz/arch/arm64/boot/dts/renesas/overlays/rzv2h-rdk-1.0*.dtbo``
     - Device tree overlay files used to modify or extend the base hardware description.
   * - Kernel modules
     - ``$KERNEL_MODULES_OUTPUT_DIR/lib/modules/6.10.14-arm64-renesas/``
     - Loadable kernel drivers and other kernel components used at runtime.

Target deployment locations
---------------------------

The updated files should be deployed to the following locations on the target board:

.. list-table::
   :header-rows: 1
   :widths: 25 30 45

   * - Artifact
     - Target location
     - Purpose
   * - Kernel image
     - ``/boot/Image``
     - Linux kernel binary loaded by U-Boot during boot.
   * - DTB
     - ``/boot/dtb/renesas/rzv2h-rdk-ver1.dtb``
     - Hardware description used by the kernel for the base board configuration.
   * - DTBO
     - ``/boot/dtb/renesas/overlays/rzv2h-rdk-1.0*.dtbo``
     - Device tree overlay files used to modify or extend the base hardware description.
   * - Kernel modules
     - ``/usr/lib/modules/6.10.14-arm64-renesas/``
     - Loadable kernel drivers and other kernel components used at runtime.

.. warning::

   Do not modify, remove, or overwrite any files other than those listed above, except ``uEnv.txt``, unless explicitly required.

Copy files from the build host to the target board
--------------------------------------------------

.. tip::

   If you only modify the device tree source files and do not make any changes to the kernel configuration or source code, you can choose to copy only the updated DTB and DTBO files to the target board without copying the kernel image and modules.

.. important::

   Before overwriting any existing files on the target board, back up the current kernel image, DTB, DTBO files, and kernel modules.

   Make sure to keep a copy of the original files so that you can restore them if needed.

Run the following commands on the **target board** to copy the generated artifacts from the build host over SSH by using ``rsync``.

Install ``rsync`` on the target board if it is not already installed:

.. code-block:: bash

   sudo apt update
   sudo apt install rsync

Set environment variables:

.. code-block:: bash

   export BUILD_DIR=~/rzv2h_workspace/linux-rz  # Change this to the actual kernel source path on the build host
   export BUILD_USER=<build-host-user>
   export BUILD_HOST=<build-host-ip>
   export KERNEL_MODULES_OUTPUT_DIR=<path-to-kernel-modules-output>  # Change if needed

Copy the kernel image:

.. code-block:: bash

   sudo rsync -avz "${BUILD_USER}@${BUILD_HOST}:${BUILD_DIR}/arch/arm64/boot/Image" /boot/Image

Copy the DTB:

.. code-block:: bash

   sudo rsync -avz "${BUILD_USER}@${BUILD_HOST}:${BUILD_DIR}/arch/arm64/boot/dts/renesas/rzv2h-rdk-ver1.dtb" \
     /boot/dtb/renesas/rzv2h-rdk-ver1.dtb

Copy the DTBO files:

.. code-block:: bash

   sudo rsync -avz \
     "${BUILD_USER}@${BUILD_HOST}:${BUILD_DIR}/arch/arm64/boot/dts/renesas/overlays/rzv2h-rdk-1.0*.dtbo" \
     /boot/dtb/renesas/overlays/

Copy the kernel modules:

.. code-block:: bash

   sudo rsync -avz --delete-during \
     --exclude='extra/' \
     "${BUILD_USER}@${BUILD_HOST}:${KERNEL_MODULES_OUTPUT_DIR}/lib/modules/6.10.14-arm64-renesas/" \
     /usr/lib/modules/6.10.14-arm64-renesas/

.. caution::

   The local ``/usr/lib/modules/6.10.14-arm64-renesas/extra/`` directory **must be preserved**, as it contains additional
   kernel modules that are not part of the synchronized output. For this reason,
   ``--exclude='extra/'`` is required when running ``rsync`` with
   ``--delete-during``.

   If the ``extra/`` directory is accidentally removed, you can restore it from a backup or copy it from the original image of the RZ/V2H RDK.

After copying the files, update module dependencies on the target device:

.. code-block:: bash

   sudo depmod 6.10.14-arm64-renesas

Reboot the target board to apply the updated kernel, device tree, and modules:

Recovering Files from an Unbootable SD Card
-------------------------------------------

If the target board can no longer boot or required files have been accidentally
removed, you can restore the original files by mounting the SD card on a PC.

1. Power off the target board and remove the SD card.

2. Insert the SD card into a Linux PC or another system that can access its
   partitions.

3. Mount the root filesystem partition from the SD card:

   .. code-block:: bash
      :emphasize-lines: 3

      mkdir -p /mnt/rootfs
      # Mount the root filesystem partition (replace /dev/sdX2 with the actual device name)
      sudo mount /dev/sdX2 /mnt/rootfs

4. Copy the original files back to the mounted partitions. Key files to restore
   include:

   - ``/mnt/rootfs/boot/Image``
   - ``/mnt/rootfs/boot/dtb/renesas/rzv2h-rdk-ver1.dtb``
   - ``/mnt/rootfs/boot/dtb/renesas/overlays/``
   - ``/mnt/rootfs/usr/lib/modules/6.10.14-arm64-renesas/``
   - ``/mnt/rootfs/usr/lib/modules/6.10.14-arm64-renesas/extra/``

5. Safely unmount the partitions:

   .. code-block:: bash

      sudo umount /mnt/rootfs

6. Remove the SD card from the PC, reinsert it into the target board, and power
   on the system.

.. tip::

   If available, restore these files from a backup created before deployment or
   copy them from the original SD card image of the RZ/V2H RDK.