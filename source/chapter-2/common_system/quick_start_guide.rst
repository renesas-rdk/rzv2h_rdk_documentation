.. _linux_kernel_and_device_tree:

Overview
^^^^^^^^

The `rz-utils <https://github.com/renesas-rdk/rz-utils/tree/ubuntu/rz-v2h-rdk>`_ build scripts (``local-build-scripts/``) build the following components from source for the RZ/V2H RDK ver1 (16 GB) and ver101 (8 GB):

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - Component
     - Content
   * - Linux kernel
     - Linux kernel 6.18.20: ``Image``, DTBs, DT overlays (DTBOs), and in-tree modules from `linux-rz <https://github.com/renesas-rdk/linux-rz/tree/ubuntu/rz-v2h-rdk>`_ (branch ``ubuntu/rz-v2h-rdk``).
   * - Out-of-tree modules
     - ``mmngr``, ``mmngrbuf``, ``vspm``, ``vspm_if``, ``mali_kbase``, ``uvcs_drv``.
   * - IPL
     - BL2 and FIP (BL31 + U-Boot) for each board and RZ/V2H Multi-OS mode.

All scripts are driven by ``main_build.sh`` and configured by ``config.ini``.

Prerequisites
^^^^^^^^^^^^^

- Ubuntu 24.04 host, or a Docker container based on Ubuntu 24.04.
- SSH and ``rsync`` on both the build host and the target board.

Install the host packages:

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
       device-tree-compiler \
       libgnutls28-dev \
       git \
       rsync

Clone ``rz-utils``:

.. code-block:: bash

   mkdir -p ~/rzv2h_workspace && cd ~/rzv2h_workspace
   git clone -b ubuntu/rz-v2h-rdk --single-branch --depth 1 https://github.com/renesas-rdk/rz-utils.git

The kernel, TF-A/U-Boot, and out-of-tree module sources are cloned automatically on the first build.

.. important::

   ``mali_kbase`` and ``uvcs_drv`` require proprietary tarballs that are not in the repository. Download them from the **RZ/V2H AI SDK v8.00** and copy them to ``rz-utils/vendor/``:

   - ``mali-g31_km_v1.3.0.tar.gz``
   - ``uvcs_kernel_package_v4.3.4.0.tar.bz2``

   See ``rz-utils/vendor/README.md`` for the SDK paths and SHA-256 checksums.

Configuration
^^^^^^^^^^^^^

Edit ``rz-utils/local-build-scripts/config.ini`` before the first build. Every setting can also be overridden from the environment. Relative paths are resolved against the directory the script is run from.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Setting
     - Description
   * - ``WORKDIR``
     - Base directory for all sources and outputs. Default: ``/workspace/workspace``.
   * - ``KERNEL_DIR``
     - Kernel source tree. Cloned from ``KERNEL_REPO``/``KERNEL_BRANCH`` if missing; never pulled or reset afterwards. Default: ``$WORKDIR/linux-rz``.
   * - ``KERNEL_SRCREV``
     - Optional. Pins the kernel to a commit (checked out with ``-f``; local changes are lost).
   * - ``KERNEL_VARIANT``
     - Optional. Merges ``kernel-config/<name>.config`` on top of ``renesas_defconfig``, for example ``preempt-rt``.
   * - ``KERNEL_MODULES_OUTPUT_DIR``
     - Install directory for in-tree and out-of-tree modules. Default: ``$WORKDIR/kernel-modules``.
   * - ``IPL_BOARDS``
     - ``ver1`` and/or ``ver101``. Default: ``ver101``.
   * - ``IPL_FEATURES``
     - Optional. Multi-OS options; overrides ``ipl_build/machine-features.conf``.
   * - ``DEPLOY_DIR``
     - Output of the ``deploy`` target. Default: ``$WORKDIR/deploy``.
   * - ``GIT_SHALLOW``
     - ``1`` (default): shallow clones. ``0``: full history.

Example:

.. code-block:: ini

   WORKDIR=/home/<user>/rzv2h_workspace/work

Build Commands
^^^^^^^^^^^^^^

.. code-block:: bash

   cd ~/rzv2h_workspace/rz-utils/local-build-scripts
   ./main_build.sh all

``all`` runs ``kernel all``, ``kernel-modules install``, ``ipl all``, and ``deploy`` in sequence. Run ``./main_build.sh`` without arguments for the full help.

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Command
     - Description
   * - ``./main_build.sh kernel <sub_command>``
     - ``defconfig`` | ``menuconfig`` | ``image`` | ``dtbs`` | ``modules`` | ``modules-install`` | ``all`` | ``reset-src`` | ``clean`` | ``distclean``
   * - ``./main_build.sh kernel-modules <sub_command> [<module>]``
     - ``fetch`` | ``reset-src`` | ``all`` | ``install`` | ``clean``. Without ``<module>``, builds all modules in dependency order. Requires the kernel to be built first.
   * - ``./main_build.sh ipl [<sub_command>]``
     - ``all`` (default) | ``keep`` | ``force`` | ``clean``
   * - ``./main_build.sh deploy``
     - Collects the build output into the board root filesystem layout.
   * - ``./main_build.sh clean-all``
     - ``kernel distclean``, ``kernel-modules clean``, and ``ipl clean``.

.. _build_kernel:

Custom Linux Kernel and Device Tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Kernel Source and Configuration
"""""""""""""""""""""""""""""""

Edit the sources in ``$KERNEL_DIR``, then build:

.. code-block:: bash

   ./main_build.sh kernel all                    # Image, DTBs/DTBOs, modules, modules-install
   ./main_build.sh kernel-modules install        # rebuild out-of-tree modules against the new kernel

To change the configuration:

- **Persistent:** edit ``$KERNEL_DIR/arch/arm64/configs/renesas_defconfig``, then run ``./main_build.sh kernel defconfig``.
- **Temporary:** run ``./main_build.sh kernel menuconfig``. Changes are kept until the next ``kernel defconfig``.

Build targets run ``defconfig`` automatically only if ``.config`` is missing, or if ``renesas_defconfig``, ``KERNEL_VARIANT``, or its fragment has changed since the last ``defconfig``.

To build the PREEMPT_RT kernel:

.. code-block:: bash

   KERNEL_VARIANT=preempt-rt ./main_build.sh kernel all
   KERNEL_VARIANT=preempt-rt ./main_build.sh kernel-modules install

.. tip::

   To check an option in the running kernel, run on the **target board**:

   .. code-block:: bash

      zcat /proc/config.gz | grep CONFIG_<option_name>

.. _modify_dts:

Device Tree
"""""""""""

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - File
     - Location in ``$KERNEL_DIR``
   * - Base DTS
     - ``arch/arm64/boot/dts/renesas/rzv2h-rdk-ver1.dts``, ``rzv2h-rdk-ver101.dts``
   * - Overlays
     - ``arch/arm64/boot/dts/renesas/overlays/rzv2h-rdk-*.dts``

After editing, rebuild the device trees only:

.. code-block:: bash

   ./main_build.sh kernel dtbs

Overlays are enabled at boot through ``/boot/uEnv.txt``. See :ref:`device_tree_overlay`.

.. _build_ipl:

IPL (BL2 and FIP)
^^^^^^^^^^^^^^^^^

The release package provides the IPL in the default remoteproc mode, inside the microSD card image and as xSPI files. Build it from source to use another Multi-OS mode or to modify TF-A/U-Boot.

.. code-block:: bash

   ./main_build.sh ipl                                                # IPL_BOARDS, default options
   IPL_BOARDS="ver1 ver101" ./main_build.sh ipl                       # both boards
   IPL_FEATURES="RZ_CM33_FIRMWARE_LOAD RZ_CA55_CPU_CLOCKUP" ./main_build.sh ipl

The default mode is **remoteproc** (``RZ_SRAM_REGION_ACCESS RZ_REMOTEPROC``). Supported modes:

.. list-table::
   :header-rows: 1
   :widths: 35 40 25

   * - Mode
     - ``IPL_FEATURES``
     - Boot device
   * - Remoteproc (default)
     - ``RZ_SRAM_REGION_ACCESS RZ_REMOTEPROC``
     - xSPI or SD
   * - CM33/CR8 started from U-Boot
     - ``RZ_SRAM_REGION_ACCESS``
     - xSPI or SD
   * - CA55 cold boot, CA55 at 1.8 GHz
     - ``RZ_CM33_FIRMWARE_LOAD RZ_CA55_CPU_CLOCKUP``
     - xSPI
   * - CM33 cold boot
     - ``RZ_CM33_COLDBOOT``
     - xSPI

TF-A/U-Boot sources are patched in ``$WORKDIR/ipl-work``. By default, ``ipl`` stops if these trees have local changes. Use ``ipl keep`` to build them as they are, or ``ipl force`` to discard the changes and re-apply the patches.

Output in ``$WORKDIR/ipl-out/rzv2h-rdk-<ver>/``:

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - File
     - Use
   * - ``bl2_bp_spi-rzv2h-rdk-<ver>.srec``
     - BL2 for xSPI boot
   * - ``bl2_bp_esd-rzv2h-rdk-<ver>.bin``
     - BL2 for SD boot (not built with ``RZ_CM33_COLDBOOT`` or ``RZ_CM33_FIRMWARE_LOAD``)
   * - ``fip-rzv2h-rdk-<ver>.srec`` / ``.bin``
     - FIP (BL31 + U-Boot)
   * - ``Flash_Writer_SCIF_RZV2H_DEV_INTERNAL_MEMORY.mot``
     - Flash Writer for SCIF download mode
   * - ``ipl-info.md``
     - Options used and flash addresses of this build

.. warning::

   - Flash the IPL that matches the board. A ver101 IPL on a ver1 board (or vice versa) programs the wrong DDR configuration.
   - Set ``enable_overlay_remoteproc=1`` in ``/boot/uEnv.txt`` **only** in remoteproc mode. Leave it unset in all other modes.
   - ``RZ_CA55_CPU_CLOCKUP`` requires the kernel patch ``ipl_build/kernel/0001-arm64-dts-renesas-r9a09g057-CA55-OPPs-for-1.8GHz-PLL.patch``. Apply it to ``$KERNEL_DIR`` with ``git am`` and rebuild the device trees.

**Flash to xSPI:** follow :ref:`Quick Setup Guide - Option 2: xSPI Boot Mode <quick_setup_rdk_guide>`. The addresses in that guide apply to the remoteproc and CM33/CR8-from-U-Boot modes. For other modes, use the addresses in ``ipl-info.md``.

**Flash to microSD card** (remoteproc and CM33/CR8-from-U-Boot modes only). Run on the build host, with the card unmounted:

.. code-block:: bash

   cd $WORKDIR/ipl-out/rzv2h-rdk-<ver>
   sudo dd if=bl2_bp_esd-rzv2h-rdk-<ver>.bin of=/dev/sdX bs=512 skip=1 seek=1 conv=notrunc
   sudo dd if=fip-rzv2h-rdk-<ver>.bin of=/dev/sdX bs=512 seek=768 conv=notrunc
   sync

``skip=1 seek=1`` keeps sector 0 (partition table) intact.

Deploy to the Target Board
^^^^^^^^^^^^^^^^^^^^^^^^^^

``deploy`` collects the build output into the board root filesystem layout:

.. code-block:: bash

   ./main_build.sh deploy                                  # $DEPLOY_DIR/rzv2h-rdk/
   KERNEL_VARIANT=preempt-rt ./main_build.sh deploy        # $DEPLOY_DIR/rzv2h-rdk-rt/

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - Path in ``$DEPLOY_DIR/rzv2h-rdk[-rt]/``
     - Content
   * - ``boot/Image``
     - Kernel image
   * - ``boot/dtb/renesas/rzv2h-rdk-{ver1,ver101}.dtb``
     - Base device trees
   * - ``boot/dtb/renesas/overlays/rzv2h-rdk-*.dtbo``
     - Device tree overlays
   * - ``boot/uEnv.txt``
     - U-Boot environment (copy of ``ipl_build/uEnv.txt``)
   * - ``boot/{bl2_bp_esd,fip}-rzv2h-rdk-<ver>.bin``
     - IPL binaries, for flashing only
   * - ``usr/lib/modules/<release>/``
     - In-tree modules and out-of-tree modules (``extra/``), debug info stripped
   * - ``deploy-info.txt``
     - Build information (flavor, release, sources, date)

``deploy`` checks that the kernel ``.config``, ``Image``, and the vermagic of every module match the selected flavor (RT or non-RT), so RT and non-RT outputs are never mixed. Components that are not built are skipped with a message. The output directory is recreated on every run.

.. important::

   - Back up ``/boot`` and ``/usr/lib/modules`` on the board before deploying.
   - ``/boot/uEnv.txt`` is overwritten. Merge your local settings (enabled overlays, ``rootdev``) back afterwards.

Run on the **target board** to pull the output from the build host, then reboot:

.. code-block:: bash

   sudo rsync -av --exclude deploy-info.txt \
       <build-user>@<build-host>:<DEPLOY_DIR>/rzv2h-rdk/ /
   sudo reboot

After reboot, check the running kernel with ``uname -a``.

Recovering an Unbootable microSD Card
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If the board no longer boots, restore the files from a PC:

1. Power off the board, remove the microSD card, and insert it into a Linux PC.

2. Mount the root filesystem partition (replace ``/dev/sdX2`` with the actual device):

   .. code-block:: bash

      sudo mkdir -p /mnt/rootfs
      sudo mount /dev/sdX2 /mnt/rootfs

3. Restore the backed-up files, or copy them from the original RZ/V2H RDK image:

   - ``/mnt/rootfs/boot/Image``
   - ``/mnt/rootfs/boot/uEnv.txt``
   - ``/mnt/rootfs/boot/dtb/renesas/``
   - ``/mnt/rootfs/usr/lib/modules/<release>/``

4. Unmount the partition, then reinsert the card into the board:

   .. code-block:: bash

      sudo umount /mnt/rootfs
