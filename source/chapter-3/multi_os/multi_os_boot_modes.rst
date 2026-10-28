.. _multi_os_boot_modes:

CA55 1.8 GHz Clock-up and CM33 Cold Boot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default IPL of the RZ/V2H RDK runs in **remoteproc** mode: CA55 boots first at 1.7 GHz, and Linux starts the CM33/CR8 cores. This section describes two other boot modes of the RZ/V Multi-OS Package:

- **CA55 1.8 GHz clock-up at CA55 cold boot**: CA55 boots first. BL2 loads the CM33 firmware from xSPI flash and starts CM33, and the CM33 firmware raises the CA55 clock to 1.8 GHz.
- **CM33 cold boot**: CM33 boots first from xSPI flash, then loads the CA55 IPL from xSPI flash and starts CA55.

Both modes need an IPL built from source and booting from **xSPI flash**. The kernel, device tree, and root filesystem are still loaded from the microSD card.

This section follows chapters 5 and 6 of the `RZ/V2H Quick Start Guide for RZ/V Multi-OS Package <https://www.renesas.com/en/document/qsg/rzv2h-quick-start-guide-rzv-multi-os-package>`_ (R01QS0077EJ0420), adapted to the RZ/V2H RDK and the `rz-utils <https://github.com/renesas-rdk/rz-utils/tree/ubuntu/rz-v2h-rdk>`_ build scripts.

.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - **Mode**
     - **IPL options (IPL_FEATURES)**
     - **Other changes**
   * - CA55 1.8 GHz clock-up
     - ``RZ_CM33_FIRMWARE_LOAD RZ_CA55_CPU_CLOCKUP``
     - Kernel device tree patch (CA55 OPPs), CM33 firmware with **Clock up for CA55** enabled, written to xSPI flash
   * - CM33 cold boot
     - ``RZ_CM33_COLDBOOT``
     - CM33 firmware with **Launch CA55(core0)** enabled, written and started from e² studio

.. important::

   - Remoteproc is not available in these modes. Comment out ``enable_overlay_remoteproc=1`` in ``/boot/uEnv.txt`` on the target board before you boot with the new IPL. See :ref:`Device Tree Overlay <device_tree_overlay>`.
   - Use the IPL that matches your board version (``ver1`` or ``ver101``). See :ref:`rdk_board_versions`.
   - To return to the default mode, build the IPL with the default options, flash it as described in :ref:`Quick Setup Guide - Option 2: xSPI Boot Mode <quick_setup_rdk_guide>`, revert the kernel patch if you applied it, and set ``enable_overlay_remoteproc=1`` again.

Prerequisites
"""""""""""""

- The build environment of ``rz-utils`` is set up as described in :ref:`linux_kernel_and_device_tree`. The commands below run from ``rz-utils/local-build-scripts``, and ``$WORKDIR`` and ``$KERNEL_DIR`` are the values in ``config.ini``.
- The RZ/V2H RDK boots Linux from the microSD card (see :ref:`quick_setup_rdk_guide`).
- e² studio and RZ/V FSP 4.2 are installed (see :ref:`Requirement <multi_os_requirement>`).
- A Windows PC with a terminal emulator (for example, Tera Term) to write the xSPI flash in SCIF download mode.
- A SEGGER J-Link debugger, for CM33 cold boot only.

.. _ca55_clockup:

CA55 1.8 GHz Clock-up at CA55 Cold Boot
"""""""""""""""""""""""""""""""""""""""

In this mode, CA55 boots first from xSPI flash. BL2 loads the CM33 firmware written at offset ``0x202000`` of the xSPI flash and starts CM33. The CM33 firmware, built with the **Clock up for CA55** BSP property, configures the CA55 clock to 1.8 GHz instead of 1.7 GHz.

.. list-table:: Boot flow (CA55 1.8 GHz clock-up)
   :header-rows: 1
   :widths: 10 30 60

   * - **Step**
     - **Running on**
     - **Action**
   * - 1
     - CA55 boot ROM
     - Loads BL2 from xSPI offset ``0x0``
   * - 2
     - BL2 (CA55)
     - Loads the CM33 firmware from xSPI offset ``0x202000`` and starts CM33
   * - 3
     - CM33 firmware
     - Configures the CA55 clock to 1.8 GHz
   * - 4
     - BL2 (CA55)
     - Loads the FIP (BL31 and U-Boot) from xSPI offset ``0x60000``
   * - 5
     - U-Boot
     - Loads the kernel and device tree from the microSD card

The CM33 firmware (S-record) is written to xSPI offset ``0x202000`` with the Flash Writer, together with BL2 and the FIP, as described in `Write the Build Artifacts to xSPI Flash`_.

Build the IPL
~~~~~~~~~~~~~

#. Build the IPL with the CA55 clock-up options:

   .. code-block:: bash

      cd ~/rzv2h_workspace/rz-utils/local-build-scripts
      IPL_FEATURES="RZ_CM33_FIRMWARE_LOAD RZ_CA55_CPU_CLOCKUP" ./main_build.sh ipl

   For RDK ver1, add ``IPL_BOARDS=ver1``. ``IPL_FEATURES`` replaces the options in ``ipl_build/machine-features.conf``.

   .. note::

      Do not combine these options with ``RZ_REMOTEPROC`` or ``RZ_CM33_COLDBOOT``. ``build_ipl.sh`` rejects unsupported combinations.

#. Check the output in ``$WORKDIR/ipl-out/rzv2h-rdk-<ver>/``:

   - ``bl2_bp_spi-rzv2h-rdk-<ver>.srec``
   - ``fip-rzv2h-rdk-<ver>.srec``
   - ``Flash_Writer_SCIF_RZV2H_DEV_INTERNAL_MEMORY.mot``
   - ``0001-arm64-dts-renesas-r9a09g057-CA55-OPPs-for-1.8GHz-PLL.patch``
   - ``ipl-info.md`` (options and flash addresses of this build)

   Only the xSPI BL2 is built. SD boot is not supported in this mode.

Update the Kernel Device Tree
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The CA55 OPP table in ``r9a09g057.dtsi`` is defined for a 1.7 GHz PLL (1.7 GHz, 850, 425, and 212.5 MHz). With the 1.8 GHz PLL, it must be changed to 1.8 GHz, 900, 450, and 225 MHz. The patch is not in the kernel branch because it is wrong in the other modes.

#. Apply the patch to the kernel source:

   .. code-block:: bash

      git -C $KERNEL_DIR am $PWD/ipl_build/kernel/0001-arm64-dts-renesas-r9a09g057-CA55-OPPs-for-1.8GHz-PLL.patch

#. Rebuild the device trees:

   .. code-block:: bash

      ./main_build.sh kernel dtbs

#. Copy the device tree of your board to the target board, for example ``rzv2h-rdk-ver101.dtb``:

   .. code-block:: bash

      # On the host machine
      scp $KERNEL_DIR/arch/arm64/boot/dts/renesas/rzv2h-rdk-ver101.dtb <user>@<board_ip>:/tmp/

      # On the target board
      sudo cp /boot/dtb/renesas/rzv2h-rdk-ver101.dtb /boot/dtb/renesas/rzv2h-rdk-ver101.dtb.bak
      sudo cp /tmp/rzv2h-rdk-ver101.dtb /boot/dtb/renesas/

   You can also deploy the full build output as described in :ref:`linux_kernel_and_device_tree`.

#. On the target board, comment out the remoteproc overlay in ``/boot/uEnv.txt``:

   .. code-block:: bash

      #enable_overlay_remoteproc=1

Build the CM33 Firmware
~~~~~~~~~~~~~~~~~~~~~~~

#. Import or create an RZ/V FSP project for CM33 in e² studio (see :ref:`How to Create the New Project for RZ/V2H RDK <multi_os_create_project>`).

#. Open ``configuration.xml`` in the project and select the **BSP** tab.

#. In the **Properties** view, set the following properties under **RZV2H**:

   .. list-table::
      :header-rows: 1
      :widths: 50 50

      * - **Property**
        - **Value**
      * - Clock up for CA55
        - **Enabled**
      * - Launch CA55(core0)
        - Disabled
   .. figure:: ../../images/ca55_clockup_1_8_GHz.png
      :alt: CM33 Cold Boot Example
      :width: 600px

#. Click **Generate Project Content** to apply the changes to the project.

#. Build the project with **Project** > **Build Project**.

   The S-record file ``<project_name>.srec`` is generated in the ``Debug`` folder of the project.

Write the Build Artifacts to xSPI Flash
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Copy ``Flash_Writer_SCIF_RZV2H_DEV_INTERNAL_MEMORY.mot``, ``bl2_bp_spi-rzv2h-rdk-<ver>.srec``, ``fip-rzv2h-rdk-<ver>.srec``, and ``<project_name>.srec`` to the Windows PC.

#. Power off the board and set **DSW1** to **SCIF download** mode (DSW1-4 OFF, DSW1-5 ON). Keep DSW1-1 ON (CA55 cold boot).

#. Connect the board to the Windows PC with a USB-to-microUSB cable, power on the board, and send ``Flash_Writer_SCIF_RZV2H_DEV_INTERNAL_MEMORY.mot`` from the terminal emulator.

   The terminal settings and the transfer procedure are the same as in :ref:`Quick Setup Guide - Option 2: xSPI Boot Mode <quick_setup_rdk_guide>`.

#. Write each file with the ``XLS2`` command. Enter the **Program Top Address** and the **Qspi Save Address** below, send the file as text after ``please send !``, and enter ``y`` if the Flash Writer asks to clear the area:

   .. list-table::
      :header-rows: 1
      :widths: 50 25 25

      * - **File**
        - **Program Top Address**
        - **Qspi Save Address**
      * - ``bl2_bp_spi-rzv2h-rdk-<ver>.srec``
        - ``8101E00``
        - ``0``
      * - ``fip-rzv2h-rdk-<ver>.srec``
        - ``0``
        - ``60000``
      * - ``<project_name>.srec`` (CM33 firmware)
        - ``0``
        - ``202000``

   For example, the CM33 firmware is written as follows. The end address depends on the firmware size:

   .. code-block:: text

      > XLS2
      ===== Qspi writing of RZ/V2x Board Command =============
      Load Program to Spiflash
      Writes to any of SPI address.
      Program size & Qspi Save Address
      ===== Please Input Program Top Address ============
      Please Input : H'00000

      ===== Please Input Qspi Save Address ===
      Please Input : H'202000
      please send ! ('.' & CR stop load)
      Erase SPI Flash memory...
      Erase Completed
      Write to SPI Flash memory.
      ======= Qspi Save Information =================
      SpiFlashMemory Stat Address : H'00202000
      SpiFlashMemory End Address  : H'002071F2
      ===========================================================

#. Power off the board and set **DSW1** to **xSPI boot** mode (DSW1-4 OFF, DSW1-5 OFF), with DSW1-1 ON.

#. Insert the microSD card and power on the board.

Verify the CA55 Clock
~~~~~~~~~~~~~~~~~~~~~

After Linux boots, check the CA55 frequencies on the target board:

.. code-block:: bash

   cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies
   cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq

With the patched device tree, the output is:

.. code-block:: text

   225000 450000 900000 1800000
   1800000

.. _cm33_coldboot:

CM33 Cold Boot
""""""""""""""

In this mode, CM33 boots first from xSPI flash. The CM33 firmware, built with the **Launch CA55(core0)** BSP property, loads BL2 and the FIP of CA55 from xSPI flash and starts CA55. BL2 and the FIP are written at different xSPI offsets from the other modes, because the CM33 firmware uses the start of the flash.

.. list-table:: Boot flow (CM33 cold boot)
   :header-rows: 1
   :widths: 10 30 60

   * - **Step**
     - **Running on**
     - **Action**
   * - 1
     - CM33 boot ROM
     - Reads the boot parameter and the CM33 firmware from xSPI offset ``0x0``, copies the firmware to the internal SRAM, and starts it
   * - 2
     - CM33 firmware
     - Loads BL2 from xSPI offset ``0x100000`` and starts CA55
   * - 3
     - BL2 (CA55)
     - Loads the FIP (BL31 and U-Boot) from xSPI offset ``0x280000``
   * - 4
     - U-Boot
     - Loads the kernel and device tree from the microSD card

The CM33 firmware is written to xSPI offset ``0x0`` by e² studio through the J-Link debugger, as described in `Build and Run the CM33 Firmware`_. The Flash Writer is used only for BL2 and the FIP.

Build the IPL
~~~~~~~~~~~~~

#. Build the IPL with the CM33 cold boot option:

   .. code-block:: bash

      cd ~/rzv2h_workspace/rz-utils/local-build-scripts
      IPL_FEATURES="RZ_CM33_COLDBOOT" ./main_build.sh ipl

   For RDK ver1, add ``IPL_BOARDS=ver1``.

   .. note::

      ``RZ_CM33_COLDBOOT`` cannot be combined with ``RZ_REMOTEPROC``, ``RZ_CM33_FIRMWARE_LOAD``, or ``RZ_CA55_CPU_CLOCKUP``. Only the xSPI BL2 is built.

#. Check the output in ``$WORKDIR/ipl-out/rzv2h-rdk-<ver>/``: ``bl2_bp_spi-rzv2h-rdk-<ver>.srec``, ``fip-rzv2h-rdk-<ver>.srec``, ``Flash_Writer_SCIF_RZV2H_DEV_INTERNAL_MEMORY.mot``, and ``ipl-info.md``.

#. On the target board, comment out the remoteproc overlay in ``/boot/uEnv.txt``:

   .. code-block:: bash

      #enable_overlay_remoteproc=1

Write the CA55 IPL to xSPI Flash
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#. Start the Flash Writer in SCIF download mode as described in steps 1 and 2 of `Write the Build Artifacts to xSPI Flash`_.

#. Write each file with the ``XLS2`` command and the following addresses:

   .. list-table::
      :header-rows: 1
      :widths: 50 25 25

      * - **File**
        - **Program Top Address**
        - **Qspi Save Address**
      * - ``bl2_bp_spi-rzv2h-rdk-<ver>.srec``
        - ``8101E00``
        - ``100000``
      * - ``fip-rzv2h-rdk-<ver>.srec``
        - ``0``
        - ``280000``

   .. warning::

      These addresses are different from the default mode. If you write BL2 at ``0`` or the FIP at ``60000``, the board does not boot in CM33 cold boot mode.

#. Power off the board.

Build and Run the CM33 Firmware
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The CM33 firmware is written to xSPI flash and started from e² studio with the J-Link debugger.

#. Import or create an RZ/V FSP project for CM33 in e² studio (see :ref:`How to Create the New Project for RZ/V2H RDK <multi_os_create_project>`).

#. Open ``configuration.xml`` in the project and select the **BSP** tab.

#. In the **Properties** view, set the following properties under **RZV2H**:

   .. list-table::
      :header-rows: 1
      :widths: 50 50

      * - **Property**
        - **Value**
      * - Launch CA55(core0)
        - **Enabled**
      * - Clock up for CA55
        - Enabled only if CA55 must run at 1.8 GHz

   .. figure:: ../../images/cm33_coldboot_launch_ca55.png
      :alt: CM33 Cold Boot Example
      :width: 600px

   .. note::

      If you enable **Clock up for CA55**, the kernel device tree must also use the 1.8 GHz OPP table. Apply the kernel patch as described in `Update the Kernel Device Tree`_.

#. Click **Generate Project Content** to apply the changes to the project.

#. Build the project with **Project** > **Build Project**.

#. Open **Run** > **Debug Configurations...**, expand **Renesas GDB Hardware Debugging**, and select **<project_name> Debug_Flat**.

#. In the **Debugger** tab, open **Connection Settings** and set **Reset after download** to **Yes**.

#. In the **Startup** tab, change the **Load type** of the program binary to **Symbols only**.

#. In the **Startup** tab, click **Add...** > **Workspace...**, select ``Debug/<project_name>.srec``, and click **OK**. Keep its load type as **Image and Symbols**.

   .. figure:: ../../images/Startup.png
      :alt: CM33 Cold Boot Example
      :width: 600px

#. Set **DSW1** on the RZ/V2H RDK as follows:

   .. list-table::
      :header-rows: 1
      :widths: 20 20 60

      * - **Switch**
        - **Setting**
        - **Function**
      * - DSW1-1
        - OFF
        - CM33 cold boot
      * - DSW1-4
        - OFF
        - xSPI boot (with DSW1-5)
      * - DSW1-5
        - OFF
        - xSPI boot (with DSW1-4)
      * - DSW1-6
        - ON
        - JTAG debug mode

#. Insert the microSD card, connect the J-Link debugger, and power on the board.

#. Click **Debug** to start the debug session. The CM33 firmware starts, loads the CA55 IPL from xSPI flash, and starts CA55. U-Boot and Linux then boot from the microSD card.

.. attention::

   RZ/V FSP 4.2 installs J-Link DLL version 9.44, which e² studio uses to connect to the board. For CM33 cold boot, you **must** use J-Link DLL version **7.96e**. For how to install it and switch e² studio to it, see `Getting Started with RZ/V Flexible Software Package Page 27 of 35 <https://www.renesas.com/en/document/apn/rzv-getting-started-flexible-software-package>`_.
