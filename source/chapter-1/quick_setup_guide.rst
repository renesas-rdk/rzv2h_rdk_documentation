.. _quick_setup_rdk_guide:

Quick start guide for RZ/V2H RDK
================================

This quick start guide focuses on booting the board using a **microSD card**, which is the most straightforward method.

Other advanced boot methods, such as **xSPI flash**, are also supported.

The **TFTP + NFS boot** method is supported as well but is not covered in detail here.

Preparing the microSD card
----------------------------

To boot the RZ/V2H RDK using a microSD card, you must first flash a bootable Linux image onto it.

There are two options for flashing the image:

- **Option 1: Flash using bmaptool (Ubuntu)** - A faster command-line tool for flashing images using block map files.
- **Option 2: Flash using Balena Etcher** - A user-friendly GUI tool that supports multiple platforms (Windows, macOS, Linux).

Requirements
^^^^^^^^^^^^^^^

- A host machine for flashing the image:

  - Ubuntu with ``bmaptool``, or
  - Windows, macOS, or Linux with Balena Etcher

- **microSD card**: 16 GB or larger.
  For best performance and compatibility, we recommend using the included 64 GB SanDisk microSD card.

- **Provided bootable files:**

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - **File name**
     - **Description**
   * - ubuntu-24.04-server-arm64-rzv2h-rdk.img.xz
     - Compressed Ubuntu 24.04 server image for RZ/V2H RDK
   * - ubuntu-24.04-server-arm64-rzv2h-rdk.bmap
     - Block map file for fast flashing with bmaptool

Option 1: Flash using bmaptool (Ubuntu)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

bmaptool is a faster command-line tool for flashing images to microSD cards using block map files (bmap).
It provides quicker flashing compared to traditional methods by skipping empty blocks and verifying data integrity automatically.

1. **Install bmaptool**

   On **Ubuntu**:

   .. code-block:: bash

      sudo apt-get install bmap-tools

2. **Flashing the Image**

   a. Insert your microSD card into your machine.

   b. Identify the microSD card device name:

      .. code-block:: bash

         lsblk

      Look for your microSD card (e.g., ``/dev/sdb``). Make sure to identify it correctly.

      .. warning::

         Please confirm the microSD card device name carefully.
         Double-check to avoid overwriting your main disk.

   c. Unmount any auto-mounted partitions on the microSD card:

      .. code-block:: bash

         sudo umount /dev/sdX*

   d. Flash the image directly (no need to extract):

      .. code-block:: bash

         sudo bmaptool copy ubuntu-24.04-server-arm64-rzv2h-rdk.img.xz /dev/sdX

      Replace ``/dev/sdX`` with your actual microSD card device (e.g., ``/dev/sdb``, not ``/dev/sdb1``).

      .. note::

         Please ensure that the ``.bmap`` file is in the same directory as the image file.
         bmaptool automatically detects the ``.bmap`` file with the same base name.

         You can also specify it explicitly:

         .. code-block:: bash

            sudo bmaptool copy --bmap ubuntu-24.04-server-arm64-rzv2h-rdk.bmap \
              ubuntu-24.04-server-arm64-rzv2h-rdk.img.xz /dev/sdX

   e. Wait for the process to complete. bmaptool will display progress and verify the image after flashing.

   f. Before removing the microSD card, run:

      .. code-block:: bash

         sync

Option 2: Flash using Balena Etcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Balena Etcher is a user-friendly GUI tool to flash OS images to microSD cards and USB drives.
It provides a simple and safe method.

It supports many OS platforms, including Windows, macOS, and Linux.

1. **Install Balena Etcher**

   Download and install the software from the `Balena Etcher Official Website <https://etcher.balena.io/>`_.

2. **Flashing the Image**

   - Once Etcher is open:

     .. figure:: ../images/balenaetcher-eye.jpg
        :alt: Balena Etcher Application
        :width: 500px
        :align: center

        Balena Etcher Application

   - **Select Image:** Click ``Flash from file`` and choose your image file (e.g., ubuntu-24.04-server-arm64-rzv2h-rdk.img.xz)
   - **Select Target:** Insert your microSD card into the host machine and choose the correct device.

     .. note::
        Please confirm the microSD card device name carefully.
        Double-check to avoid overwriting your main disk.

   - **Flashing:** Click ``Flash`` to begin. Etcher will:

     - Write the image
     - Validate the image
     - Automatically unmount the microSD card

   - **Finish:** Remove the microSD card safely after Etcher reports successful completion.

Boot Mode Configuration (DIP Switch)
------------------------------------

Before powering up the RZ/V2H RDK, make sure the board's boot mode is configured correctly using the DIP switches.

+------------+-------------+----------------------------------------------------+
| Switch No. | RZ/V2H Pin  | Function                                           |
+============+=============+====================================================+
| 1          | BOOTSELCPU  | Select the cold boot CPU                           |
|            |             |                                                    |
|            |             | OFF: CM33, **ON**: CA55 (default)                  |
+------------+-------------+----------------------------------------------------+
| 2          | BOOTPLLCA1  | Input the CA55 frequency at the CA55 cold boot     |
|            |             |                                                    |
|            |             | BOOTPLLCA[1:0]                                     |
+------------+-------------+                                                    +
| 3          | BOOTPLLCA0  | - = [OFF:OFF]: 1.6 GHz                             |
|            |             | - = [**OFF:ON**]: 1.7 GHz (default)                |
|            |             | - = [ON:OFF]: 1.1 GHz                              |
|            |             | - = [ON:ON]: 1.5 GHz                               |
+------------+-------------+----------------------------------------------------+
| 4          | MD_BOOT1    | Input the boot mode select signal                  |
|            |             |                                                    |
|            |             | MD_BOOT[1:0]                                       |
+------------+-------------+                                                    +
| 5          | MD_BOOT0    | - = [OFF:OFF]: xSPI                                |
|            |             | - = [OFF:ON]: SCIF                                 |
|            |             | - = [**ON:OFF**]: SD (default)                     |
|            |             | - = [ON:ON]: eMMC (not supported) on RZ/V2H RDK    |
+------------+-------------+----------------------------------------------------+
| 6          | MD_BOOT3    | Select JTAG debug mode                             |
|            |             |                                                    |
|            |             | **OFF**: Normal mode (default), ON: Debug mode     |
+------------+-------------+----------------------------------------------------+

.. attention::

   Always power off the board before changing boot switches.

Boot Mode Support
-----------------

The board supports two boot options, including:

.. list-table::
   :header-rows: 1
   :widths: 30 50 20

   * - **Boot Source**
     - **Description**
     - **DSW1 Setting**
   * - microSD card
     - Boot from microSD card
     - SD mode
   * - xSPI
     - Boot from xSPI flash
     - xSPI mode

.. _jtag_reset_tip:

.. tip::
   The serial port is powered by the **board's power supply**, not by the **USB port** from the PC.
   Early boot messages might not appear automatically in the terminal (including U-Boot console and SCIF terminal).
   To view them, manually reset the board by connecting **JTAG QRESN (PIN10)** to **GND**, as shown below.

.. figure:: ../images/JTAG_Reset.png
   :alt: JTAG Reset Pin Example
   :width: 500px
   :align: center

   JTAG Reset Pin Example

.. note::

    Before proceeding, ensure that your machine has the necessary drivers and a terminal emulator (`MobaXterm <https://mobaxterm.mobatek.net/download.html>`_, `Tera Term <https://teratermproject.github.io/index-en.html>`_, etc.) installed.

    The serial communication between the **Windows PC** and **RZ/V2H RDK** requires: `FTDI Virtual COM Port (VCP) driver <https://ftdichip.com/drivers/vcp-drivers/>`_

    Download and install the Windows version (`.exe`).

.. important::

    The power supply for the RZ/V2H RDK should satisfy the maximum requirement of 24V / 5A.

    The common DC power adapter specifications are:

    - DC power adapter 12V, 2A. (Included in the package)

    - DC power adapter 24V, 1A.

Common hardware setup
^^^^^^^^^^^^^^^^^^^^^^

The following image shows the common hardware setup for both boot modes:

.. figure:: ../images/hardware_connect.png
   :alt: Common Hardware Setup
   :width: 700px
   :align: center

   Common Hardware Setup

The setup includes:

- Power supply connection
- Serial connection for terminal access
- Ethernet connection for network access

Option 1: microSD card boot mode
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For **microSD card boot mode**, the required bootloaders components are already included in the flashed microSD card image.

On the RZ/V2H RDK, configure the **DSW1** switches as shown below:


.. figure:: ../images/DSW1_SD.png
   :alt: DSW1 microSD card Boot Mode
   :width: 500px
   :align: center

   DSW1 microSD card Boot Mode

After that, insert the microSD card and connect the power supply to the board.

Open a terminal emulator (e.g., **Tera Term**) and connect to the **COM** port of the board.

The COM port settings are listed below:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - **Item**
     - **Value**
   * - **Baud rate**
     - **115200**
   * - Data
     - 8-bit
   * - Parity
     - None
   * - Stop
     - 1-bit
   * - Flow control
     - None
   * - Transmit delay
     - 0 msec/char

The board will start the boot process.

.. tip::

   - If no serial output is shown at all, try the :ref:`JTAG reset tip <jtag_reset_tip>`.
   - If the U-Boot prompt appears but the system does not boot correctly, reset the U-Boot environment variables:

   .. code-block:: bash

      env default -a
      saveenv
      boot

If you intend to use **microSD card boot mode only**, proceed to :ref:`first time boot setup <first_time_boot_setup>` to complete the setup.

Option 2: xSPI boot mode
^^^^^^^^^^^^^^^^^^^^^^^^

Follow the instructions below to set up the board.

1. **Install Terminal Emulator**

   .. note::
      If already installed, skip this step.

   - **Terminal Emulator:** `Tera Term`_

.. _Tera Term: https://teratermproject.github.io/index-en.html

   - **Operating Environment:** Windows

.. _write_bootloaders_to_board:

2. **Write Bootloaders to the Board**

Copy the bootloaders files to your Windows PC.

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - **File Name**
     - **Description**
   * - Flash_Writer_SCIF_RZV2H_DEV_INTERNAL_MEMORY.mot
     - Flash writer for RZ/V2H (used in SCIF download mode)
   * - bl2_bp_spi-rzv2h-rdk.srec
     - Boot loader stage 2 binary
   * - fip-rzv2h-rdk.srec
     - Firmware Image Package for RZ/V2H

- Connect the **Windows PC** and **Board** using a **USB-to-microUSB** cable.
- Change the **DSW1** setting to **Boot Mode 3 (SCIF download)**.

  .. figure:: ../images/DSW1_SCIF.png
     :alt: DSW1 SCIF Download Mode
     :width: 500px
     :align: center

     DSW1 SCIF Download Mode

- Connect the power cable.
- Open **Tera Term** and configure:

  **Setup → Terminal:**

  .. list-table::
     :header-rows: 1
     :widths: 40 60

     * - **Item**
       - **Value**
     * - New-line
       - Receive: Auto / Transmit: CR

  **Setup → Serial Port:**

  .. list-table::
     :header-rows: 1
     :widths: 40 60

     * - **Item**
       - **Value**
     * - Baud rate
       - **115200**
     * - Data
       - 8-bit
     * - Parity
       - None
     * - Stop
       - 1-bit
     * - Flow control
       - None
     * - Transmit delay
       - 0 msec/char

- Open **File → Send file...** and send the **Flash Writer** file (.mot) as text.

  If the following message is displayed, the file transfer was successful::

      Flash writer for RZ/V2x Series Vx.xx xxx.xx,20xx
      Product Code : RZ/V2x

- Next, enter the ``XLS2`` command in the terminal::

      > XLS2
      ===== Qspi writing of RZ/V2x Board Command =============
      Load Program to Spiflash
      Writes to any of SPI address.
      Program size & Qspi Save Address
      ===== Please Input Program Top Address ============
      Please Input : H'

- Enter ``8101e00``. The log continues::

      Please Input : H'8101e00
      ===== Please Input Qspi Save Address ===
      Please Input : H'

- Enter ``00000``. The log continues::

      Please Input : H'00000
      please send ! ('.' & CR stop load)

- After the "please send!" message, open **File → Send file...** and send the `bl2_bp_spi-rzv2*.srec` file as text from the terminal software.

- If prompted to clear data, enter `y`::

      SPI Data Clear(H'FF) Check : H'00000000-0000FFFF, Clear OK?(y/n)

- The following log will be displayed. The end address depends on the version of IPL::

      Write to SPI Flash memory.
      ======= Qspi Save Information =================
      SpiFlashMemory Stat Address : H'00000000
      SpiFlashMemory End Address  : H'00036D17
      ===========================================================

- Enter ``XLS2`` on the terminal again to get the following messages::

      > XLS2
      ===== Qspi writing of RZ/V2x Board Command =============
      Load Program to Spiflash
      Writes to any of SPI address.
      Program size & Qspi Save Address
      ===== Please Input Program Top Address ============
      Please Input : H'

- Enter ``00000``. The log continues::

      Please Input : H'00000
      ===== Please Input Qspi Save Address ===
      Please Input : H'

- Enter ``60000``. The log continues::

      Please Input : H'60000
      please send ! ('.' & CR stop load)

- After the "please send!" message, open **File → Send file...** and send the `fip-rzv2*.srec` file as text from the terminal software.

- If prompted to clear data, enter ``y``::

      SPI Data Clear(H'FF) Check : H'00000000-0000FFFF, Clear OK?(y/n)

- The following log will be displayed. The end address depends on the version of IPL::

      Write to SPI Flash memory.
      ======= Qspi Save Information =================
      SpiFlashMemory Stat Address : H'00060000
      SpiFlashMemory End Address  : H'0011C2EE
      ===========================================================

- Power off the board and change DSW1 to configure the boot mode.

3. **Set up U-Boot Configuration**

   a. Insert the microSD card into the board.

   b. Change DSW1 to **Boot Mode 2 (xSPI boot)**:

      .. figure:: ../images/DSW1_xSPI.png
          :alt: DSW1 xSPI Boot
          :width: 500px
          :align: center

          DSW1 xSPI Boot Mode

   c. Connect the board to the PC using a **USB-to-microUSB** cable.

   d. Power on the board.

   e. Open the terminal emulator and connect to the **COM** port (same configuration as before).

   f. The board will boot.

.. tip::

   - If no serial output is shown at all, try the :ref:`JTAG reset tip <jtag_reset_tip>`.
   - If the U-Boot prompt appears but the system does not boot correctly, reset the U-Boot environment variables:

   .. code-block:: bash

      env default -a
      saveenv
      boot

.. _first_time_boot_setup:

First Time Boot Setup
---------------------

The default user credentials for the provided Ubuntu images are as follows:

.. list-table:: Default Login Information
   :header-rows: 1
   :widths: 35 25 25

   * - **Image Type**
     - **Username**
     - **Password**
   * - Ubuntu 24.04 Server
     - ubuntu
     - ubuntu

After powering on the board **for the first time**, connect to the serial console and check the boot log to verify that Ubuntu boots successfully.

.. note::

   The next operation is required **only once**, immediately after flashing the new root filesystem.

Connect an Ethernet cable to the board and run:

.. code-block:: bash

   # Check network
   ping 8.8.8.8 -c 3
   ping bing.com -c 3

1. Perform apt update and resize the microSD card:

   .. code-block:: bash

      sudo apt update
      sudo apt install parted
      sudo parted /dev/mmcblk0 resizepart 2 100%
      sudo resize2fs /dev/mmcblk0p2

   .. note::

      The above commands resize the second partition to utilize the full capacity of the microSD card.

      If you are using a different partition layout, please adjust the command accordingly (e.g., change the partition number).

2. Install the ROS 2 Jazzy:

   We provide the script called: `common_utils/ros2_utils/apt_install_ros2.sh <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/utility/common_utils/-/blob/main/ros2_utils/apt_install_ros2.sh?ref_type=heads>`_ to install ROS 2 Jazzy packages on the target system.

   Please download the script to the target system and run the following commands:

   .. code-block:: bash

      chmod +x apt_install_ros2.sh
      sudo ./apt_install_ros2.sh

   For more details about the ROS 2 Jazzy installation, please refer to the `ROS 2 Jazzy Installation Guide <https://docs.ros.org/en/jazzy/Installation.html>`_.

3. (Optional) Add the ROS 2 environment setup to ``.bashrc``:

   .. code-block:: bash

      echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
      source ~/.bashrc

This completes the **Quick Start Guide for RZ/V2H RDK**.

Reference
---------

- Advanced Boot Options (xSPI):
  `Renesas RZ/V AI SDK Developer Guide <https://renesas-rz.github.io/rzv_ai_sdk/latest/dev_guide.html#D3>`_
- Balena Etcher Official Website:
  `https://www.balena.io/etcher <https://www.balena.io/etcher>`_
