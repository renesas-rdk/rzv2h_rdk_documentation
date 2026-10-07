RZ/V2H RDK Multi-OS Example Packages
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section contains a collection of Multi-OS packages designed for applications on Renesas RZ/V MPU platforms, specifically targeting the RZ/V2H RDK.

These packages provide practical examples demonstrating how to operate and integrate Multi-OS environments on the RZ/V2H RDK, helping developers understand cross-core communication, system setup, and interaction between Linux and RTOS components.

The examples cover two RPMsg communication scenarios:

-  **Linux-RTOS**: The Linux core (CA55) exchanges messages with an RTOS core (CM33, CR8_0, or CR8_1).

-  **RTOS-RTOS**: Two RTOS cores exchange messages without Linux involvement (CM33 with CR8_0/CR8_1, or CR8_0 with CR8_1).

Additionally, a demo showcasing micro-ROS (uROS) running on the real-time CR8 core is supported. It demonstrates the implementation of micro-ROS on an MCU-class core within the device.

Hardware Supported
""""""""""""""""""

-  Platform: Renesas RZ/V2H MPU

-  Development Board: RZ/V2H RDK (SoC: R9A09G057H44GBG)

Software Supported
""""""""""""""""""

-  Target RZ/V2H RDK image: ``ubuntu-24.04-server-arm64-rzv2h-rdk-<ver>.img.xz`` (``<ver>``: ``ver1`` or ``ver101``)

-  RZ/V Multi-OS Package version 4.2

-  RZ/V FSP version 4.2

-  Micro XRCE-DDS Agent version 3.0.1

-  micro-ROS Client Jazzy

-  ROS 2 Distribution: ROS 2 Jazzy

- Example packages: See the `Package Specification`_ section below for details.

Package Specification
"""""""""""""""""""""

The following table provides an overview of the Multi-OS example packages, including their target cores, operating systems, and main functionalities.

You can access the source code and detailed documentation for each package through the provided links.

.. important::

   Use the shortest path possible. If you place the project in a deeply nested path, you may encounter issues when building the project with e² studio.

   The recommended workspace path for e² studio on Windows is ``C:\rzv2h_e2_workspace``.

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - **Package**
     - **Target Core**
     - **Purpose / Description**
   * - `Micro XRCE-DDS Agent <https://github.com/renesas-rdk/Micro-XRCE-DDS-Agent>`_
     - CA55 (Linux)
     - Provides the middleware agent running on the Linux core (CA55) for communication

       between micro-ROS clients (running on RTOS CR8_0 core) and the ROS 2 environment on Linux via the XRCE-DDS protocol.
   * - `RZ/V2H RDK Blinky <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/rzv2h_rdk_blinky>`_
     - CM33 (RTOS)
     - A simple LED blinking demo running on the CM33 core that verifies basic GPIO functionality

       and confirms that the RTOS environment is running correctly on the RZ/V2H RDK.
   * - `RZ/V2H RDK CM33 RPMsg Linux-RTOS Demo <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/rzv2h_rdk_cm33_rpmsg_linux_rtos_demo>`_
     - CM33 (RTOS)
     - Demonstrates inter-core communication (RPMsg) between the Linux core (CA55) and the CM33 RTOS core,

       showing message exchange and synchronization.
   * - `RZ/V2H RDK CR8 Core0 RPMsg Linux-RTOS Demo <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/rzv2h_rdk_cr8_core0_rpmsg_linux_rtos_demo>`_
     - CR8_0 (RTOS)
     - Demonstrates RPMsg-based communication between the Linux core (CA55) and the CR8_0 real-time core,

       validating message passing and core coordination.
   * - `RZ/V2H RDK CR8 Core1 RPMsg Linux-RTOS Demo <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/rzv2h_rdk_cr8_core1_rpmsg_linux_rtos_demo>`_
     - CR8_1 (RTOS)
     - Demonstrates RPMsg-based communication between the Linux core (CA55) and the CR8_1 real-time core,

       validating message passing and core coordination.
   * - `RZ/V2H RDK CR8 Core0 RPMsg Micro-ROS Demo <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/rzv2h_rdk_cr8_core0_rpmsg_microros_demo>`_
     - CR8_0 (RTOS)
     - Showcases micro-ROS running on the CR8_0 real-time core, integrating the uROS client

       with the custom RPMsg transport layer for communication with Linux and ROS 2.
   * - `RZ/V2H RDK CM33 RPMsg RTOS-RTOS Demo <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/rzv2h_rdk_cm33_rpmsg_rtos_rtos_demo>`_
     - CM33 (RTOS)
     - Master side of the CM33-CR8 RPMsg echo test. The CM33 core sends payloads to the CR8_0 or CR8_1 core,

       validates the echoed data, and prints the test result through SEGGER RTT.
   * - `RZ/V2H RDK CR8 Core0 RPMsg RTOS-RTOS Demo <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/rzv2h_rdk_cr8_core0_rpmsg_rtos_rtos_demo>`_
     - CR8_0 (RTOS)
     - Remote (slave) side of the CM33-CR8_0 echo test. Can also be configured as the master side

       of the CR8_0-CR8_1 echo test.
   * - `RZ/V2H RDK CR8 Core1 RPMsg RTOS-RTOS Demo <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/rzv2h_rdk_cr8_core1_rpmsg_rtos_rtos_demo>`_
     - CR8_1 (RTOS)
     - Remote (slave) side of the CM33-CR8_1 or CR8_0-CR8_1 echo test.

Installation Guide
""""""""""""""""""

To set up and use the Multi-OS example packages on the RZ/V2H RDK, follow the steps below:

Firmware Code for CM33/CR8
~~~~~~~~~~~~~~~~~~~~~~~~~~

This section describes how to build and flash the firmware for the CM33/CR8 core using e² studio and the provided sample project.

#. Clone the CM33/CR8 project into your host machine.

#. Open **e² studio** and import the above project using **"Import Existing Project"**.

#. Open the configuration file.

#. Click **"Generate"** to generate configuration files.

#. Click **"Build Project"** and wait for the build process to complete.

#. Flash the firmware to the CM33/CR8 core using your preferred method (e.g., J-Link).

.. important::

   Every CR8 project requires a preceding project:
   CR8 Core0 projects use a CM33 project, and CR8 Core1 projects use the matching CR8 Core0 project.
   Import the preceding project into your e² studio workspace and build it before building the CR8 project.

   .. list-table::
      :header-rows: 1
      :widths: 50 50

      * - **CR8 Project**
        - **Preceding Project**
      * - ``RZ/V2H RDK CR8 Core0 RPMsg Linux-RTOS Demo``

          ``RZ/V2H RDK CR8 Core0 RPMsg Micro-ROS Demo``
        - ``RZ/V2H RDK CM33 RPMsg Linux-RTOS Demo``
      * - ``RZ/V2H RDK CR8 Core1 RPMsg Linux-RTOS Demo``
        - ``RZ/V2H RDK CR8 Core0 RPMsg Linux-RTOS Demo``
      * - ``RZ/V2H RDK CR8 Core0 RPMsg RTOS-RTOS Demo``
        - ``RZ/V2H RDK CM33 RPMsg RTOS-RTOS Demo``
      * - ``RZ/V2H RDK CR8 Core1 RPMsg RTOS-RTOS Demo``
        - ``RZ/V2H RDK CR8 Core0 RPMsg RTOS-RTOS Demo``

**Special Note for** ``RZ/V2H RDK CR8 Core0 RPMsg Micro-ROS Demo`` **Package**

#. This demo includes a pre-built libmicroros library. If you want to rebuild this library, use this project on the **Ubuntu host PC machine** and perform the following steps:

   Go to **Project → Properties → C/C++ Build → Settings → Build Steps** tab and in **Pre-build steps**, add the command:

   .. code-block:: bash

      cd ../micro_ros_renesas2estudio_component/library_generation && ./library_generation.sh "${cross_toolchain_flags}"

   Then click **Apply and Close** → **Build the project**.

#. The code includes a 30-second delay (in ``main_task_entry.c`` line 243) before initializing MCU tasks to prevent issues with PWM and I2C pin control on the CR8 core.
   By default, this delay is commented out.

   -  If flashing via **J-Link**, this delay can be skipped.
   -  However, when invoking the firmware from **U-Boot**, enable this delay.

Usage Guide
"""""""""""

RPMsg Linux-RTOS Demo
~~~~~~~~~~~~~~~~~~~~~

This demo behaves identically to the version released in the **RZ/V Multi-OS Package**.

For more details, refer to the `RZ/V2H Quick Start Guide: Section 4.4 CM33/CR8 Sample Program Invocation for communicating with Linux <https://www.renesas.com/en/document/qsg/rzv2h-quick-start-guide-rzv-multi-os-package?r=1570181>`_ for the RZ/V Multi-OS Package.

#. Flash the ``RPMsg Linux-RTOS Demo`` firmware to the target board.

#. On the board's terminal, run the ``rpmsg_sample_client`` with sudo privilege:

   .. code-block:: bash

      sudo rpmsg_sample_client

   Example output:

   .. code-block:: bash

      [694] proc_id:0 rsc_id:0 mbx_id:1
      metal: warning:   metal_linux_irq_handling: Failed to set scheduler: -1.
      metal: info:      metal_uio_dev_open: No IRQ for device 10480000.mbox-uio.
      [694] Successfully probed IPI device
      ...

#. Based on the firmware you have flashed, select the corresponding option below and press **Enter** when prompted.

   .. list-table::
      :header-rows: 1
      :widths: 10 25 65

      * - **Input Option**
        - **Target Core / Firmware**
        - **Description**
      * - **1**
        - CM33 (Linux <=> RTOS RPMsg Demo)
        - Select this if you have flashed the ``RZ/V2H RDK CM33 RPMsg Linux-RTOS Demo`` firmware.
      * - **4**
        - CR8_0 (Linux <=> RTOS RPMsg Demo)
        - Select this if you have flashed the ``RZ/V2H RDK CR8 Core0 RPMsg Linux-RTOS Demo`` firmware.
      * - **6**
        - CR8_1 (Linux <=> RTOS RPMsg Demo)
        - Select this if you have flashed the ``RZ/V2H RDK CR8 Core1 RPMsg Linux-RTOS Demo`` firmware.

   By default, the CM33 firmware uses RPMsg channel 0 and the CR8 firmware uses RPMsg channel 1.
   The options above match these defaults. For the full list of options, see the menu printed by ``rpmsg_sample_client``:

   .. code-block:: text

      1. communicate with CM33      ch0
      2. communicate with CM33      ch1
      3. communicate with CR8 core0 ch0
      4. communicate with CR8 core0 ch1
      5. communicate with CR8 core1 ch0
      6. communicate with CR8 core1 ch1
      7. communicate with CM33 ch0 and CR8 core0 ch1
      8. communicate with CM33 ch0 and CR8 core1 ch1
      9. communicate with CR8 core0 ch0 and CR8 core1 ch1

      e. exit

   .. note::

      Ensure that the firmware on your target board matches the selected option to avoid communication errors.

#. Example output:

   -  If the Input Option is ``1``:

      .. code-block:: bash

         [CM33]  received payload number 469 of size 486
         [CM33] sending payload number 470 of size 487
         [828] cond signal 1 sync:0
         ...

   -  If the Input Option is ``4``:

      .. code-block:: bash

         [CR8_0 ]  received payload number 469 of size 486
         [CR8_0 ] sending payload number 470 of size 487
         [790] cond signal 2 sync:0
         ...

#. By typing ``e``, the sample program should terminate with the message shown below:

   .. code-block:: bash

      please input
      > e
      [xxx] 42f00000.rsctbl closed
      [xxx] 43000000.vring-ctl0 closed
      ...

.. _multi_os_remoteproc:

Remoteproc Support
~~~~~~~~~~~~~~~~~~

Besides J-Link and U-Boot, the CM33/CR8 firmware can be loaded and started from Linux by using the **remoteproc** framework.
Remoteproc support for the CM33 and CR8 cores is enabled by default in the RZ/V2H RDK image.

Remoteproc is supported by the Linux-RTOS projects:

-  ``RZ/V2H RDK CM33 RPMsg Linux-RTOS Demo``
-  ``RZ/V2H RDK CR8 Core0 RPMsg Linux-RTOS Demo``
-  ``RZ/V2H RDK CR8 Core1 RPMsg Linux-RTOS Demo``
-  ``RZ/V2H RDK CR8 Core0 RPMsg Micro-ROS Demo``

**Build the firmware for remoteproc**

You can use one of the projects above as is, or as a base project for your own application.

#. Import the base project into e² studio as described in `Firmware Code for CM33/CR8`_.

   -  For CM33, use ``RZ/V2H RDK CM33 RPMsg Linux-RTOS Demo``.
   -  For CR8, use ``RZ/V2H RDK CR8 Core0 RPMsg Linux-RTOS Demo`` or ``RZ/V2H RDK CR8 Core1 RPMsg Linux-RTOS Demo``.

#. (Optional) Rename the project. In **Project Explorer**, right-click the project name and select **Rename...**.

#. (Optional) Implement your application code in the ``main_task_entry()`` function in ``src/main_task_entry.c``.

   Keep the calls to ``init_system()`` and ``platform_init()`` at the beginning of the function, then add your code after them.

#. Enable remoteproc by setting the ``ENABLE_REMOTEPROC`` macro in ``src/platform_info.h`` to ``1``:

   .. code-block:: c

      #define ENABLE_REMOTEPROC        (1U)

#. Build the project. The ELF file (for example, ``rzv2h_rdk_cm33_rpmsg_linux_rtos_demo.elf``) is generated in the ``Debug`` or ``Release`` folder of the project.

#. Copy the ELF file to the ``/lib/firmware`` folder on the target board, for example, by using **scp**:

   .. code-block:: bash

      # On the host machine
      scp Debug/rzv2h_rdk_cm33_rpmsg_linux_rtos_demo.elf <user>@<board_ip>:/tmp/

      # On the target board
      sudo cp /tmp/rzv2h_rdk_cm33_rpmsg_linux_rtos_demo.elf /lib/firmware/

   Replace ``<user>`` and ``<board_ip>`` with the user name and IP address of your board.

**Run the firmware from remoteproc**

#. Boot up Linux on the RZ/V2H RDK.

#. Find the remoteproc instance of the target core:

   .. code-block:: bash

      cat /sys/class/remoteproc/remoteproc*/name

   The following table shows the expected mapping:

   .. list-table::
      :header-rows: 1
      :widths: 30 70

      * - **Core**
        - **Remoteproc instance**
      * - CM33
        - ``/sys/class/remoteproc/remoteproc0``
      * - CR8_0
        - ``/sys/class/remoteproc/remoteproc1``
      * - CR8_1
        - ``/sys/class/remoteproc/remoteproc2``

#. Specify the firmware file to be loaded. Use the file name of the ELF file copied to ``/lib/firmware``:

   .. code-block:: bash

      echo rzv2h_rdk_cm33_rpmsg_linux_rtos_demo.elf | sudo tee /sys/class/remoteproc/remoteproc0/firmware

#. Start the core:

   .. code-block:: bash

      echo start | sudo tee /sys/class/remoteproc/remoteproc0/state

   If the CM33 firmware starts successfully, the kernel log (``dmesg``) shows messages similar to the following:

   .. code-block:: text

      remoteproc remoteproc0: powering up cm33
      remoteproc remoteproc0: Booting fw image rzv2h_rdk_cm33_rpmsg_linux_rtos_demo.elf, size xxx
      rproc-virtio rproc-virtio.2.auto: assigned reserved memory node vdev0buffer@0x43200000
      rproc-virtio rproc-virtio.2.auto: registered virtio0 (type 7)
      remoteproc remoteproc0: remote processor cm33 is now up

   For CR8_0, use ``remoteproc1``. The log shows ``powering up cr8_0`` and ``remote processor cr8_0 is now up``.

#. Run the Linux side application, for example ``rpmsg_sample_client`` as described in `RPMsg Linux-RTOS Demo`_.

#. To stop the core, run:

   .. code-block:: bash

      echo stop | sudo tee /sys/class/remoteproc/remoteproc0/state

.. note::

   The firmware files are loaded from ``/lib/firmware``. When you update the firmware, replace the ELF file in this folder and restart the core.

RPMsg RTOS-RTOS Demo
~~~~~~~~~~~~~~~~~~~~

This demo runs an RPMsg echo test between two RTOS cores. Linux is not involved in the communication.

-  The **master** core sends payloads of increasing size (each byte is filled with ``0xA5``).
-  The **slave** core echoes every payload back to the master.
-  The master validates the echoed data and prints the test result.

Two communication modes are supported:

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - **Master**
     - **Slave**
     - **Firmware**
   * - CM33
     - CR8_0 or CR8_1
     - ``RZ/V2H RDK CM33 RPMsg RTOS-RTOS Demo`` and

       ``RZ/V2H RDK CR8 Core0 RPMsg RTOS-RTOS Demo`` (or ``RZ/V2H RDK CR8 Core1 RPMsg RTOS-RTOS Demo``)
   * - CR8_0
     - CR8_1
     - ``RZ/V2H RDK CR8 Core0 RPMsg RTOS-RTOS Demo`` and

       ``RZ/V2H RDK CR8 Core1 RPMsg RTOS-RTOS Demo``

**Configure the communication mode**

The default configuration is **CM33 (master) with CR8_0 (slave)**. To change it, edit the macros below in ``src/platform_info.h`` of each project, then rebuild the projects.

-  **CM33 with CR8_0 or CR8_1**:

   In the CM33 project, set ``TO_CR8_CORE`` to the target CR8 core:

   .. code-block:: c

      #define TO_CR8_CORE             (0)     /* 0: CR8 core0, 1: CR8 core1 */

   In the CR8 project, keep the default communication mode:

   .. code-block:: c

      #define RPMSG_COMMUNICATION_MODE        CM33_MASTER_CR8_SLAVE

-  **CR8_0 with CR8_1**:

   In **both** the CR8_0 and CR8_1 projects, set the communication mode as follows:

   .. code-block:: c

      #define RPMSG_COMMUNICATION_MODE        CR8_CORE0_MASTER_CR8_CORE1_SLAVE

   In this mode, the CR8_0 core becomes the master and its log output is enabled automatically.

.. note::

   The test result is printed by the master core through **SEGGER RTT**. The slave core does not print any log.

   -  CM33 master: Logging is enabled by ``ENABLE_RTTVIEWER`` in the CM33 project's ``platform_info.h``.
   -  CR8_0 master: Logging is enabled automatically when ``RPMSG_COMMUNICATION_MODE`` is set to ``CR8_CORE0_MASTER_CR8_CORE1_SLAVE``.

**Run the demo**

#. Build the master and slave projects as described in `Firmware Code for CM33/CR8`_.

#. Load and run the **slave** firmware first, using an e² studio debug session over J-Link. Click **Resume** so that the slave core runs and creates its RPMsg endpoint.

#. Load and run the **master** firmware in the same way.

   .. important::

      In the debug configuration of the master project, **disable** the ``Reset at the beginning of connection`` option of the J-Link debugger. Otherwise, the reset at connection also resets the slave core that is already running, and the test does not start.

#. Open **SEGGER J-Link RTT Viewer** and connect to the master core (CM33 or CR8_0). The master core starts the echo test when the RPMsg endpoint of the slave core is ready.

#. Check the log in the RTT Viewer. Example output:

   .. code-block:: text

      1 - Send data to remote core, retrieve the echo and validate its integrity ..
      RPMSG service has created.
      sending payload number 0 of size 9
      echo test: sent : 9
       received payload number 0 of size 9
      sending payload number 1 of size 10
      echo test: sent : 10
       received payload number 1 of size 10
      ...
      ************************************
       Test Results: Error count = 0
      ************************************
      Quitting application .. Echo test end

   ``Error count = 0`` means that all echoed payloads matched the sent data.

   If you can't use the SEGGER RTT Viewer, you can also check the log output through `release_rtt_reader <https://github.com/renesas-rdk/rzv_multi-os_samples/tree/main/release_rtt_reader>`_.

#. After the test ends, the master sends a shutdown message to the slave. Both cores wait 10 seconds and then reconnect to run the echo test again.

uROS and Custom Micro XRCE-DDS Agent
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This section describes how to run the micro-ROS Client demo and the custom XRCE-DDS RPMsg Agent.

Prerequisite
~~~~~~~~~~~~

Install ROS 2 Jazzy on the CA55 core (Ubuntu side). You can find and use the provided script here: `apt_install_ros2.sh <https://raw.githubusercontent.com/renesas-rdk/ros2_demo_workspace/refs/heads/main/common_utils/apt_install_ros2.sh>`_.

Quick installation steps:

.. code-block::

   wget https://raw.githubusercontent.com/renesas-rdk/ros2_demo_workspace/refs/heads/main/common_utils/apt_install_ros2.sh
   chmod +x apt_install_ros2.sh
   sudo ./apt_install_ros2.sh

For detailed installation instructions, refer to the official ROS 2 documentation: `ROS 2 Jazzy Installation Guide <https://docs.ros.org/en/jazzy/Installation.html>`_.

Cross-Compile the Micro XRCE-DDS Agent
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before running the micro-ROS demo on the CR8 core, you need to cross-compile the custom Micro XRCE-DDS Agent for the Linux CA55 core.

.. note::

   If you have already set up this Docker container (e.g., when building a ROS 2 application), you can use the same container to compile the Micro XRCE-DDS Agent without needing to set up a new environment.


#. Make sure your machine has the Docker Engine installed and running. You can use Windows, Linux, or macOS as your host machine.

   For the best experience, it is recommended to use a **Ubuntu 24.04 host machine** for cross-compilation.

   If you are using Windows or macOS, ensure that Docker Desktop is properly set up and configured to use Linux containers.

#. Clone the ``Micro-XRCE-DDS-Agent`` to your local machine:

   .. code-block:: bash

      git clone https://github.com/renesas-rdk/Micro-XRCE-DDS-Agent.git

#. Pull the Docker image provided by Renesas RDK for cross-compilation:

   .. code-block:: bash

      docker pull ghcr.io/renesas-rdk/rzv2h_ubuntu_xbuild:multiarch

#. Create a new Docker container:

   .. code-block:: bash

      docker run -it --rm -v /path/to/Micro-XRCE-DDS-Agent:/home/ubuntu/Micro-XRCE-DDS-Agent ghcr.io/renesas-rdk/rzv2h_ubuntu_xbuild:multiarch

   Replace ``/path/to/Micro-XRCE-DDS-Agent`` with the actual path on your host machine where the repository is located.

#. Navigate to the ``Micro-XRCE-DDS-Agent`` directory inside the Docker container:

   .. code-block:: bash

      cd Micro-XRCE-DDS-Agent/

#. Build the project:

   .. code-block:: bash

      mkdir build && cd build

      cmake .. -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAINS_WS/cross.cmake \
               -DUAGENT_BUILD_USAGE_EXAMPLES=ON \
               -DUAGENT_LOGGER_PROFILE=OFF \
               -DCMAKE_BUILD_TYPE=Release \
               -DCMAKE_INSTALL_PREFIX=./arm64-install

      make -j$(nproc)
      make install

   .. note::

      Note that the ``-DUAGENT_LOGGER_PROFILE`` is set to ``OFF`` due to incompatibility during cross-building.

      If you want to see the logs, build the libraries natively on the RZ/V2H RDK without the ``-DUAGENT_LOGGER_PROFILE=OFF`` flag.

#. Wait until the build process completes.

#. Deploy the output to the target board by copying the output artifact:

   -  On the **Host machine**:

      .. code-block:: bash

         # Copy the built CustomXRCEAgent binary to the arm64-install folder for deployment
         cp ./examples/custom_agent/CustomXRCEAgent arm64-install/bin

         # Compress the arm64-install folder
         tar -cf libdds_agent.tar.bz2 -C arm64-install .

   -  Then copy ``libdds_agent.tar.bz2`` to the target board using **scp** or another file transfer method.

   -  On the **Target machine**:

      .. code-block:: bash

         # Extract the archive
         mkdir tmp-install
         sudo tar -xf libdds_agent.tar.bz2 -C tmp-install

         # Install libdds_agent to the system
         cd tmp-install
         sudo cp -r * /usr/local/
         sudo ldconfig

#. (Optional) Connect the UART-to-TTL cable to **GPIO 40 pins** on the RDK board to view log output from the CR8 core over UART channel 5.

   .. list-table:: UART5 Interface Pins
      :header-rows: 1
      :widths: 20 20 40

      * - Pin Name
        - Function
        - Description
      * - P72 - GPIO14 - Pin number 8
        - TXD5
        - UART5 transmit data (TX) signal.
      * - P73 - GPIO15 - Pin number 10
        - RXD5
        - UART5 receive data (RX) signal.

#. Flash the ``RZ/V2H RDK CR8 Core0 RPMsg Micro-ROS Demo`` firmware to the target board.

#. On the board's terminal, run the CustomXRCEAgent with sudo privilege:

   .. code-block:: bash

      sudo CustomXRCEAgent

   Example output:

   .. code-block:: bash

      [787] proc_id:0 rsc_id:0 mbx_id:1
      metal: warning:   metal_linux_irq_handling: Failed to set scheduler: -1.
      ...

#. On another terminal, use the following ROS 2 commands to verify communication:

   .. note::
      Because the custom Micro XRCE-DDS Agent bridges the CR8 core and the ROS 2 environment on Linux by using ``sudo`` privileges, you must also run ROS 2 commands with ``sudo`` to view the relevant topics and messages.

   .. code-block:: bash

      sudo su
      source /opt/ros/jazzy/setup.bash
      ros2 topic list
      ros2 topic echo /cr8/heartbeat

**Behavior:**

- The CR8 firmware creates the topic ``/cr8/heartbeat`` and continuously publishes data to it.
- The custom Micro XRCE-DDS Agent makes this topic available in the ROS 2 environment running on the CA55 core.
- From the CA55 core, you can subscribe to and retrieve data from the ``/cr8/heartbeat`` topic.

.. code-block:: bash

   sudo su
   source /opt/ros/jazzy/setup.bash
   ros2 topic list

Example output:

.. code-block:: text

   /cr8/heartbeat
   /parameter_events
   /rosout

See the data from the CR8 core by running:

.. code-block:: bash

   ros2 topic echo /cr8/heartbeat

Example output:

.. code-block:: text

   data: 328
   ---
   data: 329
   ---
   data: 330
   ...

Troubleshooting
"""""""""""""""

#. **Can't see the heartbeat topic or data?**

   Make sure you have run the CustomXRCEAgent with ``sudo`` privileges and that the agent is running without errors.

   Check the terminal where you ran the CustomXRCEAgent for any error messages or logs that might indicate issues with the agent or communication.

   Also, ensure that you are running the ROS 2 commands with ``sudo`` privileges to access the topics bridged by the agent.

#. **Can't open the configuration.xml of CR8 e² studio project?**

   Confirm the RZ/V FSP version is 4.2 and import the **CM33 project** into the workspace and build it first, then try opening the CR8 project again.

#. **The behavior of the RPMsg demo is strange?**

   **Reboot the RDK board** to reset the RPMsg endpoint.

#. **Unknown status of the micro-ROS demo?**

   Use a **USB-to-TTL** module to read logs from the UART interface of the RDK board (baud rate: **115200**).

   You should see output similar to the following:

   .. code-block:: bash

      [CR8] Start main_task_entry
      [CR8] RPMsg endpoint ready
      [CR8] Heartbeat publisher ready on /cr8/heartbeat
      [CR8] Heartbeat #50 (uptime=16061 ms)

   You should run the ``CustomXRCEAgent`` only after the message ``[CR8] RPMsg endpoint ready`` appears on the UART log.

#. **Can't flash the firmware over J-Link?**

   Make sure you are using the correct **J-Link firmware version** and that **DIP switch SW1-6** is turned **ON**.
