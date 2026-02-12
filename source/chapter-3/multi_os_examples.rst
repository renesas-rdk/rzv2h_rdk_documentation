RZV2H RDK Multi-OS Example Packages
-----------------------------------------------------

This section contains a collection of Multi-OS packages designed for applications on Renesas RZ/V MPU platforms, specifically targeting the RZ/V2H RDK.

These packages provide practical examples demonstrating how to operate and integrate Multi-OS environments on the RZ/V2H RDK, helping developers understand cross-core communication, system setup, and interaction between Linux and RTOS components.

Additionally, a demo showcasing Micro-ROS (uROS) running on the real-time CR8 core is supported. It demonstrates the implementation of Micro-ROS on an MCU-class core within the device.

Hardware supported
^^^^^^^^^^^^^^^^^^^^^^^^^

- Platform: Renesas RZ/V2H MPU

- Development Board: RZ/V2H RDK (SoC: R9A09G057H44GBG)

Software supported
^^^^^^^^^^^^^^^^^^^^^^^^^

- Target Operating System: Ubuntu 24.04

- RZ/V Multi-OS Package version 3.2

- RZ/V FSP version 3.1

- Micro XRCE-DDS Agent version 3.0.1

- Micro ROS Client Jazzy

- ROS2 Distribution: ROS2 Jazzy

Package Specification
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - **Package**
     - **Target Core**
     - **Purpose / Description**
   * - Micro XRCE-DDS Agent
     - CA55 (Linux)
     - Provides the middleware agent running on the Linux core (CA55) for communication

       between Micro-ROS clients (running on RTOS CR8_0 core) and the ROS 2 environment on Linux via the XRCE-DDS protocol.
   * - RZ/V2H RDK Blinky
     - CM33 (RTOS)
     - A simple LED blinking demo running on the CM33 core that verifies basic GPIO functionality

       and confirms that the RTOS environment is running correctly on the RZ/V2H RDK.
   * - RZ/V2H RDK CM33

       RPMsg Linux-RTOS Demo
     - CM33 (RTOS)
     - Demonstrates inter-core communication (RPMsg) between the Linux core (CA55) and the CM33 RTOS core,

       showing message exchange and synchronization.
   * - RZ/V2H RDK CR8 Core0

       RPMsg Linux-RTOS Demo
     - CR8_0 (RTOS)
     - Demonstrates RPMsg-based communication between the Linux core (CA55) and the CR8_0 real-time core,

       validating message passing and core coordination.
   * - RZ/V2H RDK CR8 Core0

       RPMsg Micro-ROS Demo
     - CR8_0 (RTOS)
     - Showcases Micro-ROS running on the CR8_0 real-time core, integrating the uROS client

       with the custom RPMsg transport layer for communication with Linux and ROS 2.

Installation Guide
^^^^^^^^^^^^^^^^^^^^^^^^^^

To set up and use the Multi-OS example packages on the RZ/V2H RDK, follow the steps below:

Firmware Code for CM33/CR8
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This section describes how to build and flash the firmware for the CM33/CR8 core using e² studio and the provided sample project.

1. Clone the CM33/CR8 project into your host machine.
2. Open **e² studio** and import the above project using **"Import Existing Project"**.
3. Open the configuration file.
4. Click **"Generate"** to generate configuration files.
5. Click **"Build Project"** and wait for the build process to complete.
6. Flash the firmware to the CM33/CR8 core using your preferred method (e.g., J-Link).

.. important::

   The preceding project for all of CR8_0 packages is ``RZ/V2H RDK CM33 RPMsg Linux-RTOS Demo``.
   Please import this CM33 project into your e² studio workspace and build it before using the CR8_0 packages.

**Special Note for ``RZ/V2H RDK CR8 Core0 RPMsg Micro-ROS Demo`` Package**

1. This demo includes a prebuilt libmicroros library. If you want to rebuild this library, use this project on the Ubuntu machine and perform the following steps:

   Go to **Project → Properties → C/C++ Build → Settings → Build Steps** tab and in **Pre-build steps**, add the command:

   .. code-block:: bash

      cd ../micro_ros_renesas2estudio_component/library_generation && ./library_generation.sh "${cross_toolchain_flags}"

   Then click **Apply and Close** → **Build the project**.

2. The code includes a 30-second delay (in ``main_task_entry.c`` line 168) before initializing MCU tasks to prevent issues with PWM and I2C pin control on the CR8 core.
   By default, this delay is commented out.

   - If flashing via **J-Link**, this delay can be skipped.
   - However, when invoking the firmware from **U-Boot**, please enable this delay.

Cross Compile the Micro XRCE-DDS Agent
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This section describes how to deploy and build the custom OpenAMP Micro-ROS agent application running on the CA55 core of the RZ/V2H platform.

**Prerequisites:** The Linux local host machine must have the Yocto SDK installed.
It is recommended to use the :ref:`ROS2 cross-build Docker container <docker_sdk_setup>`, since the environment is already fully set up.

1. Clone the ``Micro-XRCE-DDS-Agent`` to your local machine.
2. The custom OpenAMP agent source code is located at: ``Micro-XRCE-DDS-Agent/examples/custom_agent``.
3. Navigate to the directory ``Micro-XRCE-DDS-Agent/``.
4. Build the project:

   .. code-block:: bash

      mkdir build && cd build

      # We have to disable the UAGENT_LOGGER_PROFILE because the current SDK cannot compile spdlog.
      # For native builds, we can remove the DUAGENT_LOGGER_PROFILE flag.
      cmake .. -DCMAKE_TOOLCHAIN_FILE=../coretexa55-toolchain.cmake \
               -DUAGENT_BUILD_USAGE_EXAMPLES=ON \
               -DUAGENT_LOGGER_PROFILE=OFF \
               -DCMAKE_BUILD_TYPE=Release \
               -DCMAKE_INSTALL_PREFIX=./arm64-install

      make -j$(nproc)
      make install

5. Wait until the build process completes.
6. Deploy the output to the target board by copying the output artifact:

   - On the **Host machine**:

     .. code-block:: bash

        # Copy the built CustomXRCEAgent binary to the arm64-install folder for deployment
        $ cp ./examples/custom_agent/CustomXRCEAgent arm64-install/bin

        # Compress the arm64-install folder
        $ tar -cf libdds_agent.tar.bz2 -C arm64-install .

   - Then copy ``libdds_agent.tar.bz2`` to the target board using **scp** or another file transfer method.

   - On the **Target machine**:

     .. code-block:: bash

        # Extract the archive
        $ mkdir tmp-install
        $ sudo tar -xf libdds_agent.tar.bz2 -C tmp-install

        # Install libdds_agent to the system
        $ cd tmp-install
        $ sudo cp -r * /usr/local/
        $ sudo ldconfig

Usage Guide
^^^^^^^^^^^^^^^^^^^^^^^^^^

To run the Multi-OS example packages on the RZ/V2H RDK, follow the instructions below for each package.

RPMsg Linux-RTOS Demo
~~~~~~~~~~~~~~~~~~~~~~~~~~~

This demo behaves identically to the version released in the **RZ/V Multi-OS Package**.
For more details, refer to the `RZ/V2H Quick Start Guide <https://www.renesas.com/en/document/qsg/rzv2h-quick-start-guide-rzv-multi-os-package?r=1570181>`_ for the RZ/V Multi-OS Package.

1. Flash the ``RPMsg Linux-RTOS Demo`` firmware to the target board.
2. On the board's terminal, run the ``rpmsg_sample_client`` with sudo privilege:

   .. code-block:: bash

      root@localhost:~# rpmsg_sample_client

   Example output:

   .. code-block:: bash

      root@localhost:~# rpmsg_sample_client
      [694] proc_id:0 rsc_id:0 mbx_id:1
      metal: warning:   metal_linux_irq_handling: Failed to set scheduler: -1.
      metal: info:      metal_uio_dev_open: No IRQ for device 10480000.mbox-uio.
      [694] Successfully probed IPI device
      ...

3. Based on the firmware you have flashed, select the corresponding option below and press **Enter** when prompted.

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

   .. note::
      Ensure that the firmware on your target board matches the selected option to avoid communication errors.

4. Example output:

   - If the Input Option is ``1``:

     .. code-block:: bash

        [CM33]  received payload number 469 of size 486
        [CM33] sending payload number 470 of size 487
        [828] cond signal 1 sync:0
        ...

   - If the Input Option is ``4``:

     .. code-block:: bash

        [CR8_0 ]  received payload number 469 of size 486
        [CR8_0 ] sending payload number 470 of size 487
        [790] cond signal 2 sync:0
        ...

5. By typing ``e``, the sample program should terminate with the message shown below:

   .. code-block:: bash

      please input
      > e
      [xxx] 42f00000.rsctbl closed
      [xxx] 43000000.vring-ctl0 closed
      ...

uROS and Custom Micro XRCE-DDS Agent
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This section describes how to run the Micro-ROS Client demo and the custom XRCE-DDS RPMsg Agent.

1. (Optional) Connect the UART-to-TTL cable to **GPIO pin 40** on the RDK board to view log output from the CR8 core over UART channel 5 (P72–TXD5 / P73–RXD5).
2. Flash the ``RZ/V2H RDK CR8 Core0 RPMsg Micro-ROS Demo`` firmware to the target board.
3. On the board's terminal, run the CustomXRCEAgent with sudo privilege:

   .. code-block:: bash

      root@localhost:~# CustomXRCEAgent

   Example output:

   .. code-block:: bash

      root@localhost:~# CustomXRCEAgent
      [787] proc_id:0 rsc_id:0 mbx_id:1
      metal: warning:   metal_linux_irq_handling: Failed to set scheduler: -1.
      ...

4. On another terminal, use the following ROS 2 commands to verify communication:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      ros2 topic list
      ros2 topic echo /cr8/heartbeat

**Behavior:**

- The CR8 firmware creates the topic ``/cr8/heartbeat`` and continuously publishes data to it.
- The custom Micro XRCE-DDS Agent makes this topic available in the ROS 2 environment running on the CA55 core.
- From the CA55 core, you can subscribe to and retrieve data from the ``/cr8/heartbeat`` topic.

Example output:

.. code-block:: bash

   rz@localhost:~$ source /opt/ros/jazzy/setup.bash
   rz@localhost:~$ ros2 topic list
   /cr8/heartbeat
   /parameter_events
   /rosout
   rz@localhost:~$ ros2 topic echo /cr8/heartbeat
   data: 328
   ---
   data: 329
   ---
   data: 330
   ...

Troubleshooting
^^^^^^^^^^^^^^^^^^^^^^^^^^

1. **Can't open the configuration.xml of CR8 e² studio project?**

   Confirm the RZ/V FSP version is 3.1 and import the **CM33 project** into the workspace and build it first, then try opening the CR8 project again.

2. **The behavior of the RPMsg demo is strange?**

   Restart the **RDK board** to reset the RPMsg endpoint.

3. **Unknown status of the micro-ROS demo?**

   Use a **USB-to-TTL** module to read logs from the UART interface of the RDK board (baud rate: **115200**).

   You should see output similar to the following:

   .. code-block:: bash

      [CR8] Start main_task_entry
      [CR8] RPMsg endpoint ready
      [CR8] Heartbeat publisher ready on /cr8/heartbeat
      [CR8] Heartbeat #50 (uptime=16061 ms)

   You should run the ``CustomXRCEAgent`` only after the message ``[CR8] RPMsg endpoint ready`` appears on the UART log.

4. **Can't flash the firmware over J-Link?**

   Make sure you are using the correct **J-Link firmware version** and that **DIP switch SW1-6** is turned **ON**.
