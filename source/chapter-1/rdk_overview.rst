Overview
--------

WS125 Robotic Development Kit is a solution with Renesas new generation `RZ/V2H MPU <https://www.renesas.com/en/products/rz-v2h?tab=overview>`_ for AI application,
which has AI inference processing performance of up to 80 TOPS with multi-core CPU to run multiple OS
simultaneously for high performance AI image processing.

It is also equipped with many interfaces that make it suitable for development and integration into a variety of
robotic applications.

.. _rdk_board_versions:

Board Versions
^^^^^^^^^^^^^^

The RZ/V2H RDK is available in two versions, which differ in memory size. Identify the version by the label printed on the top side of the board, below the fan area.

.. list-table::
   :header-rows: 1
   :widths: 20 20 30 30

   * - **Version**
     - **Board label**
     - **Memory**
     - **Board name in software**
   * - ver1
     - ``V1.0``
     - 16 GB LPDDR4X (8 GB x 2)
     - ``rzv2h-rdk-ver1``
   * - ver101
     - ``V1.0.1``
     - 8 GB LPDDR4X (4 GB x 2)
     - ``rzv2h-rdk-ver101``

.. figure:: ../images/rdk_version_1.png
   :alt: RZ/V2H RDK ver1 board label (V1.0)
   :width: 400px
   :align: center

   RZ/V2H RDK ver1 (16 GB): label ``V1.0``

.. figure:: ../images/rdk_version_101.png
   :alt: RZ/V2H RDK ver101 board label (V1.0.1)
   :width: 400px
   :align: center

   RZ/V2H RDK ver101 (8 GB): label ``V1.0.1``

.. important::

   The IPL (BL2 and FIP) is board-specific. Always use the IPL that matches the board version. An IPL for the other version programs the wrong DDR configuration and the board does not boot.

Software Environment
^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 25 75

   * - **Category**
     - **Description**
   * - **OS Support**
     - **Ubuntu 24.04** (available in headless (Server) and Desktop).
   * - **Default Credentials**
     - Username: **ubuntu** | Password: **ubuntu**
   * - **ROS 2 Distribution**
     - Tested with **ROS 2 Jazzy**

Hardware Environment
^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - **Items**
     - **Description**

   * - **RZ/V2H**
     - **CPU:**

       * 4 x Arm Cortex-A55 (1.8 GHz)
       * 2 x Arm Cortex-R8 (800 MHz)
       * 1 x Arm Cortex-M33 (200 MHz)

       **DRP:**

       * Vision/Dynamically Reconfigurable Processor

       **DRP-AI3:**

       * Hardware AI Accelerator (8 dense TOPS, 80 sparse TOPS)

       **Package:**

       * R9A09G057H44GBG: 1368-pin FCBGA

   * - **Memory**
     - LPDDR4X 1600 MHz

       * ver1: 16 GB (8 GB x 2)
       * ver101: 8 GB (4 GB x 2)

       See `Board Versions`_.

   * - **SD Card**
     - Includes a 64 GB SanDisk microSD card in the box

   * - **QSPI Flash ROM**
     - 64 MB

   * - **Interfaces**
     - * DC Jack power input supported: 12–24 V / 2 A (12 V / 2 A power adapter included in the box)
       * JTAG (10-pin)
       * MIPI CSI-2 4-Lane x2 (22-pin / 0.5 mm)
       * Micro-HDMI
       * USB 3.2 Type-A x2
       * USB Micro-B (SCIF)
       * 10/100/1000 Base-T RJ45
       * microSD
       * PCIe 3.0 Root Complex (16-pin / 0.5 mm)
       * CAN-FD x2
       * 40-pin RasPi GPIO Header

For more details about RZ/V2H RDK's specification, visit the `WS125 Robotic Development Kit Hardware Manual <https://github.com/renesas-rdk/rzv2h_rdk_documentation/raw/refs/heads/main/docs/pdf/WS125-V2HRDKREFZ-Hardware-Manual.pdf?download=>`_.

**RZ/V2H RDK Image View:**

The following image shows the top/bottom view of the RZ/V2H Robotic Development Kit (RDK) board,
highlighting its main connectors and interfaces.

.. figure:: ../images/RDK_Top.png
   :alt: RZ/V2H RDK Top View
   :width: 500px
   :align: center

   RZ/V2H RDK Top View

.. figure:: ../images/RDK_Bottom.png
   :alt: RZ/V2H RDK Bottom View
   :width: 500px
   :align: center

   RZ/V2H RDK Bottom View

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

When setting up the development environment for the RZ/V2H RDK, it is important to have the necessary hardware components and software tools in place. Below is an overview of the required items and their descriptions.

RZ/V2H RDK
""""""""""

.. list-table::
   :widths: 20 80
   :header-rows: 0

   * - RZ/V2H RDK
     - RZ/V2H Robotic Development Kit (RDK).
   * - AC Adapter
     - Power Delivery adapter for the board power supply (included in box).
   * - HDMI Cable
     - Used to connect the HDMI monitor to the board.
       The RZ/V2H RDK has an HDMI port.
   * - USB Camera
     - Since the RZ/V2H RDK does not include a camera module, this will be the standard camera input source.

       Supported resolution: 640x480

       Supported format: 'YUYV' (YUYV 4:2:2)

Common
""""""

.. list-table::
   :widths: 20 80
   :header-rows: 0

   * - USB to microUSB Cable
     - Used to connect the board to the PC for initial setup and development.
   * - Ethernet Cable
     - Used to connect the board to the network for software installation and updates.
   * - HDMI Monitor
     - Used to display the graphical output of the board.
   * - microSD Card
     - Must have at least 16 GB of free space and must support high-speed mode.
   * - Ubuntu 24.04 PC with Docker
     - Used for microSD card setup and development environment setup.

       Operating environment: **Ubuntu 24.04**
   * - SD Card Reader
     - Used for setting up the microSD card.
   * - USB Hub
     - Used to connect a USB keyboard and USB mouse to the board.
   * - USB Keyboard
     - Used to type strings on the terminal of the board.
   * - USB Mouse
     - Used to operate the mouse on the screen of the board.