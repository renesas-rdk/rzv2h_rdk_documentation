Overview
=======================================================

WS125 Robotic Development Kit is a solution with Renesas new generation `RZ/V2H MPU <https://www.renesas.com/en/products/rz-v2h?tab=overview>`_ for AI application,
which has AI inference processing performance of up to 80TOPS with multi-core CPU to run multiple OS
simultaneously for high performance AI image processing.

It is also equipped with many interfaces that make it suitable for development and integration into a variety of
robotic applications.

Software Environment
--------------------

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
--------------------

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - **Items**
     - **Description**

   * - **RZ/V2H**
     - **CPU:**

       * 4 x Arm Cortex-A55 (1.8GHz)
       * 2 x Arm Cortex-R8 (800MHz)
       * 1 x Arm Cortex-M33 (200MHz)

       **DRP:**

       * Vision/Dynamically Reconfigurable Processor

       **DRP-AI3:**

       * Hardware AI Accelerator (8 dense TOPS, 80 sparse TOPS)

       **Package:**

       * R9A09G057H44GBG: 1368-pin FCBGA

   * - **Memory**
     - LPDDR4 1600MHz - 16GB (8GB x 2)

   * - **SD Card**
     - Includes a 64GB SanDisk microSD card in the box

   * - **QSPI Flash ROM**
     - 64MB

   * - **Interfaces**
     - * DC Jack power input supported: 12-24V / 2A (12V/2A power adapter included in the box)
       * JTAG (10-pin)
       * MIPI CSI-2 4-Lane x2 (22-pin / 0.5mm)
       * Micro-HDMI
       * USB3.2 Type-A x2
       * USB Micro-B (SCIF)
       * 10/100/1000 Base-T RJ45
       * Micro SD
       * PCIe 3.0 Root Complex (16-pin / 0.5mm)
       * CAN-FD x2
       * 40-pin RasPi GPIO Header

For more details about RZ/V2H RDK's specification, visit the `WS125 Robotic Development Kit Hardware Manual <https://github.com/renesas-rdk/rzv2h_rdk_documentation/raw/refs/heads/main/docs/pdf/WS125_Robotic_Development_Kit_HardwareManual.pdf?download=>`_.

**RZ/V2H RDK Board Image View:**

The following image shows the top/bottom view of the RZ/V2H Robotics Development Kit (RDK) board,
highlighting its main connectors and interfaces.

.. figure:: ../images/RDK_Top.png
   :alt: RZ/V2H RDK Board Top View
   :width: 500px
   :align: center

   RZ/V2H RDK Board Top View

.. figure:: ../images/RDK_Bottom.png
   :alt: RZ/V2H RDK Board Bottom View
   :width: 500px
   :align: center

   RZ/V2H RDK Board Bottom View