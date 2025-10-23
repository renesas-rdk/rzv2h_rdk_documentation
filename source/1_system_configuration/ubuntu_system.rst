Ubuntu System with RZ/V2H RDK
========================================

This section provides usage information about the interfaces available on the RZ/V2H RDK when running the Ubuntu system.

For more details about specification of each interface, refer to the `RZ/V2H Group User's Manual: Hardware <https://www.renesas.com/en/document/mah/rzv2h-group-users-manual-hardware>`_.

Overview
--------------------------------
The RZ/V2H RDK supports multiple peripheral interfaces that allow users to connect and control external devices for various robotic and industrial applications.
These interfaces include:

.. figure:: images/RDK_hw_interface.png
   :alt: RZ/V2H Robotic Development Kit Front View
   :align: center
   :width: 500px

   RZ/V2H Robotic Development Kit Interfaces

Main Interfaces
--------------------------------

The main interfaces available on the RZ/V2H Robotic Development Kit are listed below.

.. list-table:: High-Speed Interfaces
   :header-rows: 1
   :widths: 5 25 30

   * - No
     - Interface Name
     - Description
   * - 1
     - PCIe 3.0 16-pin connector
     - High-speed peripheral expansion interface.
   * - 2
     - MIPI-CSI 22-pin connector ×2
     - Dual camera input interface for image sensors.
   * - 3
     - 1000M RJ45
     - Gigabit Ethernet LAN port for network connectivity.
   * - 4
     - USB 3.0 Type A ×2
     - USB host ports for external devices such as mouse, keyboard, or USB camera.

.. list-table:: Communication Interfaces
   :header-rows: 1
   :widths: 5 25 30

   * - No
     - Interface Name
     - Description
   * - 5
     - CAN-FD ×2
     - Controller Area Network Flexible Data-Rate communication ports.
   * - 6
     - RasPi GPIO 40-pin Header (I2C, SPI, UART, GPIO, PCM)
     - General purpose I/O interface compatible with Raspberry Pi pin layout.

.. list-table:: Other Interfaces
   :header-rows: 1
   :widths: 5 25 30

   * - No
     - Interface Name
     - Description
   * - 7
     - Mini-HDMI
     - Video output interface for external display.
   * - 8
     - USB-UART
     - Serial console interface for debugging.
   * - 9
     - JTAG 10-pin
     - JTAG interface for debugging and programming.

Each subsection provides details on how to identify, configure, and access these interfaces within the Ubuntu environment.

.. toctree::
    :hidden:
    :maxdepth: 2

    high_speed_interfaces
    communication_interface
    other_interfaces