High-Speed Interface
--------------------------------

The RZ/V2H Robotic Development Kit is equipped with several high-speed interfaces that enable users to connect a variety of peripherals and expansion modules.

This sections describes the High-Speed Interface unit of this Kit.

1. PCIe 3.0 16-pin connector
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The PCIe 3.0 interface on the RZ/V2H RDK allows for high-speed data transfer and connectivity with compatible PCIe devices.

For example, you can connect a PCIe NVMe SSD to enhance storage performance. Following are the steps to set up and use a PCIe NVMe SSD with the RZ/V2H RDK.

- Hardware Requirements (recommended):

  - `PCIe TO M.2 Board (D) <https://www.waveshare.com/wiki/PCIe_TO_M.2_Board_(D)>`_.
  - M.2 NVMe SSD.

- Hardware Setup:

  1. Power off the RZ/V2H RDK.
  2. Connect the PCIe TO M.2 Board to the PCIe 3.0 16-pin connector on the RZ/V2H RDK.
  3. Insert the M.2 NVMe SSD into the PCIe TO M.2 Board.
  4. Connect 5V power and GND (from GPIO 40 Pins) to the PCIe TO M.2 Board.
  5. Power on the RZ/V2H RDK.

.. important::

   - Ensure that the PCIe TO M.2 Board is properly powered, as the RZ/V2H RDK does not supply power to PCIe devices.

   - Make sure to handle the M.2 NVMe SSD with care to avoid damage from static electricity.

   - Make sure you connect the PCIe TO M.2 Board to the correct PCIe 3.0 16-pin connector on the RZ/V2H RDK.

Usage example with pciutils:

First, install the pciutils package if it is not already installed:

.. code-block:: bash

    $ sudo apt install pciutils

To list all PCIe devices connected to the system, use the following command:

.. code-block:: bash

    $ lspci

Example output:

.. code-block:: console

    root@localhost:~# lspci
    00:00.0 PCI bridge: Renesas Technology Corp. Device 003b
    01:00.0 Non-Volatile memory controller: Samsung Electronics Co Ltd NVMe SSD Controller 980 (DRAM-less)
    root@localhost:~#

To check the NVMe SSD is recognized by the system, use the following command:

.. code-block:: bash

    $ lsblk

Example output:

.. code-block:: console
    :emphasize-lines: 10

    root@localhost:~# lsblk
    NAME        MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
    mtdblock0    31:0    0 116.5K  1 disk
    mtdblock1    31:1    0   1.8M  1 disk
    mtdblock2    31:2    0   128K  1 disk
    mtdblock3    31:3    0    14M  0 disk
    mmcblk0     179:0    0  29.7G  0 disk
    |-mmcblk0p1 179:1    0   100M  0 part
    `-mmcblk0p2 179:2    0   2.4G  0 part /
    nvme0n1     259:0    0 465.8G  0 disk

Mount the NVMe SSD:

.. code-block:: bash

    $ sudo mkdir /mnt/nvme
    $ sudo mount /dev/nvme0n1 /mnt/nvme

Unmount the NVMe SSD:

.. code-block:: bash

    $ sudo umount /mnt/nvme
    $ sudo rmdir /mnt/nvme

2. MIPI-CSI 22-pin connector ×2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The RZ/V2H RDK features dual MIPI-CSI connectors that support camera input for applications requiring image capture and processing.

.. seealso::

   For information about available partner Camera module list on RZ/V2H, see `[RZ/V2H] AVAILABLE PARTNER CAMERA MODULE LIST <https://www.renesas.com/en/document/apn/rzv-available-partner-camera-module-list?r=25471761>`_.

The default RZ/V2H RDK device tree supports the OV5645 camera module connected to the MIPI-CSI interface.

.. hint::

    It is required to change the dts file to use the MIPI-CSI interface with other camera module. Refer to: :ref:`Modify the DTS file <modify_dts>` section in the Build Kernel chapter for more details about customize the dts file.

Set up the MIPI-CSI interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Before using the MIPI-CSI interface, we have to configure the property of the camera first.

For example, to use the OV5645 camera module, create and run the `v4l2_init.sh <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/utility/common_utils/-/blob/main/linux_utils/mipi_camera/v4l2_init.sh?ref_type=heads>`_ script in the terminal:

This script detects the connected camera module and sets the desired resolution.

For other camera modules, please modify the script accordingly.

Usage example with v4l2-ctl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

First, install the v4l-utils package if it is not already installed:

.. code-block:: bash

    $ sudo apt install v4l-utils

List all connected cameras:

.. code-block:: bash

    $ v4l2-ctl --list-devices

List all supported formats for selected camera /dev/video0:

.. code-block:: bash

    $ v4l2-ctl -d /dev/video0 --list-formats-ext

To capture an image from the camera using **Renesas Core Image Weston**, use the following command:

.. code-block:: bash

    $ gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! waylandsink

3. 1000M RJ45 - Gigabit Ethernet Port
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Gigabit Ethernet (1000M RJ45) port on the RZ/V2H RDK provides high-speed network connectivity for data communication and internet access.

Connect the network cable to (3) before using the Ethernet interface.

The Ubuntu installer configures the system to obtain its network settings via DHCP.

After connecting the Ethernet cable, use the following command to confirm the network configuration:

To list all network interfaces and their IP addresses:

.. code-block:: bash

    $ ifconfig

To test network connectivity reach an external server, use the ping command:

.. code-block:: bash

    $ ping -c 4 bing.com
    $ ping -c 4 8.8.8.8

Set a static IP address
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In Ubuntu, the network is configured with Netplan, if you need to set a static IP address for the Ethernet interface (example: use a static IP address 192.168.0.100), follow these steps:

- Open the network configuration file with vim:

  .. code-block:: bash

      $ sudo vi /etc/netplan/01-netcfg.yaml

- Modify the file to set a static IP address. For example:

  .. code-block:: yaml

    # This file describes the network interfaces available on your system
    # For more information, see netplan(5).
    network:
    version: 2
    renderer: NetworkManager
    ethernets:
        end0:
        dhcp4: no
        dhcp6: no
        addresses: [169.254.43.99/24]
        routes:
          - to: default
            via: 169.254.43.86
        nameservers:
            addresses: [8.8.8.8, 8.8.4.4]

- Apply the changes with the following command:

  .. code-block:: bash

      $ sudo netplan apply


Set DHCP
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In case you want to revert back to DHCP configuration, modify the ``/etc/netplan/01-netcfg.yaml`` file as follows:

  .. code-block:: yaml

      # This file describes the network interfaces available on your system
      # For more information, see netplan(5).
      network:
        version: 2
        renderer: NetworkManager
        ethernets:
          end0:
            dhcp4: yes
            dhcp6: yes

- Apply the changes with the following command:

  .. code-block:: bash

      $ sudo netplan apply

4. USB 3.0 Type A ×2
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The RZ/V2H RDK includes two USB 3.0 Type-A ports that support high-speed data transfer for connecting various USB peripherals, such as external storage devices, cameras, and input devices.

To use these devices, simply connect them to the USB 3.0 Type-A ports.

Verify USB 3.0 functionality
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To verify that the USB 3.0 ports are functioning correctly, you can use the following command to list USB devices and check their connection speed:

.. code-block:: bash

    $ lsusb -t

Example output:

.. code-block:: console

    root@localhost:~# lsusb -t
    /:  Bus 001.Port 001: Dev 001, Class=root_hub, Driver=xhci-renesas-hcd/1p, 480M
    /:  Bus 002.Port 001: Dev 001, Class=root_hub, Driver=xhci-renesas-hcd/1p, 20000M/x2
    /:  Bus 003.Port 001: Dev 001, Class=root_hub, Driver=xhci-renesas-hcd/1p, 480M
    /:  Bus 004.Port 001: Dev 001, Class=root_hub, Driver=xhci-renesas-hcd/1p, 20000M/x2
    root@localhost:~#
