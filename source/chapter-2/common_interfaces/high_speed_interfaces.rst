High-Speed Interface
^^^^^^^^^^^^^^^^^^^^

The RZ/V2H Robotic Development Kit is equipped with several high-speed interfaces that enable users to connect a variety of peripherals and expansion modules.

This section describes the high-speed interface unit of this kit.

PCIe 3.0 16-pin connector
"""""""""""""""""""""""""

The PCIe 3.0 interface on the RZ/V2H RDK allows for high-speed data transfer and connectivity with compatible PCIe devices.

For example, you can connect a PCIe NVMe SSD to enhance storage performance. The following steps describe how to set up and use a PCIe NVMe SSD with the RZ/V2H RDK.

- Hardware requirements (recommended):

  - `PCIe TO M.2 Board <https://category.yahboom.net/products/yb-pcle-m-2?variant=49960215609660>`_.
  - M.2 NVMe SSD.

- Hardware setup:

  1. Power off the RZ/V2H RDK.
  2. Connect the PCIe TO M.2 Board to the PCIe 3.0 16-pin connector on the RZ/V2H RDK.
  3. Insert the M.2 NVMe SSD into the PCIe TO M.2 Board.
  4. Power on the RZ/V2H RDK.

.. important::

   - Handle the M.2 NVMe SSD with care to avoid damage from static electricity.
   - Make sure that you connect the PCIe TO M.2 Board to the correct PCIe 3.0 16-pin connector on the RZ/V2H RDK.

Usage example with pciutils:

First, install the ``pciutils`` package if it is not already installed:

.. code-block:: bash

   sudo apt install pciutils

To list all PCIe devices connected to the system, use the following command:

.. code-block:: bash

   lspci

Example output:

.. code-block:: console
   :emphasize-lines: 2

   00:00.0 PCI bridge: Renesas Technology Corp. Device 003b
   01:00.0 Non-Volatile memory controller: Samsung Electronics Co Ltd NVMe SSD Controller 980 (DRAM-less)

To check whether the NVMe SSD is recognized by the system, use the following command:

.. code-block:: bash

   lsblk

Example output:

.. code-block:: console
   :emphasize-lines: 9

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

   sudo mkdir /mnt/nvme
   sudo mount /dev/nvme0n1 /mnt/nvme

Unmount the NVMe SSD:

.. code-block:: bash

   sudo umount /mnt/nvme
   sudo rmdir /mnt/nvme

MIPI-CSI 22-pin connector x2
""""""""""""""""""""""""""""

The RZ/V2H RDK features dual MIPI-CSI connectors that support camera input for applications requiring image capture and processing.

.. seealso::

   For information about the available partner camera module list for RZ/V2H, see `[RZ/V2H] AVAILABLE PARTNER CAMERA MODULE LIST <https://www.renesas.com/en/document/apn/rzv-available-partner-camera-module-list?r=25471761>`_.

The default RZ/V2H RDK device tree supports the OV5645 camera module connected to the MIPI-CSI interface.

.. hint::

   To use the MIPI-CSI interface with another camera module, you must change the DTS file. Refer to the :ref:`Modify the DTS file <modify_dts>` section in the Build Kernel chapter for more details about customizing the DTS file.

Set up the MIPI-CSI interface
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Before using the MIPI-CSI interface, configure the camera properties first.

For example, to use the OV5645 camera module, create and run the `v4l2_init.sh <https://partnergitlab.renesas.solutions/sst1/industrial/ws078/utility/common_utils/-/blob/main/linux_utils/mipi_camera/v4l2_init.sh?ref_type=heads>`_ script in the terminal:

.. code-block:: bash

   # Install necessary packages
   sudo apt install v4l-utils

   # Download the v4l2_init.sh script
   wget https://github.com/Renesas-SST/meta-renesas/raw/refs/heads/styhead/rz-cmn/recipes-extend/v4l2-init/files/v4l2-init.sh

   # Make the script executable
   chmod +x v4l2_init.sh

   # Run the script to initialize the camera
   ./v4l2_init.sh

This script detects the connected camera module and sets the desired resolution.

For other camera modules, modify the script accordingly.

Usage example with v4l2-ctl
~~~~~~~~~~~~~~~~~~~~~~~~~~~

List all connected cameras:

.. code-block:: bash

   v4l2-ctl --list-devices

List all supported formats for the selected camera ``/dev/video0``:

.. code-block:: bash

   v4l2-ctl -d /dev/video0 --list-formats-ext

Refer to the :ref:`Video Codec section <video_codec>` for an example of video capture with a MIPI camera.

.. _high_speed_interfaces_set_static_ip:

1000M RJ45 - Gigabit Ethernet Port
""""""""""""""""""""""""""""""""""

The Gigabit Ethernet (1000M RJ45) port on the RZ/V2H RDK provides high-speed network connectivity for data communication and internet access.

Connect the network cable to the Gigabit Ethernet port before using the Ethernet interface.

The current Ubuntu netplan configures the system to obtain its network settings via DHCP.

After connecting the Ethernet cable, use the following command to confirm the network configuration.

To list all network interfaces and their IP addresses:

.. code-block:: bash

   ip a

To test network connectivity to an external server, use the ``ping`` command:

.. code-block:: bash

   ping -c 4 bing.com
   ping -c 4 8.8.8.8

Set a static IP address
~~~~~~~~~~~~~~~~~~~~~~~

In Ubuntu, the network is configured with Netplan. If you need to set a static IP address for the Ethernet interface, for example ``192.168.0.100``, follow these steps:

- Open the network configuration file with ``vim``:

  .. code-block:: bash

     sudo vi /etc/netplan/50-cloud-init.yaml

- Modify the file to set a static IP address. For example:

  .. code-block:: yaml

     # This file describes the network interfaces available on your system
     # For more information, see netplan(5).
     network:
       version: 2
       renderer: networkd
       ethernets:
         end0:
           dhcp4: no
           addresses: [169.254.43.99/24]
           routes:
             - to: default
               via: 169.254.43.86
           nameservers:
             addresses: [8.8.8.8, 8.8.4.4]

  .. note::

     Make sure to replace the ``addresses`` and ``routes`` values with the appropriate values for your network.

     If you are using a different network configuration file, such as ``/etc/netplan/50-cloud-init.yaml``, modify that file instead.

- Apply the changes with the following command:

  .. code-block:: bash

     sudo netplan apply

Set DHCP
~~~~~~~~

If you want to revert to DHCP configuration, modify the ``/etc/netplan/50-cloud-init.yaml`` file as follows:

.. code-block:: yaml

   # This file describes the network interfaces available on your system
   # For more information, see netplan(5).
   network:
     version: 2
     renderer: networkd
     ethernets:
       end0:
         dhcp4: yes

Apply the changes with the following command:

.. code-block:: bash

   sudo netplan apply

USB 3.0 Type A x2
"""""""""""""""""

The RZ/V2H RDK includes two USB 3.0 Type-A ports that support high-speed data transfer for connecting various USB peripherals, such as external storage devices, cameras, and input devices.

To use these devices, simply connect them to the USB 3.0 Type-A ports.

Verify USB 3.0 functionality
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To verify that the USB 3.0 ports are functioning correctly, you can use the following command to list USB devices and check their connection speed:

.. code-block:: bash

   lsusb -t

Example output:

.. code-block:: console

   /:  Bus 001.Port 001: Dev 001, Class=root_hub, Driver=xhci-renesas-hcd/1p, 480M
   /:  Bus 002.Port 001: Dev 001, Class=root_hub, Driver=xhci-renesas-hcd/1p, 20000M/x2
   /:  Bus 003.Port 001: Dev 001, Class=root_hub, Driver=xhci-renesas-hcd/1p, 480M
   /:  Bus 004.Port 001: Dev 001, Class=root_hub, Driver=xhci-renesas-hcd/1p, 20000M/x2

USB-WIFI Adapter Support
~~~~~~~~~~~~~~~~~~~~~~~~

The following USB-WIFI adapters have been tested and are compatible with the RZ/V2H RDK:

- Ralink Technology, Corp. MT7601U Wireless Adapter
- AC1300 TP-Link T3U Nano

.. note::

   If you want to use a different USB WiFi adapter, make sure the required driver is available for the RZ/V2H RDK.

   You need to identify the appropriate driver for the USB WiFi adapter and enable it in the Linux kernel configuration file. For example, add ``CONFIG_MT7601U=y`` to ``linux-rz/arch/arm64/configs/renesas_defconfig``, then rebuild and deploy the kernel image.

   Refer to the :ref:`Custom Linux Kernel and Device Tree <build_kernel>` section for instructions on how to add support for additional drivers by modifying the Linux kernel.

Usage example
~~~~~~~~~~~~~

- Install necessary packages

  .. code-block:: bash

     sudo apt update
     sudo apt install rfkill iw wpasupplicant

- Check USB devices

  First, connect the USB-WIFI adapter to the RZ/V2H RDK.

  Then, run the following command to list all connected USB devices:

  .. code-block:: bash

     lsusb

  Example output:

  .. code-block:: bash
     :emphasize-lines: 4,6

     Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
     Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
     Bus 003 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
     Bus 003 Device 003: ID 2357:0138 TP-Link 802.11ac NIC
     Bus 004 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
     Bus 005 Device 003: ID 148f:7601 Ralink Technology, Corp. MT7601U Wireless Adapter

- Check interface name

  .. code-block:: bash

     ip a | grep wl

  Example output:

  .. code-block:: bash

     7: wlx98ba5f1918cf: <BROADCAST,MULTICAST,DYNAMIC> mtu 1500 qdisc noqueue state DOWN group default qlen 1000

- Unlock the WiFi interface (if necessary)

  .. code-block:: bash

     sudo rfkill list wifi                # Check if the WiFi interface is blocked
     sudo rfkill unblock wifi             # If it is blocked, unblock the WiFi interface
     sudo rfkill list wifi                # Verify that the WiFi interface is now unblocked

  Example output:

  .. code-block:: bash

     0: phy0: Wireless LAN
        Soft blocked: no
        Hard blocked: no

- Bring up the WiFi interface

  .. code-block:: bash

     sudo ip link set wlx98ba5f1918cf up  # Bring up the WiFi interface
     ip a | grep wl                       # Check the interface status again

  Example output:

  .. code-block:: bash

     7: wlx98ba5f1918cf: <NO-CARRIER,BROADCAST,MULTICAST,UP> mtu 1500 qdisc noqueue state DOWN group default qlen 1000

- Scan for available WiFi networks

  .. code-block:: bash

     sudo iw dev wlx98ba5f1918cf scan | grep <YOUR_SSID>

- Modify network configuration to connect to the WiFi network by editing the Netplan configuration file:

  .. code-block:: bash

     sudo vi /etc/netplan/50-cloud-init.yaml

  Add the following configuration to connect to the WiFi network (replace ``MY_SSID`` and ``MY_PASSWORD`` with your actual WiFi SSID and password):

  .. code-block:: yaml

     network:
       version: 2
       renderer: networkd
       ethernets:
         end0:
           dhcp4: true
       wifis:
         wlx98ba5f1918cf:
           dhcp4: true
           access-points:
             "MY_SSID":
               password: "MY_PASSWORD"

- Apply the changes with the following command:

  .. code-block:: bash

     sudo netplan apply

- Test network connectivity

  .. code-block:: bash

     ping -I wlx98ba5f1918cf bing.com

  Example output:

  .. code-block:: bash

     PING bing.com (150.171.27.10) from 192.168.19.177 wlx98ba5f1918cf: 56(84) bytes of data.
     64 bytes from 150.171.27.10: icmp_seq=1 ttl=120 time=420 ms
     64 bytes from 150.171.27.10: icmp_seq=2 ttl=120 time=482 ms
     --- bing.com ping statistics ---
     2 packets transmitted, 2 received, 0% packet loss, time 1001ms
     rtt min/avg/max/mdev = 419.837/451.166/482.495/31.329 ms