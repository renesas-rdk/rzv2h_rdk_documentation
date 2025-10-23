High-Speed Interface
--------------------------------

The RZ/V2H Robotic Development Kit is equipped with several high-speed interfaces that enable users to connect a variety of peripherals and expansion modules.

This sections describes the High-Speed Interface unit of this Kit.

1. PCIe 3.0 16-pin connector
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The PCIe 3.0 interface on the RZ/V2H RDK allows for high-speed data transfer and connectivity with compatible PCIe devices.

.. important::

   The PCIe interface is not available in this release.

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

For example, to use the OV5645 camera module, create and run the following script in the terminal:

.. code-block:: bash

    #!/bin/bash

    cru=$(cat /sys/class/video4linux/v4l-subdev*/name | grep "cru-ip")
    csi2=$(cat /sys/class/video4linux/v4l-subdev*/name | grep "csi2")
    ov=$(cat /sys/class/video4linux/v4l-subdev*/name | grep "ov")
    default_camera=$(echo "$ov" | awk '{print $1}')

    declare -A camera_default_resolution
    camera_default_resolution["ov5640"]="1280x720"
    camera_default_resolution["ov5645"]="1280x960"

    declare -A camera_valid_resolutions
    camera_valid_resolutions["ov5640"]="720x480 720x576 1024x768 1280x720 1920x1080 2592x1944"
    camera_valid_resolutions["ov5645"]="1280x960 1920x1080 2592x1944"

    # Usage information function
    function print_usage {
        echo "Usage: $0 <resolution>"
        echo "Detected camera: $default_camera"
        echo "Available resolutions for $default_camera: ${camera_valid_resolutions[$default_camera]}"
        echo ""
        echo "Example: $0 1920x1080"
        echo "If no resolution is specified, $default_camera will used the default resolution '${camera_default_resolution[$default_camera]}'."
    }

    # Check if help is requested
    if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
        print_usage
        exit 0
    fi

    # Check for no input
    if [ -z "$1" ]; then
        echo "Detected camera: $default_camera"
        echo "No resolution specified. Using default resolution: ${camera_default_resolution[$default_camera]}"
        ov564x_res="${camera_default_resolution[$default_camera]}"
    else
        ov564x_res="$1"
        valid_resolutions=(${camera_valid_resolutions[$default_camera]})
        # Check if the given resolution is valid
        if [[ ! " ${valid_resolutions[@]} " =~ " ${ov564x_res} " ]]; then
            echo "Invalid resolution: $ov564x_res for camera $default_camera"
            ov564x_res="${camera_default_resolution[$default_camera]}"
            echo "Input resolution is not available. Using default resolution: ${camera_default_resolution[$default_camera]}"
        fi
    fi

    if [ -z "$cru" ]
    then
        echo "No CRU video device founds"
    else
        media-ctl -d /dev/media0 -r
        if [ -z "$csi2" ]
        then
            echo "No MIPI CSI2 sub video device founds"
        else
            media-ctl -d /dev/media0 -V "'$csi2':0 [fmt:UYVY8_1X16/$ov564x_res field:none]"
            media-ctl -d /dev/media0 -V "'$csi2':1 [fmt:UYVY8_1X16/$ov564x_res field:none]"
            media-ctl -d /dev/media0 -V "'$ov':0 [fmt:UYVY8_1X16/$ov564x_res field:none]"
            media-ctl -d /dev/media0 -V "'$cru':0 [fmt:UYVY8_1X16/$ov564x_res field:none]"
            media-ctl -d /dev/media0 -V "'$cru':1 [fmt:UYVY8_1X16/$ov564x_res field:none]"

            width=${ov564x_res%x*}
            height=${ov564x_res#*x}
            v4l2-ctl -d /dev/video0 --set-fmt-video=width=${width},height=${height},pixelformat=UYVY
            echo "Link CRU/CSI2 to $ov with format UYVY8_1X16 and resolution ${ov564x_res}"
        fi
    fi

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

TODO: Update the command about capturing the image from the camera module.

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
