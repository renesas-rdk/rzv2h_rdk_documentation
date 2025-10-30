Frequency Asked Questions
================================

1. Does the RZ/V2H RDK support multiple operating systems?

Yes, the RZ/V2H RDK supports multiple operating systems running concurrently on its different CPU cores.

For more details, refer to the :ref:`RZ/V Multi-OS <multi_os>` section.

2. How can I set up the USB-UART interface on the RZ/V2H RDK?

Refer to the :ref:`Other interfaces - USB-UART <usb_uart>` section for detailed instructions on setting up the USB-UART interface for serial communication and debugging.

3. How do I configure a static IP address for high-speed interfaces on the RZ/V2H RDK?

Refer to the :ref:`Set Static IP <high_speed_interfaces_set_static_ip>` section for step-by-step instructions on configuring a static IP address.

4. I build the Ruiyan related ROS2 packages, but encounter the following error:

.. code-block:: text

   --- stderr: rh6_ctrl
    CMake Error at CMakeLists.txt:47 (message):
    library Path: /home/ubuntu/ros2_ws/src/ruiyan_rh2_controller/rh6_ctrl/lib
    not found!


    ---
    Failed   <<< rh6_ctrl [0.97s, exited with code 1]

Please make sure to copy the ``libRyhandArm64.so`` file to the SDK sysroot before cross-compilation as mentioned in the :ref:`Dexterous Hand Control Application <dexhand>` and :ref:`Rock Paper Scissors Application <rock_paper_scissors>` sections.

5. Does the RZ/V2H RDK support Docker?

Yes, the RZ/V2H RDK supports Docker. You can install Docker on the RZ/V2H RDK by following steps:

.. code-block:: bash

    $ sudo apt update

    # Make sure you can access the internet
    $ ping bing.com

    # Install docker
    $ curl -fsSL https://get.docker.com | sudo sh

    # Add user to docker group
    $ sudo usermod -aG docker $USER

    # Logout and Login back to make it effects

    # Simple check
    $ docker --version
    $ docker run hello-world

6. How to avoid ABI mismatch error when cross-compiling ROS2 applications?

Please refer to the :ref:`How to avoid ABI mismatch error? <abi_mismatch>` section for detailed instructions.

7. How to update/add the Yocto SDK with the correct ROS2 package versions with eSDK?

Please refer to the :ref:`How to update/add the Yocto SDK with the correct ROS2 package versions with eSDK? <update_ros2_packages_sdk>` section for detailed instructions.

8. What USB-WIFI adapters are compatible with the RZ/V2H RDK?

The following USB-WIFI adapters have been tested and are compatible with the RZ/V2H RDK:

- Ralink Technology, Corp. MT7601U Wireless Adapter
- AC1300 Tp-Link T3U Nano

Please install the ``linux-firmware`` package on the RZ/V2H RDK to ensure proper driver support for these adapters.

If you have other USB-WIFI adapters, please refer to the manufacturer's documentation for compatibility and driver installation instructions.

You can take the following steps as general guidance to check and install the required drivers:

- Enable the corresponding driver in the Linux kernel configuration (refer to :ref:`Custom Linux Kernel and Device Tree <build_kernel>`).
- Build and install the driver module on the RZ/V2H RDK.
- Find the correct firmware files and place them in the ``/lib/firmware/`` directory on the RZ/V2H RDK.