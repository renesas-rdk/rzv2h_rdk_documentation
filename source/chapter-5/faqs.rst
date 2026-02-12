Frequency Asked Questions
---------------------------

**1. Does the RZ/V2H RDK support multiple operating systems?**

Yes, the RZ/V2H RDK supports multiple operating systems running concurrently on its different CPU cores.
For more details, refer to the :ref:`RZ/V Multi-OS <multi_os>` section.

**2. How can I set up the USB-UART interface on the RZ/V2H RDK?**

Refer to the :ref:`Other interfaces - USB-UART <usb_uart>` section for detailed instructions on setting up the USB-UART interface for serial communication and debugging.

**3. How do I configure a static IP address for high-speed interfaces on the RZ/V2H RDK?**

Refer to the :ref:`Set Static IP <high_speed_interfaces_set_static_ip>` section for step-by-step instructions on configuring a static IP address.

**4. I build the Ruiyan related ROS2 packages, but encounter the following error:**

.. code-block:: text

   --- stderr: rh6_ctrl
   CMake Error at CMakeLists.txt:47 (message):
   library Path: /home/ubuntu/ros2_ws/src/ruiyan_rh2_controller/rh6_ctrl/lib
   not found!

   ---
   Failed   <<< rh6_ctrl [0.97s, exited with code 1]

Please make sure to copy the ``libRyhandArm64.so`` file to the SDK sysroot before cross-compilation as mentioned in the :ref:`Dexterous Hand Control Application <dexhand>` and :ref:`Rock Paper Scissors Application <rock_paper_scissors>` sections.

**5. Does the RZ/V2H RDK support Docker?**

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

**6. How to avoid ABI mismatch error when cross-compiling ROS2 applications?**

Please refer to the :ref:`How to avoid ABI mismatch error? <abi_mismatch>` section for detailed instructions.

**7. How to update/add the Yocto SDK with the correct ROS2 package versions with eSDK?**

Please refer to the :ref:`How to update/add the Yocto SDK with the correct ROS2 package versions with eSDK? <update_ros2_packages_sdk>` section for detailed instructions.

.. _faq_usb_wifi:

**8. What USB-WIFI adapters are compatible with the RZ/V2H RDK?**

The following USB-WIFI adapters have been tested and are compatible with the RZ/V2H RDK:

- Ralink Technology, Corp. MT7601U Wireless Adapter
  Linux firmware download link: `mt7601u.bin <https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/mediatek/mt7601u.bin>`_
  Firmware path: ``/lib/firmware/mt7601u.bin``

- AC1300 TP-Link T3U Nano
  Linux firmware download link: `rtw8822b_fw.bin <https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/rtw88/rtw8822b_fw.bin>`_
  Firmware path: ``/lib/firmware/rtw88/rtw8822b_fw.bin``

.. important::
   By default, no firmware is installed. Please install the correct **Linux firmware file** into the appropriate **firmware path** on the **RZ/V2H RDK root file system**, and then reboot the device to ensure proper driver support for these adapters.

If you have other USB-WIFI adapters, please refer to the manufacturer's documentation for compatibility and driver installation instructions.
You can take the following steps as general guidance to check and install the required drivers and firmware for other USB-WIFI adapters:

- Enable the corresponding driver in the `Linux kernel configuration <https://www.linuxjournal.com/article/6568>`_ (refer to :ref:`Custom Linux Kernel and Device Tree <build_kernel>`).
  Recommend enable the driver as a module (m type) and copy the built module to the RZ/V2H RDK.
  For example, for the two adapters listed above, the following kernel configurations have already been enabled:

  .. code-block:: console

     CONFIG_MT7601U
     CONFIG_MT76x0U
     CONFIG_RTL8XXXU
     CONFIG_RTW88
     CONFIG_RTW88_8822BU

- Build and install the driver module on the RZ/V2H RDK.

- Find the correct **firmware files** at: `Linux fimrware repository <https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/>`_ and place them in the appropriate **directory path** within the **RZ/V2H RDK root file system** to ensure proper device operation. Typically, firmware files are located in the ``/lib/firmware/`` directory.

- Reboot the RZ/V2H RDK and check if the USB-WIFI adapter is recognized and functioning properly.

.. tip::
   All firmware file can be found in the `Linux firmware repository <https://git.kernel.org/pub/scm/linux/kernel/git/firmware/linux-firmware.git/plain/>`_.
   Please searching the repository with the USB-WIFI adapter model name or chip-set can help to locate the correct firmware file.

**How to know which driver and firmware is needed for your USB-WIFI adapter?**

- Connect the USB-WIFI adapter to the RZ/V2H RDK.
- Open a terminal and run the command: ``lsusb`` to list all connected USB devices.
- Identify your USB-WIFI adapter from the list and note its Vendor ID and Product ID.
- Use the Vendor ID and Product ID to search online for the specific driver and firmware required for your USB-WIFI adapter model.
- Refer to the manufacturer's website or Linux community forums for additional information on driver and firmware installation.
- Using ``dmesg`` command after plugging in the USB-WIFI adapter can also provide useful information about the required driver and firmware.
  For example, run the command: ``dmesg | grep -i firmware`` to check for any firmware-related messages.

  .. code-block:: console

     rz@localhost:~$ dmesg | grep firmware
     [   17.946635] rtw_8822bu 3-1:1.0: Direct firmware load for rtw88/rtw8822b_fw.bin failed with error -2

  This log indicates that the ``rtw8822b_fw.bin`` firmware file is required at ``/lib/firmware/rtw88/rtw8822b_fw.bin`` for the adapter to function properly.

- Once you have identified the required driver and firmware, follow the steps mentioned above to install them on the RZ/V2H RDK.

**9. Failed opening device /dev/video0 or /dev/tty: Permission denied**

Please complete the First Time Boot Setup as described in the :ref:`First Time Boot Setup <first_time_boot_setup>` section to add your user to the necessary groups (e.g., `video`, `dialout`) to gain access to these device files.

**10. No space left on device error**

Please complete the First Time Boot Setup as described in the :ref:`First Time Boot Setup <first_time_boot_setup>` section to expand the root filesystem to utilize the full storage capacity of the RZ/V2H RDK.