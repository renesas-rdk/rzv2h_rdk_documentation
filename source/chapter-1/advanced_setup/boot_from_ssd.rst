Booting RZ/V2H RDK from SSD
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The advantages of booting from an SSD include **faster read/write speeds**, improved performance, and increased storage capacity compared to booting from an SD card.

Hardware Required
"""""""""""""""""

- RZ/V2H RDK set.
- SD card (for initial bootloader storage). Please flash the SD card with the RDK image.
- `PCIe TO M.2 Board <https://category.yahboom.net/products/yb-pcle-m-2?variant=49960215609660>`_.
- M.2 NVMe SSD.

Hardware Connection
"""""""""""""""""""

The following image shows how to connect the SSD to the RZ/V2H RDK using the PCIe TO M.2 Board:

.. figure:: ../../images/ssd_connection.png
   :alt: SSD Connection Diagram
   :align: center
   :width: 600px

   SSD Connection Diagram

Detail Steps
""""""""""""

.. important::

   - Make sure to back up any important data on the SSD before proceeding, as the following steps will erase all existing data on the SSD.
   - Connect the SSD to the PCIe TO M.2 Board before powering on the RZ/V2H board.
   - Make sure that you connect the PCIe TO M.2 Board to the correct PCIe 3.0 16-pin connector on the RZ/V2H RDK.
   - Handle the M.2 NVMe SSD with care to avoid damage from static electricity.

.. note::

   The following steps assume that the SSD is detected as ``/dev/nvme0n1``.
   If your system detects the SSD with a different device name, replace ``/dev/nvme0n1`` accordingly in the commands and examples.

#. Prepare the SSD:

   - Insert the M.2 NVMe SSD into the PCIe TO M.2 Board.
   - Connect the PCIe TO M.2 Board to the RZ/V2H RDK.

#. Boot from the SD card:

   - Insert the SD card with the Ubuntu image into the RZ/V2H RDK and power it on.
   - Ensure that the system boots successfully from the SD card.

#. Install the required tools:

   .. code-block:: bash

      sudo apt update
      sudo apt-get install bmap-tools

#. Partition and format the SSD:

   - Once booted from the SD card, open a terminal.
   - Make sure the SSD is recognized by running:

     .. code-block:: bash

        lsblk

   - Identify the SSD device, for example ``/dev/nvme0n1``.
   - Copy the ``ubuntu-24.04-server-arm64-rzv2h-rdk.img.xz`` file and the ``ubuntu-24.04-server-arm64-rzv2h-rdk.img.bmap`` file to the target board.

     .. code-block:: bash

        # Copy the image file to the target board
        scp ubuntu-24.04-server-arm64-rzv2h-rdk.img.xz ubuntu@<rzv2h_rdk_ip>:/home/ubuntu/

        # Copy the bmap file to the target board
        scp ubuntu-24.04-server-arm64-rzv2h-rdk.img.bmap ubuntu@<rzv2h_rdk_ip>:/home/ubuntu/

   - Flash the root filesystem image to the SSD by running:

     .. code-block:: bash

        # Please change the device name if your SSD is recognized with a different name.
        sudo bmaptool copy ubuntu-24.04-server-arm64-rzv2h-rdk.img.xz /dev/nvme0n1

#. Configure the bootloader to boot from the SSD:

   - Open and edit the ``/boot/uEnv.txt`` file.
   - Change the ``mmc_args`` line to the following:

     .. code-block:: bash

        mmc_args=setenv bootargs 'rw rootwait earlycon root=/dev/nvme0n1p2'

   - Note that ``/dev/nvme0n1p2`` is the partition on the SSD where the root filesystem is located. If your SSD has a different partition layout, adjust the partition number accordingly.
   - Reboot the system to apply the changes.

#. Verify booting from the SSD:

   - Once the system boots up, log in.
   - Verify that the root filesystem is mounted from the SSD by checking the location of the root filesystem ``/``:

     .. code-block:: bash

        lsblk

#. Resize the filesystem if necessary:

   - If the SSD has a larger capacity than the original root filesystem image, you may want to resize the filesystem to use the full capacity of the SSD.
   - Use the following commands to resize the filesystem:

     .. code-block:: bash

        sudo apt update
        sudo apt install -y parted
        sudo parted /dev/nvme0n1 resizepart 2 100%
        sudo resize2fs /dev/nvme0n1p2