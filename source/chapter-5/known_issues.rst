Known Issues
------------

#. **USB camera warning and error messages when running AI applications**

   The following warning and error messages may appear in the terminal when running AI applications that use USB 2.0 cameras on the RZ/V2H RDK:

   .. code-block:: text

      [ 3495.792814] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
      [ 3495.801878] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
      [ 3495.810911] xhci-renesas-hcd 15860000.usb: ERROR Transfer event TRB DMA ptr not part of current TD ep_index 2 comp_code 13
      [ 3495.821917] xhci-renesas-hcd 15860000.usb: Looking for event-dma 000000004b0b1190 trb-start 000000004b0b11a0 trb-end 000000004b0b11a0 seg-start 000000004b0b1000 seg-end 000000004b0b1ff0
      [ 3496.032127] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
      [ 3496.041226] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2
      [ 3496.299376] xhci-renesas-hcd 15860000.usb: WARN: HC couldn't access mem fast enough for slot 1 ep 2

   These messages do not affect the functionality of the USB camera or the performance of the AI applications.
   The applications continue to operate as expected.

   .. note::

      This issue does not occur with USB 3.0 cameras on the RZ/V2H RDK.

   **Workaround**: change the IPL to the experiment version, and then the warning and error messages will not appear when running AI applications that use USB 2.0 cameras on the RZ/V2H RDK.

   However, we do not warrant that the experiment version IPL is stable, so please use it at your own risk.

   You can download the experiment version IPL inside the `RZ/V2H RDK release package <https://www.renesas.com/en/design-resources/boards-kits/ws125-v2hrdkrefz>`_ the path is ``board_setup/experiment_ipl/``.

   You can use the following command to quickly flash the IPL to the board when using SD card boot mode. For xSPI boot mode, please refer to the :ref:`Quick Setup Guide <quick_setup_rdk_guide>`.

   .. code-block:: bash

      # On the Ubuntu host machine, navigate to the directory where the experiment version IPL is located.
      cd /path/to/experiment_ipl

      # Please replace /dev/sdX with the actual device name of your SD card.
      # You can use the `lsblk` command to list all block devices and identify the correct device name for your SD card.
      sudo dd if=bl2_bp_esd-rzv2h-rdk.bin of=/dev/sdX bs=512 seek=1 conv=notrunc status=progress
      sudo dd if=fip-rzv2h-rdk.bin of=/dev/sdX bs=512 seek=768 conv=notrunc status=progress
      sync

#. **Some SD cards may not work properly with the RZ/V2H RDK**

   Some SD cards may not function correctly with the RZ/V2H RDK, leading to issues such as failure to boot or read/write errors.

   To ensure compatibility, use **SD cards that support high-speed mode** from reputable brands such as SanDisk, Samsung, or Kingston.

#. **The Ethernet may not work properly when booting up the board**

   The following error may occur when booting Ubuntu 24.04 on the RZ/V2H RDK, causing no Internet connection even though the Ethernet cable is connected:

   .. code-block:: bash

      [   17.664297] dwc-eth-dwmac 15c30000.ethernet end0: __stmmac_open: Cannot attach to PHY (error: -110)

   To resolve this issue:

   #. Power off the board.
   #. Unplug the Ethernet cable.
   #. Power on the board.
   #. Wait until the system fully boots.
   #. Plug the Ethernet cable back in.